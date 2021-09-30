/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/* 
 * LDLT factorization related code of ThreadedEquationSolverLDLT 
 * Copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")
 */


#include <ode/common.h>
#include <ode/matrix.h>
#include <ode/matrix_coop.h>
#include "config.h"
#include "threaded_solver_ldlt.h"
#include "threading_base.h"
#include "resource_control.h"
#include "error.h"

#include "fastldltfactor_impl.h"


/*static */
void ThreadedEquationSolverLDLT::estimateCooperativeFactoringLDLTResourceRequirements(
    dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
    unsigned allowedThreadCount, unsigned rowCount)
{
    dxThreadingBase *threading = summaryRequirementsDescriptor->getrelatedThreading();
    unsigned limitedThreadCount = restrictFactoringLDLTAllowedThreadCount(threading, allowedThreadCount, rowCount);

    if (limitedThreadCount > 1)
    {
        doEstimateCooperativeFactoringLDLTResourceRequirementsValidated(summaryRequirementsDescriptor, allowedThreadCount, rowCount);
    }
}

/*static */
void ThreadedEquationSolverLDLT::cooperativelyFactorLDLT(
    dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
    dReal *A, dReal *d, unsigned rowCount, unsigned rowSkip)
{
    dAASSERT(rowCount != 0);

    dxThreadingBase *threading = resourceContainer->getThreadingInstance();
    unsigned limitedThreadCount = restrictFactoringLDLTAllowedThreadCount(threading, allowedThreadCount, rowCount);

    if (limitedThreadCount <= 1)
    {
        factorMatrixAsLDLT<FLDLT_D_STRIDE>(A, d, rowCount, rowSkip);
    }
    else
    {
        doCooperativelyFactorLDLTValidated(resourceContainer, limitedThreadCount, A, d, rowCount, rowSkip);
    }
}


/*static */
unsigned ThreadedEquationSolverLDLT::restrictFactoringLDLTAllowedThreadCount(
    dxThreadingBase *threading, unsigned allowedThreadCount, unsigned rowCount)
{
    unsigned limitedThreadCount = 1;

#if dCOOPERATIVE_ENABLED
    const unsigned int solvingBlockStep = FSL1S_BLOCK_SIZE; // Required by the implementation
    unsigned solvingMaximalBlockCount = deriveSolvingL1StripeBlockCount(rowCount, solvingBlockStep);
    dIASSERT(deriveSolvingL1StripeThreadCount(FLDLT_COOPERATIVE_BLOCK_COUNT_MINIMUM - 1, 2) > 1);

    if (solvingMaximalBlockCount >= FLDLT_COOPERATIVE_BLOCK_COUNT_MINIMUM)
    {
        limitedThreadCount = threading->calculateThreadingLimitedThreadCount(allowedThreadCount, false);
    }
#endif // #if dCOOPERATIVE_ENABLED

    return limitedThreadCount;
}

/*static */
void ThreadedEquationSolverLDLT::doEstimateCooperativeFactoringLDLTResourceRequirementsValidated(
    dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
    unsigned allowedThreadCount, unsigned rowCount)
{
    const unsigned int solvingBlockStep = FSL1S_BLOCK_SIZE; // Required by the implementation
    unsigned solvingTotalBlockCount = deriveSolvingL1StripeBlockCount(rowCount, solvingBlockStep);
    dIASSERT(solvingTotalBlockCount >= 1);

    unsigned solvingLastBlockIndex = solvingTotalBlockCount - 1;

    const unsigned factorizingBlockARows = FFL1S_REGULAR_A_ROWS;
    unsigned factorizingMaximalBlockCount = deriveScalingAndFactorizingL1StripeBlockCountFromSolvingBlockIndex(solvingLastBlockIndex, solvingBlockStep, factorizingBlockARows);

    unsigned blockSolvingMaximumThreads = deriveSolvingL1StripeThreadCount(solvingLastBlockIndex, allowedThreadCount);
    unsigned blockFactorizingMaximumThreads = deriveScalingAndFactorizingL1StripeThreadCount(factorizingMaximalBlockCount, allowedThreadCount);
    unsigned simultaneousCallCount = 1 // Final synchronization point
        + 2 // intermediate synchronization points
        + dMACRO_MAX(blockSolvingMaximumThreads, blockFactorizingMaximumThreads);

    FactorizationSolvingL1StripeMemoryEstimates solvingMemoryEstimates;
    FactorizationScalingAndFactorizingL1StripeMemoryEstimates scalingAndFactorizingEstimates;
    sizeint solvingMemoryRequired = estimateCooperativelySolvingL1Stripe_XMemoryRequirement(solvingTotalBlockCount, solvingMemoryEstimates);
    sizeint factorizingMemoryRequired = estimateCooperativelyScalingAndFactorizingL1Stripe_XMemoryRequirement(blockFactorizingMaximumThreads, scalingAndFactorizingEstimates);
    sizeint totalSizeRequired = solvingMemoryRequired + factorizingMemoryRequired;
    const unsigned memoryAlignmentRequired = ALLOCATION_DEFAULT_ALIGNMENT;

    unsigned featureRequirement = dxResourceRequirementDescriptor::STOCK_CALLWAIT_REQUIRED;
    summaryRequirementsDescriptor->mergeAnotherDescriptorIn(totalSizeRequired, memoryAlignmentRequired, simultaneousCallCount, featureRequirement);
}

/*static */
void ThreadedEquationSolverLDLT::doCooperativelyFactorLDLTValidated(
    dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
    dReal *A, dReal *d, unsigned rowCount, unsigned rowSkip)
{
    dIASSERT(allowedThreadCount > 1);

    const unsigned int solvingBlockStep = FSL1S_BLOCK_SIZE; // Required by the implementation
    unsigned solvingTotalBlockCount = deriveSolvingL1StripeBlockCount(rowCount, solvingBlockStep);
    dIASSERT(solvingTotalBlockCount >= 1);

    unsigned solvingLastBlockIndex = solvingTotalBlockCount - 1;

    const unsigned factorizingBlockARows = FFL1S_REGULAR_A_ROWS;
    unsigned factorizingMaximalBlockCount = deriveScalingAndFactorizingL1StripeBlockCountFromSolvingBlockIndex(solvingLastBlockIndex, solvingBlockStep, factorizingBlockARows);

    unsigned blockFactorizingMaximumThreads = deriveScalingAndFactorizingL1StripeThreadCount(factorizingMaximalBlockCount, allowedThreadCount);

    dCallWaitID completionWait = resourceContainer->getStockCallWait();
    dAASSERT(completionWait != NULL);

    FactorizationSolvingL1StripeMemoryEstimates solvingMemoryEstimates;
    FactorizationScalingAndFactorizingL1StripeMemoryEstimates scalingAndFactorizingEstimates;
    sizeint solvingMemoryRequired = estimateCooperativelySolvingL1Stripe_XMemoryRequirement(solvingTotalBlockCount, solvingMemoryEstimates);
    sizeint factorizingMemoryRequired = estimateCooperativelyScalingAndFactorizingL1Stripe_XMemoryRequirement(blockFactorizingMaximumThreads, scalingAndFactorizingEstimates);
    sizeint totalSizeRequired = solvingMemoryRequired + factorizingMemoryRequired;
    dIASSERT(totalSizeRequired <= resourceContainer->getMemoryBufferSize());

    void *bufferAllocated = resourceContainer->getMemoryBufferPointer();
    dIASSERT(bufferAllocated != NULL);
    dIASSERT(dALIGN_PTR(bufferAllocated, ALLOCATION_DEFAULT_ALIGNMENT) == bufferAllocated);

    atomicord32 solvingBlockCompletionProgress;
    cellindexint *solvingBlockProgressDescriptors;
    FactorizationSolveL1StripeCellContext *solvingCellContexts;

    FactorizationFactorizeL1StripeContext *factorizingFactorizationContext;

    void *bufferCurrentLocation = bufferAllocated;
    bufferCurrentLocation = markCooperativelySolvingL1Stripe_XMemoryStructuresOut(bufferCurrentLocation, solvingMemoryEstimates, solvingBlockProgressDescriptors, solvingCellContexts);
    bufferCurrentLocation = markCooperativelyScalingAndFactorizingL1Stripe_XMemoryStructuresOut(bufferCurrentLocation, scalingAndFactorizingEstimates, factorizingFactorizationContext);
    dIVERIFY(bufferCurrentLocation <= (uint8 *)bufferAllocated + totalSizeRequired);

    dCallReleaseeID calculationFinishReleasee;
    dxThreadingBase *threading = resourceContainer->getThreadingInstance();
    threading->PostThreadedCall(NULL, &calculationFinishReleasee, 1, NULL, completionWait, &factotLDLT_completion_callback, NULL, 0, "FactorLDLT Completion");

    FactorLDLTWorkerContext workerContext(threading, allowedThreadCount, A, d, solvingTotalBlockCount, rowCount, rowSkip, 
        solvingBlockCompletionProgress, solvingBlockProgressDescriptors, solvingCellContexts, 
        factorizingFactorizationContext,
        calculationFinishReleasee); // The variable must exist in the outer scope

    dIASSERT(solvingTotalBlockCount >= FLDLT_COOPERATIVE_BLOCK_COUNT_MINIMUM);
    dSASSERT(FLDLT_COOPERATIVE_BLOCK_COUNT_MINIMUM > 2);

    scaleAndFactorizeL1FirstRowStripe_2<FLDLT_D_STRIDE>(workerContext.m_ARow, workerContext.m_d, workerContext.m_rowSkip);
    workerContext.incrementForNextBlock();

    const unsigned blockIndex = 1;
    dIASSERT(blockIndex == workerContext.m_solvingBlockIndex);

    initializeCooperativelySolvingL1Stripe_XMemoryStructures(blockIndex, solvingBlockCompletionProgress, solvingBlockProgressDescriptors, solvingCellContexts);
    unsigned secondBlockSolvingThreadCount = deriveSolvingL1StripeThreadCount(blockIndex, allowedThreadCount);

    dCallReleaseeID secondBlockSolvingSyncReleasee;
    threading->PostThreadedCall(NULL, &secondBlockSolvingSyncReleasee, secondBlockSolvingThreadCount, NULL, NULL, &factotLDLT_solvingCompleteSync_callback, &workerContext, 0, "FactorLDLT Solving Complete Sync");
    
    if (secondBlockSolvingThreadCount > 1)
    {
        threading->PostThreadedCallsGroup(NULL, secondBlockSolvingThreadCount - 1, secondBlockSolvingSyncReleasee, &factotLDLT_solvingComplete_callback, &workerContext, "FactorLDLT Solving Complete");
    }

    factotLDLT_solvingComplete(workerContext, secondBlockSolvingThreadCount - 1);
    threading->AlterThreadedCallDependenciesCount(secondBlockSolvingSyncReleasee, -1);

    threading->WaitThreadedCallExclusively(NULL, completionWait, NULL, "FactorLDLT End Wait");
}


/*static */
int ThreadedEquationSolverLDLT::factotLDLT_solvingComplete_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID dUNUSED(callThisReleasee))
{
    FactorLDLTWorkerContext *ptrContext = (FactorLDLTWorkerContext *)callContext;

    factotLDLT_solvingComplete(*ptrContext, dCAST_TO_SMALLER(unsigned, callInstanceIndex));
    
    return 1;
}

/*static */
void ThreadedEquationSolverLDLT::factotLDLT_solvingComplete(FactorLDLTWorkerContext &ref_context, unsigned ownThreadIndex)
{
    participateSolvingL1Stripe_X<FSL1S_BLOCK_SIZE, FSL1S_REGULAR_B_ROWS>(ref_context.m_A, ref_context.m_ARow, ref_context.m_solvingBlockIndex, ref_context.m_rowSkip, 
        ref_context.m_refSolvingBlockCompletionProgress, ref_context.m_solvingBlockProgressDescriptors, ref_context.m_solvingCellContexts, ownThreadIndex);
}


/*static */
int ThreadedEquationSolverLDLT::factotLDLT_solvingCompleteSync_callback(void *callContext, dcallindex_t dUNUSED(callInstanceIndex), dCallReleaseeID dUNUSED(callThisReleasee))
{
    FactorLDLTWorkerContext *ptrContext = (FactorLDLTWorkerContext *)callContext;

    factotLDLT_solvingCompleteSync(*ptrContext);

    return 1;
}

/*static */
void ThreadedEquationSolverLDLT::factotLDLT_solvingCompleteSync(FactorLDLTWorkerContext &ref_workerContext)
{
    unsigned solvingBlockIndex = ref_workerContext.m_solvingBlockIndex;
    FactorizationFactorizeL1StripeContext *factorizingFactorizationContext = ref_workerContext.m_factorizingFactorizationContext;

    const unsigned int solvingBlockStep = FSL1S_BLOCK_SIZE;
    const unsigned factorizingBlockARows = FFL1S_REGULAR_A_ROWS;
    unsigned factorizingBlockCount = deriveScalingAndFactorizingL1StripeBlockCountFromSolvingBlockIndex(solvingBlockIndex, solvingBlockStep, factorizingBlockARows);
    unsigned blockFactorizingThreadCount = deriveScalingAndFactorizingL1StripeThreadCount(factorizingBlockCount, ref_workerContext.m_allowedThreadCount);
    initializeCooperativelyScalingAndFactorizingL1Stripe_XMemoryStructures(factorizingFactorizationContext, blockFactorizingThreadCount);

    dCallReleaseeID blockFactorizingSyncReleasee;

    dxThreadingBase *threading = ref_workerContext.m_threading;
    if (solvingBlockIndex != ref_workerContext.m_totalBlockCount - 1)
    {
        threading->PostThreadedCall(NULL, &blockFactorizingSyncReleasee, blockFactorizingThreadCount, NULL, NULL, &factotLDLT_scalingAndFactorizingCompleteSync_callback, &ref_workerContext, 0, "FactorLDLT S'n'F Sync");
    }
    else
    {
        blockFactorizingSyncReleasee = ref_workerContext.m_calculationFinishReleasee;

        if (blockFactorizingThreadCount > 1)
        {
            threading->AlterThreadedCallDependenciesCount(blockFactorizingSyncReleasee, blockFactorizingThreadCount - 1);
        }
    }

    if (blockFactorizingThreadCount > 1)
    {
        threading->PostThreadedCallsGroup(NULL, blockFactorizingThreadCount - 1, blockFactorizingSyncReleasee, &factotLDLT_scalingAndFactorizingComplete_callback, &ref_workerContext, "FactorLDLT S'n'F Complete");
    }

    factotLDLT_scalingAndFactorizingComplete(ref_workerContext, blockFactorizingThreadCount - 1);
    threading->AlterThreadedCallDependenciesCount(blockFactorizingSyncReleasee, -1);
}


/*static */
int ThreadedEquationSolverLDLT::factotLDLT_scalingAndFactorizingComplete_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID dUNUSED(callThisReleasee))
{
    FactorLDLTWorkerContext *ptrContext = (FactorLDLTWorkerContext *)callContext;

    factotLDLT_scalingAndFactorizingComplete(*ptrContext, dCAST_TO_SMALLER(unsigned, callInstanceIndex));

    return 1;
}

/*static */
void ThreadedEquationSolverLDLT::factotLDLT_scalingAndFactorizingComplete(FactorLDLTWorkerContext &ref_workerContext, unsigned ownThreadIndex)
{
    unsigned factorizationRow = ref_workerContext.m_solvingBlockIndex * FSL1S_BLOCK_SIZE;
    participateScalingAndFactorizingL1Stripe_X<FFL1S_REGULAR_A_ROWS, FLDLT_D_STRIDE>(ref_workerContext.m_ARow, ref_workerContext.m_d, factorizationRow, 
        ref_workerContext.m_rowSkip, ref_workerContext.m_factorizingFactorizationContext, ownThreadIndex);
}


/*static */
int ThreadedEquationSolverLDLT::factotLDLT_scalingAndFactorizingCompleteSync_callback(void *callContext, dcallindex_t dUNUSED(callInstanceIndex), dCallReleaseeID dUNUSED(callThisReleasee))
{
    FactorLDLTWorkerContext *ptrContext = (FactorLDLTWorkerContext *)callContext;

    factotLDLT_scalingAndFactorizingCompleteSync(*ptrContext);

    return 1;
}

/*static */
void ThreadedEquationSolverLDLT::factotLDLT_scalingAndFactorizingCompleteSync(FactorLDLTWorkerContext &ref_workerContext)
{
    ref_workerContext.incrementForNextBlock();

    unsigned blockIndex = ref_workerContext.m_solvingBlockIndex;
    dIASSERT(blockIndex < ref_workerContext.m_totalBlockCount);

    atomicord32 &refSolvingBlockCompletionProgress = ref_workerContext.m_refSolvingBlockCompletionProgress;
    cellindexint *solvingBlockProgressDescriptors = ref_workerContext.m_solvingBlockProgressDescriptors;
    FactorizationSolveL1StripeCellContext *solvingCellContexts = ref_workerContext.m_solvingCellContexts;

    initializeCooperativelySolvingL1Stripe_XMemoryStructures(blockIndex, refSolvingBlockCompletionProgress, solvingBlockProgressDescriptors, solvingCellContexts);
    unsigned blockSolvingThreadCount = deriveSolvingL1StripeThreadCount(blockIndex, ref_workerContext.m_allowedThreadCount);

    dCallReleaseeID blockSolvingSyncReleasee;

    dxThreadingBase *threading = ref_workerContext.m_threading;
    if (blockIndex != ref_workerContext.m_totalBlockCount - 1 || ref_workerContext.m_rowCount % FSL1S_REGULAR_B_ROWS == 0)
    {
        threading->PostThreadedCall(NULL, &blockSolvingSyncReleasee, blockSolvingThreadCount, NULL, NULL, &factotLDLT_solvingCompleteSync_callback, &ref_workerContext, 0, "FactorLDLT Solving Complete Sync");

        if (blockSolvingThreadCount > 1)
        {
            threading->PostThreadedCallsGroup(NULL, blockSolvingThreadCount - 1, blockSolvingSyncReleasee, &factotLDLT_solvingComplete_callback, &ref_workerContext, "FactorLDLT Solving Complete");
        }

        factotLDLT_solvingComplete(ref_workerContext, blockSolvingThreadCount - 1);
    }
    else
    {
        dSASSERT(FSL1S_REGULAR_B_ROWS == 2);
        dSASSERT(FSL1S_FINAL_B_ROWS == 1);

        threading->PostThreadedCall(NULL, &blockSolvingSyncReleasee, blockSolvingThreadCount, NULL, NULL, &factotLDLT_solvingFinalSync_callback, &ref_workerContext, 0, "FactorLDLT Solving Final Sync");

        if (blockSolvingThreadCount > 1)
        {
            threading->PostThreadedCallsGroup(NULL, blockSolvingThreadCount - 1, blockSolvingSyncReleasee, &factotLDLT_solvingFinal_callback, &ref_workerContext, "FactorLDLT Solving Final");
        }

        factotLDLT_solvingFinal(ref_workerContext, blockSolvingThreadCount - 1);
    }

    threading->AlterThreadedCallDependenciesCount(blockSolvingSyncReleasee, -1);
}


/*static */
int ThreadedEquationSolverLDLT::factotLDLT_solvingFinal_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID dUNUSED(callThisReleasee))
{
    FactorLDLTWorkerContext *ptrContext = (FactorLDLTWorkerContext *)callContext;

    factotLDLT_solvingFinal(*ptrContext, dCAST_TO_SMALLER(unsigned, callInstanceIndex));

    return 1;
}

/*static */
void ThreadedEquationSolverLDLT::factotLDLT_solvingFinal(FactorLDLTWorkerContext &ref_context, unsigned ownThreadIndex)
{
    participateSolvingL1Stripe_X<FSL1S_BLOCK_SIZE, FSL1S_FINAL_B_ROWS>(ref_context.m_A, ref_context.m_ARow, ref_context.m_solvingBlockIndex, ref_context.m_rowSkip, 
        ref_context.m_refSolvingBlockCompletionProgress, ref_context.m_solvingBlockProgressDescriptors, ref_context.m_solvingCellContexts, ownThreadIndex);
}


/*static */
int ThreadedEquationSolverLDLT::factotLDLT_solvingFinalSync_callback(void *callContext, dcallindex_t dUNUSED(callInstanceIndex), dCallReleaseeID dUNUSED(callThisReleasee))
{
    FactorLDLTWorkerContext *ptrContext = (FactorLDLTWorkerContext *)callContext;

    factotLDLT_solvingFinalSync(*ptrContext);

    return 1;
}

/*static */
void ThreadedEquationSolverLDLT::factotLDLT_solvingFinalSync(FactorLDLTWorkerContext &ref_workerContext)
{
    unsigned solvingBlockIndex = ref_workerContext.m_solvingBlockIndex;
    FactorizationFactorizeL1StripeContext *factorizingFactorizationContext = ref_workerContext.m_factorizingFactorizationContext;

    const unsigned int solvingBlockStep = FSL1S_BLOCK_SIZE;
    const unsigned factorizingBlockARows = FFL1S_FINAL_A_ROWS;
    unsigned factorizingBlockCount = deriveScalingAndFactorizingL1StripeBlockCountFromSolvingBlockIndex(solvingBlockIndex, solvingBlockStep, factorizingBlockARows);
    unsigned blockFactorizingThreadCount = deriveScalingAndFactorizingL1StripeThreadCount(factorizingBlockCount, ref_workerContext.m_allowedThreadCount);
    initializeCooperativelyScalingAndFactorizingL1Stripe_XMemoryStructures(factorizingFactorizationContext, blockFactorizingThreadCount);

    dCallReleaseeID blockFactorizingSyncReleasee = ref_workerContext.m_calculationFinishReleasee;
    dIASSERT(solvingBlockIndex == ref_workerContext.m_totalBlockCount - 1);

    dxThreadingBase *threading = ref_workerContext.m_threading;

    if (blockFactorizingThreadCount > 1)
    {
        threading->AlterThreadedCallDependenciesCount(blockFactorizingSyncReleasee, blockFactorizingThreadCount - 1);
        threading->PostThreadedCallsGroup(NULL, blockFactorizingThreadCount - 1, blockFactorizingSyncReleasee, &factotLDLT_scalingAndFactorizingFinal_callback, &ref_workerContext, "FactorLDLT S'n'F Final");
    }

    factotLDLT_scalingAndFactorizingFinal(ref_workerContext, blockFactorizingThreadCount - 1);
    threading->AlterThreadedCallDependenciesCount(blockFactorizingSyncReleasee, -1);
}


/*static */
int ThreadedEquationSolverLDLT::factotLDLT_scalingAndFactorizingFinal_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID dUNUSED(callThisReleasee))
{
    FactorLDLTWorkerContext *ptrContext = (FactorLDLTWorkerContext *)callContext;

    factotLDLT_scalingAndFactorizingFinal(*ptrContext, dCAST_TO_SMALLER(unsigned, callInstanceIndex));

    return 1;
}

/*static */
void ThreadedEquationSolverLDLT::factotLDLT_scalingAndFactorizingFinal(FactorLDLTWorkerContext &ref_workerContext, unsigned ownThreadIndex)
{
    unsigned factorizationRow = ref_workerContext.m_solvingBlockIndex * FSL1S_BLOCK_SIZE;
    participateScalingAndFactorizingL1Stripe_X<FFL1S_FINAL_A_ROWS, FLDLT_D_STRIDE>(ref_workerContext.m_ARow, ref_workerContext.m_d, factorizationRow, 
        ref_workerContext.m_rowSkip, ref_workerContext.m_factorizingFactorizationContext, ownThreadIndex);
}


/*static */
int ThreadedEquationSolverLDLT::factotLDLT_completion_callback(void *dUNUSED(callContext), dcallindex_t dUNUSED(callInstanceIndex), dCallReleaseeID dUNUSED(callThisReleasee))
{
    // Do nothing
    return 1;
}


//////////////////////////////////////////////////////////////////////////
// Public interface functions


/*extern ODE_API */
void dFactorLDLT(dReal *A, dReal *d, int n, int nskip1)
{
    factorMatrixAsLDLT<1>(A, d, n, nskip1);
}


/*extern ODE_API */
void dEstimateCooperativelyFactorLDLTResourceRequirements(dResourceRequirementsID requirements,
    unsigned maximalAllowedThreadCount, unsigned maximalRowCount)
{
    dAASSERT(requirements != NULL);

    dxResourceRequirementDescriptor *requirementsDescriptor = (dxResourceRequirementDescriptor *)requirements;
    ThreadedEquationSolverLDLT::estimateCooperativeFactoringLDLTResourceRequirements(requirementsDescriptor, maximalAllowedThreadCount, maximalRowCount);
}

/*extern ODE_API */
void dCooperativelyFactorLDLT(dResourceContainerID resources, unsigned allowedThreadCount, 
    dReal *A, dReal *d, unsigned rowCount, unsigned rowSkip)
{
    dAASSERT(resources != NULL);

    dxRequiredResourceContainer *resourceContainer = (dxRequiredResourceContainer *)resources;
    ThreadedEquationSolverLDLT::cooperativelyFactorLDLT(resourceContainer, allowedThreadCount, A, d, rowCount, rowSkip);
}
