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
 * L1Transposed Equation Solving Routines
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

#include "fastltsolve_impl.h"


/*static */
void ThreadedEquationSolverLDLT::estimateCooperativeSolvingL1TransposedResourceRequirements(
    dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
    unsigned allowedThreadCount, unsigned rowCount)
{
    dxThreadingBase *threading = summaryRequirementsDescriptor->getrelatedThreading();
    unsigned limitedThreadCount = restrictSolvingL1TransposedAllowedThreadCount(threading, allowedThreadCount, rowCount);

    if (limitedThreadCount > 1)
    {
        doEstimateCooperativeSolvingL1TransposedResourceRequirementsValidated(summaryRequirementsDescriptor, allowedThreadCount, rowCount);
    }
}

/*static */
void ThreadedEquationSolverLDLT::cooperativelySolveL1Transposed(
    dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
    const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip)
{
    dIASSERT(rowCount != 0);

    dxThreadingBase *threading = resourceContainer->getThreadingInstance();
    unsigned limitedThreadCount = restrictSolvingL1TransposedAllowedThreadCount(threading, allowedThreadCount, rowCount);

    if (limitedThreadCount <= 1)
    {
        solveL1Transposed<SL1T_B_STRIDE>(L, b, rowCount, rowSkip);
    }
    else
    {
        doCooperativelySolveL1TransposedValidated(resourceContainer, limitedThreadCount, L, b, rowCount, rowSkip);
    }
}


/*static */
unsigned ThreadedEquationSolverLDLT::restrictSolvingL1TransposedAllowedThreadCount(
    dxThreadingBase *threading, unsigned allowedThreadCount, unsigned rowCount)
{
    unsigned limitedThreadCount = 1;

#if dCOOPERATIVE_ENABLED
    const unsigned int blockStep = SL1T_BLOCK_SIZE; // Required by the implementation
    unsigned solvingBlockCount = deriveSolvingL1TransposedBlockCount(rowCount, blockStep);
    dIASSERT(deriveSolvingL1TransposedThreadCount(SL1T_COOPERATIVE_BLOCK_COUNT_MINIMUM, 2) > 1);

    if (solvingBlockCount >= SL1T_COOPERATIVE_BLOCK_COUNT_MINIMUM)
    {
        limitedThreadCount = threading->calculateThreadingLimitedThreadCount(allowedThreadCount, true);
    }
#endif // #if dCOOPERATIVE_ENABLED

    return limitedThreadCount;
}

/*static */
void ThreadedEquationSolverLDLT::doEstimateCooperativeSolvingL1TransposedResourceRequirementsValidated(
    dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
    unsigned allowedThreadCount, unsigned rowCount)
{
    const unsigned int blockStep = SL1T_BLOCK_SIZE; // Required by the implementation
    unsigned blockCount = deriveSolvingL1TransposedBlockCount(rowCount, blockStep);
    dIASSERT(blockCount >= 1);

    unsigned threadCountToUse = deriveSolvingL1TransposedThreadCount(blockCount, allowedThreadCount);
    dIASSERT(threadCountToUse > 1);

    unsigned simultaneousCallCount = 1 + (threadCountToUse - 1);

    SolvingL1TransposedMemoryEstimates solvingMemoryEstimates;
    sizeint solvingMemoryRequired = estimateCooperativelySolvingL1TransposedMemoryRequirement<blockStep>(rowCount, solvingMemoryEstimates);
    const unsigned solvingAlignmentRequired = ALLOCATION_DEFAULT_ALIGNMENT;

    unsigned featureRequirement = dxResourceRequirementDescriptor::STOCK_CALLWAIT_REQUIRED;
    summaryRequirementsDescriptor->mergeAnotherDescriptorIn(solvingMemoryRequired, solvingAlignmentRequired, simultaneousCallCount, featureRequirement);
}

/*static */
void ThreadedEquationSolverLDLT::doCooperativelySolveL1TransposedValidated(
    dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
    const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip)
{
    dIASSERT(allowedThreadCount > 1);

    const unsigned int blockStep = SL1T_BLOCK_SIZE; // Required by the implementation
    unsigned blockCount = deriveSolvingL1TransposedBlockCount(rowCount, blockStep);
    dIASSERT(blockCount >= 1);

    unsigned threadCountToUse = deriveSolvingL1TransposedThreadCount(blockCount, allowedThreadCount);
    dIASSERT(threadCountToUse > 1);

    dCallWaitID completionWait = resourceContainer->getStockCallWait();
    dAASSERT(completionWait != NULL);

    atomicord32 blockCompletionProgress;
    cellindexint *blockProgressDescriptors;
    SolveL1TransposedCellContext *cellContexts;

    SolvingL1TransposedMemoryEstimates solvingMemoryEstimates;
    sizeint solvingMemoryRequired = estimateCooperativelySolvingL1TransposedMemoryRequirement<blockStep>(rowCount, solvingMemoryEstimates);
    dIASSERT(solvingMemoryRequired <= resourceContainer->getMemoryBufferSize());

    void *bufferAllocated = resourceContainer->getMemoryBufferPointer();
    dIASSERT(bufferAllocated != NULL);
    dIASSERT(dALIGN_PTR(bufferAllocated, ALLOCATION_DEFAULT_ALIGNMENT) == bufferAllocated);

    void *bufferCurrentLocation = bufferAllocated;
    bufferCurrentLocation = markCooperativelySolvingL1TransposedMemoryStructuresOut(bufferCurrentLocation, solvingMemoryEstimates, blockProgressDescriptors, cellContexts);
    dIVERIFY(bufferCurrentLocation <= (uint8 *)bufferAllocated + solvingMemoryRequired);

    initializeCooperativelySolveL1TransposedMemoryStructures<blockStep>(rowCount, blockCompletionProgress, blockProgressDescriptors, cellContexts);

    dCallReleaseeID calculationFinishReleasee;
    SolveL1TransposedWorkerContext workerContext; // The variable must exist in the outer scope

    workerContext.init(L, b, rowCount, rowSkip, blockCompletionProgress, blockProgressDescriptors, cellContexts);

    dxThreadingBase *threading = resourceContainer->getThreadingInstance();
    threading->PostThreadedCall(NULL, &calculationFinishReleasee, threadCountToUse - 1, NULL, completionWait, &solveL1Transposed_completion_callback, NULL, 0, "SolveL1Transposed Completion");
    threading->PostThreadedCallsGroup(NULL, threadCountToUse - 1, calculationFinishReleasee, &solveL1Transposed_worker_callback, &workerContext, "SolveL1Transposed Work");

    participateSolvingL1Transposed<blockStep, SL1T_B_STRIDE>(L, b, rowCount, rowSkip, blockCompletionProgress, blockProgressDescriptors, cellContexts, threadCountToUse - 1);

    threading->WaitThreadedCallExclusively(NULL, completionWait, NULL, "SolveL1Transposed End Wait");
}

/*static */
int ThreadedEquationSolverLDLT::solveL1Transposed_worker_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID dUNUSED(callThisReleasee))
{
    SolveL1TransposedWorkerContext *ptrContext = (SolveL1TransposedWorkerContext *)callContext;

    solveL1Transposed_worker(*ptrContext, dCAST_TO_SMALLER(unsigned, callInstanceIndex));
    
    return 1;
}

/*static */
void ThreadedEquationSolverLDLT::solveL1Transposed_worker(SolveL1TransposedWorkerContext &ref_context, unsigned ownThreadIndex)
{
    const unsigned blockStep = SL1T_BLOCK_SIZE;
    participateSolvingL1Transposed<blockStep, SL1T_B_STRIDE>(ref_context.m_L, ref_context.m_b, ref_context.m_rowCount, ref_context.m_rowSkip, 
        *ref_context.m_ptrBlockCompletionProgress, ref_context.m_blockProgressDescriptors, ref_context.m_cellContexts, ownThreadIndex);
}

/*static */
int ThreadedEquationSolverLDLT::solveL1Transposed_completion_callback(void *dUNUSED(callContext), dcallindex_t dUNUSED(callInstanceIndex), dCallReleaseeID dUNUSED(callThisReleasee))
{
    return 1;
}



//////////////////////////////////////////////////////////////////////////
// Public interface functions

/*extern ODE_API */
void dSolveL1T(const dReal *L, dReal *B, int rowCount, int rowSkip)
{
    dAASSERT(rowCount != 0);

    if (rowCount != 0)
    {
        dAASSERT(L != NULL);
        dAASSERT(B != NULL);

        solveL1Transposed<1>(L, B, rowCount, rowSkip);
    }
}


/*extern ODE_API */
void dEstimateCooperativelySolveL1TransposedResourceRequirements(dResourceRequirementsID requirements, 
    unsigned maximalAllowedThreadCount, unsigned maximalRowCount)
{
    dAASSERT(requirements != NULL);

    dxResourceRequirementDescriptor *requirementsDescriptor = (dxResourceRequirementDescriptor *)requirements;
    ThreadedEquationSolverLDLT::estimateCooperativeSolvingL1TransposedResourceRequirements(requirementsDescriptor, maximalAllowedThreadCount, maximalRowCount);
}

/*extern ODE_API */
void dCooperativelySolveL1Transposed(dResourceContainerID resources, unsigned allowedThreadCount, 
    const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip)
{
    dAASSERT(resources != NULL);

    dxRequiredResourceContainer *resourceContainer = (dxRequiredResourceContainer *)resources;
    ThreadedEquationSolverLDLT::cooperativelySolveL1Transposed(resourceContainer, allowedThreadCount, L, b, rowCount, rowSkip);
}

