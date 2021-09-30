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
 * L1Straight Equation Solving Routines
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

#include "fastlsolve_impl.h"


/*static */
void ThreadedEquationSolverLDLT::estimateCooperativeSolvingL1StraightResourceRequirements(
    dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
    unsigned allowedThreadCount, unsigned rowCount)
{
    dxThreadingBase *threading = summaryRequirementsDescriptor->getrelatedThreading();
    unsigned limitedThreadCount = restrictSolvingL1StraightAllowedThreadCount(threading, allowedThreadCount, rowCount);

    if (limitedThreadCount > 1)
    {
        doEstimateCooperativeSolvingL1StraightResourceRequirementsValidated(summaryRequirementsDescriptor, allowedThreadCount, rowCount);
    }
}

/*static */
void ThreadedEquationSolverLDLT::cooperativelySolveL1Straight(
    dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
    const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip)
{
    dAASSERT(rowCount != 0);

    dxThreadingBase *threading = resourceContainer->getThreadingInstance();
    unsigned limitedThreadCount = restrictSolvingL1StraightAllowedThreadCount(threading, allowedThreadCount, rowCount);

    if (limitedThreadCount <= 1)
    {
        solveL1Straight<SL1S_B_STRIDE>(L, b, rowCount, rowSkip);
    }
    else
    {
        doCooperativelySolveL1StraightValidated(resourceContainer, limitedThreadCount, L, b, rowCount, rowSkip);
    }
}


/*static */
unsigned ThreadedEquationSolverLDLT::restrictSolvingL1StraightAllowedThreadCount(
    dxThreadingBase *threading, unsigned allowedThreadCount, unsigned rowCount)
{
    unsigned limitedThreadCount = 1;

#if dCOOPERATIVE_ENABLED
    const unsigned int blockStep = SL1S_BLOCK_SIZE; // Required by the implementation
    unsigned solvingBlockCount = deriveSolvingL1StraightBlockCount(rowCount, blockStep);
    dIASSERT(deriveSolvingL1StraightThreadCount(SL1S_COOPERATIVE_BLOCK_COUNT_MINIMUM, 2) > 1);

    if (solvingBlockCount >= SL1S_COOPERATIVE_BLOCK_COUNT_MINIMUM)
    {
        limitedThreadCount = threading->calculateThreadingLimitedThreadCount(allowedThreadCount, true);
    }
#endif // #if dCOOPERATIVE_ENABLED

    return limitedThreadCount;
}

/*static */
void ThreadedEquationSolverLDLT::doEstimateCooperativeSolvingL1StraightResourceRequirementsValidated(
    dxResourceRequirementDescriptor *summaryRequirementsDescriptor, 
    unsigned allowedThreadCount, unsigned rowCount)
{
    const unsigned int blockStep = SL1S_BLOCK_SIZE; // Required by the implementation
    unsigned blockCount = deriveSolvingL1StraightBlockCount(rowCount, blockStep);
    dIASSERT(blockCount >= 1);

    unsigned threadCountToUse = deriveSolvingL1StraightThreadCount(blockCount, allowedThreadCount);
    dIASSERT(threadCountToUse > 1);

    unsigned simultaneousCallCount = 1 + (threadCountToUse - 1);

    SolvingL1StraightMemoryEstimates solvingMemoryEstimates;
    sizeint solvingMemoryRequired = estimateCooperativelySolvingL1StraightMemoryRequirement<blockStep>(rowCount, solvingMemoryEstimates);
    const unsigned solvingAlignmentRequired = ALLOCATION_DEFAULT_ALIGNMENT;

    unsigned featureRequirement = dxResourceRequirementDescriptor::STOCK_CALLWAIT_REQUIRED;
    summaryRequirementsDescriptor->mergeAnotherDescriptorIn(solvingMemoryRequired, solvingAlignmentRequired, simultaneousCallCount, featureRequirement);
}

/*static */
void ThreadedEquationSolverLDLT::doCooperativelySolveL1StraightValidated(
    dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
    const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip)
{
    dIASSERT(allowedThreadCount > 1);

    const unsigned int blockStep = SL1S_BLOCK_SIZE; // Required by the implementation
    unsigned blockCount = deriveSolvingL1StraightBlockCount(rowCount, blockStep);
    dIASSERT(blockCount >= 1);

    unsigned threadCountToUse = deriveSolvingL1StraightThreadCount(blockCount, allowedThreadCount);
    dIASSERT(threadCountToUse > 1);

    dCallWaitID completionWait = resourceContainer->getStockCallWait();
    dAASSERT(completionWait != NULL);

    atomicord32 blockCompletionProgress;
    cellindexint *blockProgressDescriptors;
    SolveL1StraightCellContext *cellContexts;

    SolvingL1StraightMemoryEstimates solvingMemoryEstimates;
    sizeint solvingMemoryRequired = estimateCooperativelySolvingL1StraightMemoryRequirement<blockStep>(rowCount, solvingMemoryEstimates);
    dIASSERT(solvingMemoryRequired <= resourceContainer->getMemoryBufferSize());

    void *bufferAllocated = resourceContainer->getMemoryBufferPointer();
    dIASSERT(bufferAllocated != NULL);
    dIASSERT(dALIGN_PTR(bufferAllocated, ALLOCATION_DEFAULT_ALIGNMENT) == bufferAllocated);

    void *bufferCurrentLocation = bufferAllocated;
    bufferCurrentLocation = markCooperativelySolvingL1StraightMemoryStructuresOut(bufferCurrentLocation, solvingMemoryEstimates, blockProgressDescriptors, cellContexts);
    dIVERIFY(bufferCurrentLocation <= (uint8 *)bufferAllocated + solvingMemoryRequired);

    initializeCooperativelySolveL1StraightMemoryStructures<blockStep>(rowCount, blockCompletionProgress, blockProgressDescriptors, cellContexts);

    dCallReleaseeID calculationFinishReleasee;
    SolveL1StraightWorkerContext workerContext; // The variable must exist in the outer scope

    workerContext.init(L, b, rowCount, rowSkip, blockCompletionProgress, blockProgressDescriptors, cellContexts);

    dxThreadingBase *threading = resourceContainer->getThreadingInstance();
    threading->PostThreadedCall(NULL, &calculationFinishReleasee, threadCountToUse - 1, NULL, completionWait, &solveL1Straight_completion_callback, NULL, 0, "SolveL1Straight Completion");
    threading->PostThreadedCallsGroup(NULL, threadCountToUse - 1, calculationFinishReleasee, &solveL1Straight_worker_callback, &workerContext, "SolveL1Straight Work");

    participateSolvingL1Straight<blockStep, SL1S_B_STRIDE>(L, b, rowCount, rowSkip, blockCompletionProgress, blockProgressDescriptors, cellContexts, threadCountToUse - 1);

    threading->WaitThreadedCallExclusively(NULL, completionWait, NULL, "SolveL1Straight End Wait");
}

/*static */
int ThreadedEquationSolverLDLT::solveL1Straight_worker_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID dUNUSED(callThisReleasee))
{
    SolveL1StraightWorkerContext *ptrContext = (SolveL1StraightWorkerContext *)callContext;

    solveL1Straight_worker(*ptrContext, dCAST_TO_SMALLER(unsigned, callInstanceIndex));

    return 1;
}

/*static */
void ThreadedEquationSolverLDLT::solveL1Straight_worker(SolveL1StraightWorkerContext &ref_context, unsigned ownThreadIndex)
{
    const unsigned blockStep = SL1S_BLOCK_SIZE;

    participateSolvingL1Straight<blockStep, SL1S_B_STRIDE>(ref_context.m_L, ref_context.m_b, ref_context.m_rowCount, ref_context.m_rowSkip, 
        *ref_context.m_ptrBlockCompletionProgress, ref_context.m_blockProgressDescriptors, ref_context.m_cellContexts, ownThreadIndex);
}

/*static */
int ThreadedEquationSolverLDLT::solveL1Straight_completion_callback(void *dUNUSED(callContext), dcallindex_t dUNUSED(callInstanceIndex), dCallReleaseeID dUNUSED(callThisReleasee))
{
    return 1;
}



//////////////////////////////////////////////////////////////////////////
// Public interface functions

/*extern ODE_API */
void dSolveL1(const dReal *L, dReal *B, int n, int lskip1)
{
    dAASSERT(n != 0);

    if (n != 0)
    {
        dAASSERT(L != NULL);
        dAASSERT(B != NULL);

        solveL1Straight<1>(L, B, n, lskip1);
    }
}


/*extern ODE_API */
void dEstimateCooperativelySolveL1StraightResourceRequirements(dResourceRequirementsID requirements,
    unsigned maximalAllowedThreadCount, unsigned maximalRowCount)
{
    dAASSERT(requirements != NULL);

    dxResourceRequirementDescriptor *requirementsDescriptor = (dxResourceRequirementDescriptor *)requirements;
    ThreadedEquationSolverLDLT::estimateCooperativeSolvingL1StraightResourceRequirements(requirementsDescriptor, maximalAllowedThreadCount, maximalRowCount);
}

/*extern ODE_API */
void dCooperativelySolveL1Straight(dResourceContainerID resources, unsigned allowedThreadCount, 
    const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip)
{
    dAASSERT(resources != NULL);

    dxRequiredResourceContainer *resourceContainer = (dxRequiredResourceContainer *)resources;
    ThreadedEquationSolverLDLT::cooperativelySolveL1Straight(resourceContainer, allowedThreadCount, L, b, rowCount, rowSkip);
}

