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
 * Vector scaling related code of ThreadedEquationSolverLDLT 
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

#include "fastvecscale_impl.h"


/*static */
void ThreadedEquationSolverLDLT::estimateCooperativeScalingVectorResourceRequirements(
    dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
    unsigned allowedThreadCount, unsigned elementCount)
{
    dxThreadingBase *threading = summaryRequirementsDescriptor->getrelatedThreading();
    unsigned limitedThreadCount = restrictScalingVectorAllowedThreadCount(threading, allowedThreadCount, elementCount);

    if (limitedThreadCount > 1)
    {
        doEstimateCooperativeScalingVectorResourceRequirementsValidated(summaryRequirementsDescriptor, allowedThreadCount, elementCount);
    }
}

/*static */
void ThreadedEquationSolverLDLT::cooperativelyScaleVector(dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
    dReal *vectorData, const dReal *scaleData, unsigned elementCount)
{
    dAASSERT(elementCount != 0);

    dxThreadingBase *threading = resourceContainer->getThreadingInstance();
    unsigned limitedThreadCount = restrictScalingVectorAllowedThreadCount(threading, allowedThreadCount, elementCount);

    if (limitedThreadCount <= 1)
    {
        scaleLargeVector<SV_A_STRIDE, SV_D_STRIDE>(vectorData, scaleData, elementCount);
    }
    else
    {
        doCooperativelyScaleVectorValidated(resourceContainer, limitedThreadCount, vectorData, scaleData, elementCount);
    }
}

/*static */
unsigned ThreadedEquationSolverLDLT::restrictScalingVectorAllowedThreadCount(
    dxThreadingBase *threading, unsigned allowedThreadCount, unsigned elementCount)
{
    unsigned limitedThreadCount = 1;

#if dCOOPERATIVE_ENABLED
    const unsigned int blockStep = SV_BLOCK_SIZE; // Required by the implementation
    unsigned scalingBlockCount = deriveScalingVectorBlockCount(elementCount, blockStep);
    dIASSERT(deriveScalingVectorThreadCount(SV_COOPERATIVE_BLOCK_COUNT_MINIMUM - 1, 2) > 1);

    if (scalingBlockCount >= SV_COOPERATIVE_BLOCK_COUNT_MINIMUM)
    {
        limitedThreadCount = threading->calculateThreadingLimitedThreadCount(allowedThreadCount, true);
    }
#endif // #if dCOOPERATIVE_ENABLED

    return limitedThreadCount;
}

/*static */
void ThreadedEquationSolverLDLT::doEstimateCooperativeScalingVectorResourceRequirementsValidated(
    dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
    unsigned allowedThreadCount, unsigned elementCount)
{
    unsigned simultaneousCallCount = 1 + (allowedThreadCount - 1);

    sizeint scalingMemoryRequired = 0;
    const unsigned scalingAlignmentRequired = 0;

    unsigned featureRequirement = dxResourceRequirementDescriptor::STOCK_CALLWAIT_REQUIRED;
    summaryRequirementsDescriptor->mergeAnotherDescriptorIn(scalingMemoryRequired, scalingAlignmentRequired, simultaneousCallCount, featureRequirement);
}

/*static */
void ThreadedEquationSolverLDLT::doCooperativelyScaleVectorValidated(
    dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
    dReal *vectorData, const dReal *scaleData, unsigned elementCount)
{
    dIASSERT(allowedThreadCount > 1);

    const unsigned int blockStep = SV_BLOCK_SIZE; // Required by the implementation
    unsigned scalingBlockCount = deriveScalingVectorBlockCount(elementCount, blockStep);
    dIASSERT(scalingBlockCount > 0U);

    unsigned threadCountToUse = deriveScalingVectorThreadCount(scalingBlockCount - 1, allowedThreadCount);
    dIASSERT(threadCountToUse > 1);

    dCallWaitID completionWait = resourceContainer->getStockCallWait();
    dAASSERT(completionWait != NULL);

    atomicord32 blockCompletionProgress;

    initializeCooperativelyScaleVectorMemoryStructures(blockCompletionProgress);

    dCallReleaseeID calculationFinishReleasee;
    ScaleVectorWorkerContext workerContext; // The variable must exist in the outer scope

    workerContext.init(vectorData, scaleData, elementCount, blockCompletionProgress);

    dxThreadingBase *threading = resourceContainer->getThreadingInstance();
    threading->PostThreadedCall(NULL, &calculationFinishReleasee, threadCountToUse - 1, NULL, completionWait, &scaleVector_completion_callback, NULL, 0, "ScaleVector Completion");
    threading->PostThreadedCallsGroup(NULL, threadCountToUse - 1, calculationFinishReleasee, &scaleVector_worker_callback, &workerContext, "ScaleVector Work");

    participateScalingVector<blockStep, SV_A_STRIDE, SV_D_STRIDE>(vectorData, scaleData, elementCount, blockCompletionProgress);

    threading->WaitThreadedCallExclusively(NULL, completionWait, NULL, "ScaleVector End Wait");
}


/*static */
int ThreadedEquationSolverLDLT::scaleVector_worker_callback(void *callContext, dcallindex_t dUNUSED(callInstanceIndex), dCallReleaseeID dUNUSED(callThisReleasee))
{
    ScaleVectorWorkerContext *ptrContext = (ScaleVectorWorkerContext *)callContext;

    scaleVector_worker(*ptrContext);

    return 1;
}

/*static */
void ThreadedEquationSolverLDLT::scaleVector_worker(ScaleVectorWorkerContext &ref_context)
{
    const unsigned blockStep = SV_BLOCK_SIZE;

    participateScalingVector<blockStep, SV_A_STRIDE, SV_D_STRIDE>(ref_context.m_vectorData, ref_context.m_scaleData, ref_context.m_elementCount, *ref_context.m_ptrBlockCompletionProgress);
}

/*static */
int ThreadedEquationSolverLDLT::scaleVector_completion_callback(void *dUNUSED(callContext), dcallindex_t dUNUSED(callInstanceIndex), dCallReleaseeID dUNUSED(callThisReleasee))
{
    return 1;
}


//////////////////////////////////////////////////////////////////////////
// Public interface functions

/*extern ODE_API */
void dScaleVector(dReal *a, const dReal *d, int n)
{
    scaleLargeVector<1, 1>(a, d, n);
}

/*extern ODE_API_DEPRECATED ODE_API */
void dVectorScale(dReal *a, const dReal *d, int n)
{
    scaleLargeVector<1, 1>(a, d, n);
}


/*extern ODE_API */
void dEstimateCooperativelyScaleVectorResourceRequirements(dResourceRequirementsID requirements,
    unsigned maximalAllowedThreadCount, unsigned maximalElementCount)
{
    dAASSERT(requirements != NULL);

    dxResourceRequirementDescriptor *requirementsDescriptor = (dxResourceRequirementDescriptor *)requirements;
    ThreadedEquationSolverLDLT::estimateCooperativeScalingVectorResourceRequirements(requirementsDescriptor, maximalAllowedThreadCount, maximalElementCount);
}

/*extern ODE_API */
void dCooperativelyScaleVector(dResourceContainerID resources, unsigned allowedThreadCount, 
    dReal *dataVector, const dReal *scaleVector, unsigned elementCount)
{
    dAASSERT(resources != NULL);

    dxRequiredResourceContainer *resourceContainer = (dxRequiredResourceContainer *)resources;
    ThreadedEquationSolverLDLT::cooperativelyScaleVector(resourceContainer, allowedThreadCount, dataVector, scaleVector, elementCount);
}

