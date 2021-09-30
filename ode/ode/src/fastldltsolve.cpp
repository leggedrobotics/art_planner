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
 * LDLT solving related code of ThreadedEquationSolverLDLT 
 * Copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")
 */

#include <ode/common.h>
#include <ode/matrix.h>
#include <ode/matrix_coop.h>
#include "config.h"
#include "threaded_solver_ldlt.h"
#include "threading_base.h"
#include "resource_control.h"

#include "fastldltsolve_impl.h"


/*static */
void ThreadedEquationSolverLDLT::estimateCooperativeSolvingLDLTResourceRequirements(
    dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
    unsigned allowedThreadCount, unsigned rowCount)
{
    unsigned stageBlockCountSifficiencyMask;
    dxThreadingBase *threading = summaryRequirementsDescriptor->getrelatedThreading();
    unsigned limitedThreadCount = restrictSolvingLDLTAllowedThreadCount(threading, allowedThreadCount, rowCount, stageBlockCountSifficiencyMask);

    if (limitedThreadCount > 1)
    {
        if ((stageBlockCountSifficiencyMask & (1U << SLDLTS_SOLVING_STRAIGHT)) != 0)
        {
            doEstimateCooperativeSolvingL1StraightResourceRequirementsValidated(summaryRequirementsDescriptor, allowedThreadCount, rowCount);
        }

        if ((stageBlockCountSifficiencyMask & (1U << SLDLTS_SCALING_VECTOR)) != 0)
        {
            doEstimateCooperativeScalingVectorResourceRequirementsValidated(summaryRequirementsDescriptor, allowedThreadCount, rowCount);
        }

        if ((stageBlockCountSifficiencyMask & (1U << SLDLTS_SOLVING_TRANSPOSED)) == 0)
        {
            doEstimateCooperativeSolvingL1TransposedResourceRequirementsValidated(summaryRequirementsDescriptor, allowedThreadCount, rowCount);
        }
    }
}

/*static */
void ThreadedEquationSolverLDLT::cooperativelySolveLDLT(
    dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
    const dReal *L, const dReal *d, dReal *b, unsigned rowCount, unsigned rowSkip)
{
    dAASSERT(rowCount != 0);

    unsigned stageBlockCountSifficiencyMask;

    dxThreadingBase *threading = resourceContainer->getThreadingInstance();
    unsigned limitedThreadCount = restrictSolvingLDLTAllowedThreadCount(threading, allowedThreadCount, rowCount, stageBlockCountSifficiencyMask);

    if (limitedThreadCount <= 1)
    {
        solveEquationSystemWithLDLT<SLDLT_D_STRIDE, SLDLT_B_STRIDE>(L, d, b, rowCount, rowSkip);
    }
    else
    {
        doCooperativelySolveLDLTValidated(resourceContainer, limitedThreadCount, stageBlockCountSifficiencyMask, L, d, b, rowCount, rowSkip);
    }
}

/*static */
unsigned ThreadedEquationSolverLDLT::restrictSolvingLDLTAllowedThreadCount(
    dxThreadingBase *threading, unsigned allowedThreadCount, unsigned rowCount, unsigned &out_stageBlockCountSifficiencyMask)
{
    unsigned limitedThreadCount = 1;
    unsigned stageBlockCountSifficiencyMask = 0;

#if dCOOPERATIVE_ENABLED
    {
        const unsigned int blockStep = SL1S_BLOCK_SIZE; // Required by the implementation
        unsigned solvingStraightBlockCount = deriveSolvingL1StraightBlockCount(rowCount, blockStep);
        dIASSERT(deriveSolvingL1StraightThreadCount(SL1S_COOPERATIVE_BLOCK_COUNT_MINIMUM, 2) > 1);

        if (solvingStraightBlockCount >= SL1S_COOPERATIVE_BLOCK_COUNT_MINIMUM)
        {
            stageBlockCountSifficiencyMask |= 1U << SLDLTS_SOLVING_STRAIGHT;
        }
    }

    {
        const unsigned int blockStep = SV_BLOCK_SIZE; // Required by the implementation
        unsigned scalingBlockCount = deriveScalingVectorBlockCount(rowCount, blockStep);
        dIASSERT(deriveScalingVectorThreadCount(SV_COOPERATIVE_BLOCK_COUNT_MINIMUM - 1, 2) > 1);

        if (scalingBlockCount >= SV_COOPERATIVE_BLOCK_COUNT_MINIMUM)
        {
            stageBlockCountSifficiencyMask |= 1U << SLDLTS_SCALING_VECTOR;
        }
    }

    {
        const unsigned int blockStep = SL1T_BLOCK_SIZE; // Required by the implementation
        unsigned solvingTransposedBlockCount = deriveSolvingL1TransposedBlockCount(rowCount, blockStep);
        dIASSERT(deriveSolvingL1TransposedThreadCount(SL1T_COOPERATIVE_BLOCK_COUNT_MINIMUM, 2) > 1);

        if (solvingTransposedBlockCount >= SL1T_COOPERATIVE_BLOCK_COUNT_MINIMUM)
        {
            stageBlockCountSifficiencyMask |= 1U << SLDLTS_SOLVING_TRANSPOSED;
        }
    }

    if (stageBlockCountSifficiencyMask != 0)
    {
        limitedThreadCount = threading->calculateThreadingLimitedThreadCount(allowedThreadCount, true);
    }
#endif // #if dCOOPERATIVE_ENABLED

    out_stageBlockCountSifficiencyMask = stageBlockCountSifficiencyMask;
    return limitedThreadCount;
}


/*static */
void ThreadedEquationSolverLDLT::doCooperativelySolveLDLTValidated(
    dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, unsigned stageBlockCountSifficiencyMask, 
    const dReal *L, const dReal *d, dReal *b, unsigned rowCount, unsigned rowSkip)
{
    dIASSERT(allowedThreadCount > 1);

    if ((stageBlockCountSifficiencyMask & (1U << SLDLTS_SOLVING_STRAIGHT)) == 0)
    {
        solveL1Straight<SLDLT_B_STRIDE>(L, b, rowCount, rowSkip);
    }
    else
    {
        dSASSERT(SLDLT_B_STRIDE + 0 == SL1S_B_STRIDE);

        doCooperativelySolveL1StraightValidated(resourceContainer, allowedThreadCount, L, b, rowCount, rowSkip);
    }

    if ((stageBlockCountSifficiencyMask & (1U << SLDLTS_SCALING_VECTOR)) == 0)
    {
        scaleLargeVector<SLDLT_B_STRIDE, SLDLT_D_STRIDE>(b, d, rowCount);
    }
    else
    {
        dSASSERT(SLDLT_B_STRIDE + 0 == SV_A_STRIDE);
        dSASSERT(SLDLT_D_STRIDE + 0 == SV_D_STRIDE);

        doCooperativelyScaleVectorValidated(resourceContainer, allowedThreadCount, b, d, rowCount);
    }

    if ((stageBlockCountSifficiencyMask & (1U << SLDLTS_SOLVING_TRANSPOSED)) == 0)
    {
        solveL1Transposed<SLDLT_B_STRIDE>(L, b, rowCount, rowSkip);
    }
    else
    {
        dSASSERT(SLDLT_B_STRIDE + 0 == SL1T_B_STRIDE);

        doCooperativelySolveL1TransposedValidated(resourceContainer, allowedThreadCount, L, b, rowCount, rowSkip);
    }
}


//////////////////////////////////////////////////////////////////////////
// Public interface functions

/*extern ODE_API */
void dSolveLDLT(const dReal *L, const dReal *d, dReal *b, int n, int nskip)
{
    dAASSERT(n != 0);

    if (n != 0)
    {
        dAASSERT(L != NULL);
        dAASSERT(d != NULL);
        dAASSERT(b != NULL);

        solveEquationSystemWithLDLT<1, 1>(L, d, b, n, nskip);
    }
}


/*extern ODE_API */
void dEstimateCooperativelySolveLDLTResourceRequirements(dResourceRequirementsID requirements,
    unsigned maximalAllowedThreadCount, unsigned maximalRowCount)
{
    dAASSERT(requirements != NULL);

    dxResourceRequirementDescriptor *requirementsDescriptor = (dxResourceRequirementDescriptor *)requirements;
    ThreadedEquationSolverLDLT::estimateCooperativeSolvingLDLTResourceRequirements(requirementsDescriptor, maximalAllowedThreadCount, maximalRowCount);
}

/*extern ODE_API */
void dCooperativelySolveLDLT(dResourceContainerID resources, unsigned allowedThreadCount, 
    const dReal *L, const dReal *d, dReal *b, unsigned rowCount, unsigned rowSkip)
{
    dAASSERT(resources != NULL);

    dxRequiredResourceContainer *resourceContainer = (dxRequiredResourceContainer *)resources;
    ThreadedEquationSolverLDLT::cooperativelySolveLDLT(resourceContainer, allowedThreadCount, L, d, b, rowCount, rowSkip);
}

