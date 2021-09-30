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
 * Equation System Threaded Solver
 * Copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")
 */



#ifndef _ODE_THREADED_SOLVER_LDLT_H_
#define _ODE_THREADED_SOLVER_LDLT_H_


#include "coop_matrix_types.h"
#include <ode/threading.h>


class dxThreadingBase;
class dxResourceRequirementDescriptor;
class dxRequiredResourceContainer;


class ThreadedEquationSolverLDLT
{
public:
    static void estimateCooperativeFactoringLDLTResourceRequirements(dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
        unsigned allowedThreadCount, unsigned rowCount);
    static void cooperativelyFactorLDLT(dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
        dReal *A, dReal *d, unsigned rowCount, unsigned rowSkip);
    
    static void estimateCooperativeSolvingL1StraightResourceRequirements(dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
        unsigned allowedThreadCount, unsigned rowCount);
    static void cooperativelySolveL1Straight(dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
        const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip);
    
    static void estimateCooperativeSolvingL1TransposedResourceRequirements(dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
        unsigned allowedThreadCount, unsigned rowCount);
    static void cooperativelySolveL1Transposed(dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
        const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip);

    static void estimateCooperativeScalingVectorResourceRequirements(dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
        unsigned allowedThreadCount, unsigned elementCount);
    static void cooperativelyScaleVector(dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
        dReal *vectorData, const dReal *scaleData, unsigned elementCount);

    static void estimateCooperativeSolvingLDLTResourceRequirements(dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
        unsigned allowedThreadCount, unsigned rowCount);
    static void cooperativelySolveLDLT(dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
        const dReal *L, const dReal *d, dReal *b, unsigned rowCount, unsigned rowSkip);

public:
    enum
    {
        ALLOCATION_DEFAULT_ALIGNMENT = COOP_THREAD_DATA_ALIGNMENT_SIZE,
    };

private:
    struct FactorizationSolveL1StripeCellContext;
    struct FactorizationFactorizeL1StripeThreadContext;

    enum
    {
        FLDLT_D_STRIDE          = 1,
        FLDLT_COOPERATIVE_BLOCK_COUNT_MINIMUM = 5,

        FSL1S_BLOCK_SIZE        = 2,

        FSL1S_REGULAR_B_ROWS    = FSL1S_BLOCK_SIZE,
        FSL1S_FINAL_B_ROWS      = 1,

        FFL1S_REGULAR_A_ROWS    = FSL1S_BLOCK_SIZE,
        FFL1S_FINAL_A_ROWS      = 1,
        FFL1S_REGULAR_BLOCK_SIZE = 16,  // A suitable by magnitude number being a power of 2 and (naturally) not being divisible by 6
        FFL1S_FINAL_BLOCK_SIZE  = 32, // A suitable by magnitude number being a power of 2 and (naturally) not being divisible by 6
    };

    static unsigned restrictFactoringLDLTAllowedThreadCount(
        dxThreadingBase *threading, unsigned allowedThreadCount, unsigned rowCount);
    static void doEstimateCooperativeFactoringLDLTResourceRequirementsValidated(
        dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
        unsigned allowedThreadCount, unsigned rowCount);
    static void doCooperativelyFactorLDLTValidated(
        dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
        dReal *A, dReal *d, unsigned rowCount, unsigned rowSkip);


    static unsigned deriveSolvingL1StripeBlockCount(unsigned rowCount, unsigned blockStep)
    {
        return (rowCount + (blockStep - 1)) / blockStep;
    }

    struct FactorizationSolvingL1StripeMemoryEstimates
    {
        void assignData(sizeint descriptorSizeRequired, sizeint contextSizeRequired)
        {
            m_descriptorSizeRequired = descriptorSizeRequired;
            m_contextSizeRequired = contextSizeRequired;
        }

        sizeint  m_descriptorSizeRequired;
        sizeint  m_contextSizeRequired;
    };

    static unsigned deriveSolvingL1StripeThreadCount(unsigned blockCount, unsigned allowedThreadCount)
    {
        dIASSERT(allowedThreadCount >= 1);

        unsigned maximumCount = blockCount / 2;
        return maximumCount >= allowedThreadCount ? allowedThreadCount : dMACRO_MAX(maximumCount, 1U);
    }

    static sizeint estimateCooperativelySolvingL1Stripe_XMemoryRequirement(unsigned blockCount, 
        FactorizationSolvingL1StripeMemoryEstimates &ref_memoryEstimates)
    {
        sizeint descriptorSizeRequired = dOVERALIGNED_SIZE(sizeof(cellindexint) * blockCount, COOP_THREAD_DATA_ALIGNMENT_SIZE);
        sizeint contextSizeRequired = dOVERALIGNED_SIZE(sizeof(FactorizationSolveL1StripeCellContext) * (CCI__MAX + 1) * blockCount, COOP_THREAD_DATA_ALIGNMENT_SIZE);
        ref_memoryEstimates.assignData(descriptorSizeRequired, contextSizeRequired);

        sizeint totalSizeRequired = descriptorSizeRequired + contextSizeRequired;
        return totalSizeRequired;
    }

    static void *markCooperativelySolvingL1Stripe_XMemoryStructuresOut(void *buffer, 
        const FactorizationSolvingL1StripeMemoryEstimates &memoryEstimates, 
        cellindexint *&out_blockProgressDescriptors, FactorizationSolveL1StripeCellContext *&out_cellContexts)
    {
        void *currentLocation = buffer;

        out_blockProgressDescriptors = (cellindexint *)currentLocation; currentLocation = (uint8 *)currentLocation + memoryEstimates.m_descriptorSizeRequired;
        out_cellContexts = (FactorizationSolveL1StripeCellContext *)currentLocation; currentLocation = (uint8 *)currentLocation + memoryEstimates.m_contextSizeRequired;

        return currentLocation;
    }

    static void initializeCooperativelySolvingL1Stripe_XMemoryStructures(unsigned blockCount, 
        atomicord32 &out_blockCompletionProgress, cellindexint *blockProgressDescriptors, FactorizationSolveL1StripeCellContext *dUNUSED(cellContexts))
    {
        out_blockCompletionProgress = 0;
        memset(blockProgressDescriptors, 0, blockCount * sizeof(*blockProgressDescriptors));
    }

    template<unsigned int block_step, unsigned int b_rows>
    static void participateSolvingL1Stripe_X(const dReal *L, dReal *B, unsigned blockCount, unsigned rowSkip, 
        volatile atomicord32 &refBlockCompletionProgress/*=0*/, volatile cellindexint *blockProgressDescriptors/*=[blockCount]*/, 
        FactorizationSolveL1StripeCellContext *cellContexts/*=[CCI__MAX x blockCount] + [blockCount]*/, unsigned ownThreadIndex);

    static unsigned deriveScalingAndFactorizingL1StripeBlockCountFromSolvingBlockIndex(unsigned solvingBlockIndex, unsigned solvingBlockStep, unsigned blockARows)
    {
        unsigned factorizingBlockSize = deriveScalingAndFactorizingL1StripeBlockSize(blockARows);
        return deriveScalingAndFactorizingL1StripeBlockCountFromFactorizationRow(solvingBlockIndex * solvingBlockStep, factorizingBlockSize);
    }

    static unsigned deriveScalingAndFactorizingL1StripeBlockCountFromFactorizationRow(unsigned factorizationRowIndex, unsigned factorizationBlockSize)
    {
        return (factorizationRowIndex + (factorizationBlockSize - 1)) / factorizationBlockSize;
    }

    static unsigned deriveScalingAndFactorizingL1StripeBlockSize(unsigned blockARows)
    {
        unsigned result = blockARows != 1 ? FFL1S_REGULAR_BLOCK_SIZE : FFL1S_FINAL_BLOCK_SIZE;
        dIASSERT(blockARows >= 1 && blockARows <= 2);

        return result;
    }


    static unsigned deriveScalingAndFactorizingL1StripeThreadCount(unsigned blockCount, unsigned allowedThreadCount)
    {
        dIASSERT(blockCount != 0);
        dIASSERT(allowedThreadCount >= 1);

        return dMACRO_MIN(blockCount, allowedThreadCount);
    }

    struct FactorizationFactorizeL1StripeContext;

    struct FactorizationScalingAndFactorizingL1StripeMemoryEstimates
    {
        void assignData(sizeint contextSizeRequired)
        {
            m_contextSizeRequired = contextSizeRequired;
        }

        sizeint  m_contextSizeRequired;
    };

    static sizeint estimateCooperativelyScalingAndFactorizingL1Stripe_XMemoryRequirement(unsigned factorizingMaximumThreads, 
        FactorizationScalingAndFactorizingL1StripeMemoryEstimates &ref_memoryEstimates)
    {
        dIASSERT(factorizingMaximumThreads != 0);

        sizeint contextSizeRequired = dOVERALIGNED_SIZE(sizeof(FactorizationFactorizeL1StripeContext) + sizeof(FactorizationFactorizeL1StripeThreadContext) * (factorizingMaximumThreads - 1), COOP_THREAD_DATA_ALIGNMENT_SIZE);
        ref_memoryEstimates.assignData(contextSizeRequired);

        sizeint totalSizeRequired = contextSizeRequired;
        return totalSizeRequired;
    }

    static void *markCooperativelyScalingAndFactorizingL1Stripe_XMemoryStructuresOut(void *buffer, 
        const FactorizationScalingAndFactorizingL1StripeMemoryEstimates &memoryEstimates, FactorizationFactorizeL1StripeContext *&out_factorizationContext)
    {
        void *currentLocation = buffer;

        out_factorizationContext = (FactorizationFactorizeL1StripeContext *)currentLocation; currentLocation = (uint8 *)currentLocation + memoryEstimates.m_contextSizeRequired;

        return currentLocation;
    }

    static void initializeCooperativelyScalingAndFactorizingL1Stripe_XMemoryStructures( 
        FactorizationFactorizeL1StripeContext *factorizationContext, unsigned threadCount)
    {
        factorizationContext->initialize(threadCount);
    }


    template<unsigned int a_rows, unsigned int d_stride>
    static void participateScalingAndFactorizingL1Stripe_X(dReal *ARow, dReal *d, unsigned factorizationRow, unsigned rowSkip,
        FactorizationFactorizeL1StripeContext *factorizationContext, unsigned ownThreadIndex);

private:
    struct FactorLDLTWorkerContext
    {
        FactorLDLTWorkerContext(dxThreadingBase *threading, unsigned allowedThreadCount, 
            dReal *A, dReal *d, unsigned totalBlockCount, unsigned rowCount, unsigned rowSkip, 
            atomicord32 &ref_solvingBlockCompletionProgress, cellindexint *solvingBlockProgressDescriptors, 
            FactorizationSolveL1StripeCellContext *solvingCellContexts, 
            FactorizationFactorizeL1StripeContext *factorizingFactorizationContext,
            dCallReleaseeID calculationFinishReleasee):
            m_threading(threading),
            m_allowedThreadCount(allowedThreadCount),
            m_A(A),
            m_ARow(A),
            m_d(d),
            m_solvingBlockIndex(0),
            m_totalBlockCount(totalBlockCount),
            m_rowCount(rowCount),
            m_rowSkip(rowSkip),
            m_refSolvingBlockCompletionProgress(ref_solvingBlockCompletionProgress),
            m_solvingBlockProgressDescriptors(solvingBlockProgressDescriptors),
            m_solvingCellContexts(solvingCellContexts),
            m_factorizingFactorizationContext(factorizingFactorizationContext),
            m_calculationFinishReleasee(calculationFinishReleasee)
        {
        }

        void incrementForNextBlock()
        {
            const unsigned blockStep = FSL1S_BLOCK_SIZE;

            m_ARow += blockStep * m_rowSkip;
            m_solvingBlockIndex += 1;
        }

        dxThreadingBase             *m_threading;
        unsigned                    m_allowedThreadCount;
        dReal                       *m_A;
        dReal                       *m_ARow;
        dReal                       *m_d;
        unsigned                    m_solvingBlockIndex;
        unsigned                    m_totalBlockCount;
        unsigned                    m_rowCount;
        unsigned                    m_rowSkip;
        atomicord32                 &m_refSolvingBlockCompletionProgress;
        cellindexint                *m_solvingBlockProgressDescriptors;
        FactorizationSolveL1StripeCellContext *m_solvingCellContexts; 
        FactorizationFactorizeL1StripeContext *m_factorizingFactorizationContext;
        dCallReleaseeID             m_calculationFinishReleasee;
    };

    static int factotLDLT_solvingComplete_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
    static void factotLDLT_solvingComplete(FactorLDLTWorkerContext &ref_context, unsigned ownThreadIndex);

    static int factotLDLT_solvingCompleteSync_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
    static void factotLDLT_solvingCompleteSync(FactorLDLTWorkerContext &ref_workerContext);

    static int factotLDLT_scalingAndFactorizingComplete_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
    static void factotLDLT_scalingAndFactorizingComplete(FactorLDLTWorkerContext &ref_workerContext, unsigned ownThreadIndex);

    static int factotLDLT_scalingAndFactorizingCompleteSync_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
    static void factotLDLT_scalingAndFactorizingCompleteSync(FactorLDLTWorkerContext &ref_workerContext);

    static int factotLDLT_solvingFinal_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
    static void factotLDLT_solvingFinal(FactorLDLTWorkerContext &ref_context, unsigned ownThreadIndex);

    static int factotLDLT_solvingFinalSync_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
    static void factotLDLT_solvingFinalSync(FactorLDLTWorkerContext &ref_workerContext);

    static int factotLDLT_scalingAndFactorizingFinal_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
    static void factotLDLT_scalingAndFactorizingFinal(FactorLDLTWorkerContext &ref_workerContext, unsigned ownThreadIndex);

    static int factotLDLT_completion_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);

private:
    struct FactorizationSolveL1StripeCellContext
    {
        template<unsigned int block_step, unsigned int b_rows>
        static void initializePrecalculatedZs(dReal (&Z)[block_step][b_rows])
        {
            Z[0][0] = 0;
            if (b_rows >= 2)
            {
                Z[0][1] = 0;
            }
            Z[1][0] = 0;
            if (b_rows >= 2)
            {
                Z[1][1] = 0;
            }
            dSASSERT(block_step == 2);
            dSASSERT(b_rows >= 1 && b_rows <= 2);
        }

        template<unsigned int block_step, unsigned int b_rows>
        void loadPrecalculatedZs(dReal (&Z)[block_step][b_rows]) const
        {
            dSASSERT(block_step <= dARRAY_SIZE(m_c));
            dSASSERT(b_rows <= dARRAY_SIZE(m_c[0]));

            Z[0][0] = m_c[0][0];
            if (b_rows >= 2)
            {
                Z[0][1] = m_c[0][1];
            }
            Z[1][0] = m_c[1][0];
            if (b_rows >= 2)
            {
                Z[1][1] = m_c[1][1];
            }
            dSASSERT(block_step == 2);
            dSASSERT(b_rows >= 1 && b_rows <= 2);
        }

        template<unsigned int block_step, unsigned int b_rows>
        void storePrecalculatedZs(const dReal (&Z)[block_step][b_rows])
        {
            dSASSERT(block_step <= dARRAY_SIZE(m_c));
            dSASSERT(b_rows <= dARRAY_SIZE(m_c[0]));

            m_c[0][0] = Z[0][0];
            if (b_rows >= 2)
            {
                m_c[0][1] = Z[0][1];
            }
            m_c[1][0] = Z[1][0];
            if (b_rows >= 2)
            {
                m_c[1][1] = Z[1][1];
            }
            dSASSERT(block_step == 2);
            dSASSERT(b_rows >= 1 && b_rows <= 2);
        }

        dReal m_c[FSL1S_BLOCK_SIZE][FSL1S_REGULAR_B_ROWS];
        // dReal m_reserved[4];
    };

    static FactorizationSolveL1StripeCellContext &buildBlockContextRef(FactorizationSolveL1StripeCellContext *cellContexts, unsigned blockIndex, CellContextInstance contextInstance)
    {
        return cellContexts[blockIndex * CCI__MAX + contextInstance];
    }

    static FactorizationSolveL1StripeCellContext &buildResultContextRef(FactorizationSolveL1StripeCellContext *cellContexts, unsigned blockIndex, unsigned blockCount)
    {
        return cellContexts[blockCount * CCI__MAX + blockIndex];
    }

private:
    struct FactorizationFactorizeL1StripeThreadContext
    {
        template<unsigned int a_rows>
        void assignDataSum(const dReal (&sameZ)[a_rows], const dReal (&mixedZ)[dMACRO_MAX(a_rows - 1, 1)], 
            const FactorizationFactorizeL1StripeThreadContext &partialSumContext)
        {
            m_sameZ[0] = sameZ[0] + partialSumContext.m_sameZ[0];
            if (a_rows >= 2)
            {
                m_sameZ[1] = sameZ[1] + partialSumContext.m_sameZ[1];
                m_mixedZ[0] = mixedZ[0] + partialSumContext.m_mixedZ[0];
            }
        }

        template<unsigned int a_rows>
        void assignDataAlone(const dReal (&sameZ)[a_rows], const dReal (&mixedZ)[dMACRO_MAX(a_rows - 1, 1)])
        {
            m_sameZ[0] = sameZ[0];
            if (a_rows >= 2)
            {
                m_sameZ[1] = sameZ[1];
                m_mixedZ[0] = mixedZ[0];
            }
        }

        template<unsigned int a_rows>
        void retrieveData(dReal (&out_sameZ)[a_rows], dReal (&out_mixedZ)[dMACRO_MAX(a_rows - 1, 1)]) const
        {
            out_sameZ[0] = m_sameZ[0];
            if (a_rows >= 2)
            {
                out_sameZ[1] = m_sameZ[1];
                out_mixedZ[0] = m_mixedZ[0];
            }
            dAASSERT(a_rows >= 1 && a_rows <= 2);
        }

        dReal m_sameZ[FFL1S_REGULAR_A_ROWS];
        dReal m_mixedZ[dMACRO_MAX(FFL1S_REGULAR_A_ROWS - 1, 1)];
        dReal m_reserved[1]; // [5]; // for alignment
    };

    struct FactorizationFactorizeL1StripeContext
    {
        void initialize(unsigned threadCount)
        {
            m_threadsRunning = threadCount;
            m_nextColumnIndex = 0;
            m_sumThreadIndex = 0;
        }

        atomicord32 m_threadsRunning;
        atomicord32 m_nextColumnIndex;
        volatile atomicord32 m_sumThreadIndex;
        atomicord32 m_reserved[1]; // [13]; // for alignment
        FactorizationFactorizeL1StripeThreadContext m_threadContexts[1]; // =[threadCount]
    };

private:
    struct SolveL1StraightCellContext;

    enum
    {
        SL1S_COOPERATIVE_BLOCK_COUNT_MINIMUM = 8,

        SL1S_B_STRIDE   = 1,
        SL1S_BLOCK_SIZE = 4,
    };

    static unsigned restrictSolvingL1StraightAllowedThreadCount(
        dxThreadingBase *threading, unsigned allowedThreadCount, unsigned rowCount);
    static void doEstimateCooperativeSolvingL1StraightResourceRequirementsValidated(
        dxResourceRequirementDescriptor *summaryRequirementsDescriptor, 
        unsigned allowedThreadCount, unsigned rowCount);
    static void doCooperativelySolveL1StraightValidated(
        dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
        const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip);

    static unsigned deriveSolvingL1StraightBlockCount(unsigned rowCount, unsigned blockStep)
    {
        return (rowCount + (blockStep - 1)) / blockStep;
    }

    struct SolvingL1StraightMemoryEstimates
    {
        void assignData(sizeint descriptorSizeRequired, sizeint contextSizeRequired)
        {
            m_descriptorSizeRequired = descriptorSizeRequired;
            m_contextSizeRequired = contextSizeRequired;
        }

        sizeint  m_descriptorSizeRequired;
        sizeint  m_contextSizeRequired;
    };

    static unsigned deriveSolvingL1StraightThreadCount(unsigned blockCount, unsigned allowedThreadCount)
    {
        dIASSERT(allowedThreadCount >= 1);

        unsigned maximumCount = 1 + blockCount / SL1S_COOPERATIVE_BLOCK_COUNT_MINIMUM;
        return maximumCount >= allowedThreadCount ? allowedThreadCount : dMACRO_MAX(maximumCount, 1U);
    }

    template<unsigned int block_step>
    static sizeint estimateCooperativelySolvingL1StraightMemoryRequirement(unsigned rowCount, SolvingL1StraightMemoryEstimates &ref_solvingMemoryEstimates);

    static void *markCooperativelySolvingL1StraightMemoryStructuresOut(void *buffer, 
        const SolvingL1StraightMemoryEstimates &solvingMemoryEstimates, 
        cellindexint *&out_blockProgressDescriptors, SolveL1StraightCellContext *&out_cellContexts)
    {
        void *currentLocation = buffer;

        out_blockProgressDescriptors = (cellindexint *)currentLocation; currentLocation = (uint8 *)currentLocation + solvingMemoryEstimates.m_descriptorSizeRequired;
        out_cellContexts = (SolveL1StraightCellContext *)currentLocation; currentLocation = (uint8 *)currentLocation + solvingMemoryEstimates.m_contextSizeRequired;
        return currentLocation;
    }

    template<unsigned int block_step>
    static void initializeCooperativelySolveL1StraightMemoryStructures(unsigned rowCount, 
        atomicord32 &out_blockCompletionProgress, cellindexint *blockProgressDescriptors, SolveL1StraightCellContext *cellContexts);
    template<unsigned int block_step, unsigned int b_stride>
    static void participateSolvingL1Straight(const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip, 
        volatile atomicord32 &refBlockCompletionProgress/*=0*/, volatile cellindexint *blockProgressDescriptors/*=[blockCount]*/, 
        SolveL1StraightCellContext *cellContexts/*=[CCI__MAX x blockCount] + [blockCount]*/, unsigned ownThreadIndex);

private:
    struct SolveL1StraightWorkerContext
    {
        void init(const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip, 
            atomicord32 &ref_blockCompletionProgress, cellindexint *blockProgressDescriptors, SolveL1StraightCellContext *cellContexts)
        {
            m_L = L;
            m_b = b;
            m_rowCount = rowCount;
            m_rowSkip = rowSkip; 
            m_ptrBlockCompletionProgress = &ref_blockCompletionProgress;
            m_blockProgressDescriptors = blockProgressDescriptors;
            m_cellContexts = cellContexts;
        }

        const dReal     *m_L;
        dReal           *m_b;
        unsigned        m_rowCount;
        unsigned        m_rowSkip; 
        atomicord32     *m_ptrBlockCompletionProgress;
        cellindexint    *m_blockProgressDescriptors;
        SolveL1StraightCellContext *m_cellContexts;
    };

    static int solveL1Straight_worker_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
    static void solveL1Straight_worker(SolveL1StraightWorkerContext &ref_context, unsigned ownThreadIndex);

    static int solveL1Straight_completion_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);

private:
    struct SolveL1StraightCellContext
    {
        template<unsigned int block_step>
        static void initializePrecalculatedZs(dReal (&Z)[block_step])
        {
            std::fill(Z, Z + block_step, REAL(0.0));
        }

        template<unsigned int block_step>
        void loadPrecalculatedZs(dReal (&Z)[block_step]) const
        {
            dSASSERT(block_step <= dARRAY_SIZE(m_c));

            std::copy(m_c, m_c + block_step, Z);
        }

        template<unsigned int block_step>
        void storePrecalculatedZs(const dReal (&Z)[block_step])
        {
            dSASSERT(block_step <= dARRAY_SIZE(m_c));

            std::copy(Z, Z + block_step, m_c);
        }

        dReal m_c[SL1S_BLOCK_SIZE];
    };


    static SolveL1StraightCellContext &buildBlockContextRef(SolveL1StraightCellContext *cellContexts, unsigned blockIndex, CellContextInstance contextInstance)
    {
        return cellContexts[blockIndex * CCI__MAX + contextInstance];
    }

    static SolveL1StraightCellContext &buildResultContextRef(SolveL1StraightCellContext *cellContexts, unsigned blockIndex, unsigned blockCount)
    {
        return cellContexts[blockCount * CCI__MAX + blockIndex];
    }


private:
    struct SolveL1TransposedCellContext;

    enum
    {
        SL1T_COOPERATIVE_BLOCK_COUNT_MINIMUM = SL1S_COOPERATIVE_BLOCK_COUNT_MINIMUM,

        SL1T_B_STRIDE   = SL1S_B_STRIDE,
        SL1T_BLOCK_SIZE = 4,
    };

    static unsigned restrictSolvingL1TransposedAllowedThreadCount(
        dxThreadingBase *threading, unsigned allowedThreadCount, unsigned rowCount);
    static void doEstimateCooperativeSolvingL1TransposedResourceRequirementsValidated(
        dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
        unsigned allowedThreadCount, unsigned rowCount);
    static void doCooperativelySolveL1TransposedValidated(
        dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
        const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip);

    static unsigned deriveSolvingL1TransposedBlockCount(unsigned rowCount, unsigned blockStep)
    {
        return (rowCount + (blockStep - 1)) / blockStep;
    }

    struct SolvingL1TransposedMemoryEstimates
    {
        void assignData(sizeint descriptorSizeRequired, sizeint contextSizeRequired)
        {
            m_descriptorSizeRequired = descriptorSizeRequired;
            m_contextSizeRequired = contextSizeRequired;
        }

        sizeint  m_descriptorSizeRequired;
        sizeint  m_contextSizeRequired;
    };

    static unsigned deriveSolvingL1TransposedThreadCount(unsigned blockCount, unsigned allowedThreadCount)
    {
        dSASSERT(SL1T_COOPERATIVE_BLOCK_COUNT_MINIMUM + 0 == SL1S_COOPERATIVE_BLOCK_COUNT_MINIMUM);
        
        return deriveSolvingL1StraightThreadCount(blockCount, allowedThreadCount);
    }

    template<unsigned int block_step>
    static sizeint estimateCooperativelySolvingL1TransposedMemoryRequirement(unsigned rowCount, SolvingL1TransposedMemoryEstimates &ref_solvingMemoryEstimates);

    static void *markCooperativelySolvingL1TransposedMemoryStructuresOut(void *buffer, 
        const SolvingL1TransposedMemoryEstimates &solvingMemoryEstimates, 
        cellindexint *&out_blockProgressDescriptors, SolveL1TransposedCellContext *&out_cellContexts)
    {
        void *currentLocation = buffer;

        out_blockProgressDescriptors = (cellindexint *)currentLocation; currentLocation = (uint8 *)currentLocation + solvingMemoryEstimates.m_descriptorSizeRequired;
        out_cellContexts = (SolveL1TransposedCellContext *)currentLocation; currentLocation = (uint8 *)currentLocation + solvingMemoryEstimates.m_contextSizeRequired;
        return currentLocation;
    }

    template<unsigned int block_step>
    static void *allocateCooperativelySolveL1TransposedMemoryStructures(sizeint &out_sizeAllocated, unsigned rowCount, 
        cellindexint *&out_blockProgressDescriptors, SolveL1TransposedCellContext *&out_cellContexts);
    template<unsigned int block_step>
    static void initializeCooperativelySolveL1TransposedMemoryStructures(unsigned rowCount, 
        atomicord32 &out_blockCompletionProgress, cellindexint *blockProgressDescriptors, SolveL1TransposedCellContext *cellContexts);
    template<unsigned int block_step, unsigned int b_stride>
    static void participateSolvingL1Transposed(const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip, 
        volatile atomicord32 &refBlockCompletionProgress/*=0*/, volatile cellindexint *blockProgressDescriptors/*=[blockCount]*/, 
        SolveL1TransposedCellContext *cellContexts/*=[CCI__MAX x blockCount] + [blockCount]*/, unsigned ownThreadIndex);

private:
    struct SolveL1TransposedWorkerContext
    {
        void init(const dReal *L, dReal *b, unsigned rowCount, unsigned rowSkip, 
            atomicord32 &ref_blockCompletionProgress, cellindexint *blockProgressDescriptors, SolveL1TransposedCellContext *cellContexts)
        {
            m_L = L;
            m_b = b;
            m_rowCount = rowCount;
            m_rowSkip = rowSkip; 
            m_ptrBlockCompletionProgress = &ref_blockCompletionProgress;
            m_blockProgressDescriptors = blockProgressDescriptors;
            m_cellContexts = cellContexts;
        }

        const dReal     *m_L;
        dReal           *m_b;
        unsigned        m_rowCount;
        unsigned        m_rowSkip; 
        atomicord32     *m_ptrBlockCompletionProgress;
        cellindexint    *m_blockProgressDescriptors;
        SolveL1TransposedCellContext *m_cellContexts;
    };

    static int solveL1Transposed_worker_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
    static void solveL1Transposed_worker(SolveL1TransposedWorkerContext &ref_context, unsigned ownThreadIndex);

    static int solveL1Transposed_completion_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);

private:
    struct SolveL1TransposedCellContext
    {
        template<unsigned int block_step>
        static void initializePrecalculatedZs(dReal (&Z)[block_step])
        {
            std::fill(Z, Z + block_step, REAL(0.0));
        }

        template<unsigned int block_step>
        void loadPrecalculatedZs(dReal (&Z)[block_step]) const
        {
            dSASSERT(block_step <= dARRAY_SIZE(m_c));

            std::copy(m_c, m_c + block_step, Z);
        }

        template<unsigned int block_step>
        void storePrecalculatedZs(const dReal (&Z)[block_step])
        {
            dSASSERT(block_step <= dARRAY_SIZE(m_c));

            std::copy(Z, Z + block_step, m_c);
        }

        dReal m_c[SL1T_BLOCK_SIZE];
    };

    static SolveL1TransposedCellContext &buildBlockContextRef(SolveL1TransposedCellContext *cellContexts, unsigned blockIndex, CellContextInstance contextInstance)
    {
        return cellContexts[blockIndex * CCI__MAX + contextInstance];
    }

    static SolveL1TransposedCellContext &buildResultContextRef(SolveL1TransposedCellContext *cellContexts, unsigned blockIndex, unsigned blockCount)
    {
        return cellContexts[blockCount * CCI__MAX + blockIndex];
    }

private:
    enum
    {
        SV_A_STRIDE = 1,
        SV_D_STRIDE = 1,

        SV_BLOCK_SIZE = 128,
        SV_COOPERATIVE_BLOCK_COUNT_MINIMUM = 3,
    };

    static unsigned restrictScalingVectorAllowedThreadCount(
        dxThreadingBase *threading, unsigned allowedThreadCount, unsigned elementCount);
    static void doEstimateCooperativeScalingVectorResourceRequirementsValidated(
        dxResourceRequirementDescriptor *summaryRequirementsDescriptor,
        unsigned allowedThreadCount, unsigned elementCount);
    static void doCooperativelyScaleVectorValidated(dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, 
        dReal *vectorData, const dReal *scaleData, unsigned elementCount);

    static unsigned deriveScalingVectorBlockCount(unsigned elementCount, unsigned blockStep)
    {
        return (elementCount + (blockStep - 1)) / blockStep; 
    }

    static unsigned deriveScalingVectorThreadCount(unsigned lastBlockIndex, unsigned allowedThreadCount)
    {
        dIASSERT(allowedThreadCount >= 1);

        unsigned maximumCount = lastBlockIndex;
        return maximumCount >= allowedThreadCount ? allowedThreadCount : dMACRO_MAX(maximumCount, 1U);
    }

    static void initializeCooperativelyScaleVectorMemoryStructures(atomicord32 &out_blockCompletionProgress)
    {
        out_blockCompletionProgress = 0;
    }
    template<unsigned int block_step, unsigned int a_stride, unsigned int d_stride>
    static void participateScalingVector(dReal *ptrAStart, const dReal *ptrDStart, const unsigned elementCount,
        volatile atomicord32 &refBlockCompletionProgress/*=0*/);

private:
    struct ScaleVectorWorkerContext
    {
        void init(dReal *vectorData, const dReal *scaleData, unsigned elementCount, 
            atomicord32 &ref_blockCompletionProgress)
        {
            m_vectorData = vectorData;
            m_scaleData = scaleData;
            m_elementCount = elementCount;
            m_ptrBlockCompletionProgress = &ref_blockCompletionProgress;
        }

        dReal           *m_vectorData;
        const dReal     *m_scaleData;
        unsigned        m_elementCount;
        atomicord32     *m_ptrBlockCompletionProgress;
    };

    static int scaleVector_worker_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
    static void scaleVector_worker(ScaleVectorWorkerContext &ref_context);

    static int scaleVector_completion_callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);


private:
    enum SolvingLDLTStage
    {
        SLDLTS__MIN,

        SLDLTS_SOLVING_STRAIGHT = SLDLTS__MIN,
        SLDLTS_SCALING_VECTOR,
        SLDLTS_SOLVING_TRANSPOSED,

        SLDLTS__MAX,
    };

    enum
    {
        SLDLT_B_STRIDE          = SL1S_B_STRIDE,
        SLDLT_D_STRIDE          = FLDLT_D_STRIDE,
    };

    static unsigned restrictSolvingLDLTAllowedThreadCount(
        dxThreadingBase *threading, unsigned allowedThreadCount, unsigned rowCount, unsigned &out_stageBlockCountSifficiencyMask);
    
    static void doCooperativelySolveLDLTValidated(
        dxRequiredResourceContainer *resourceContainer, unsigned allowedThreadCount, unsigned stageBlockCountSifficiencyMask, 
        const dReal *L, const dReal *d, dReal *b, unsigned rowCount, unsigned rowSkip);
};


#endif

