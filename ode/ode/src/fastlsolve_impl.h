

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
 * Code style improvements and optimizations by Oleh Derevenko ????-2019
 * L1Straight cooperative solving code of ThreadedEquationSolverLDLT copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")  
 */

#ifndef _ODE_FASTLSOLVE_IMPL_H_
#define _ODE_FASTLSOLVE_IMPL_H_


/* solve L*X=B, with B containing 1 right hand sides.
 * L is an n*n lower triangular matrix with ones on the diagonal.
 * L is stored by rows and its leading dimension is lskip.
 * B is an n*1 matrix that contains the right hand sides.
 * B is stored by columns and its leading dimension is also lskip.
 * B is overwritten with X.
 * this processes blocks of 4*4.
 * if this is in the factorizer source file, n must be a multiple of 4.
 */

template<unsigned int b_stride>
void solveL1Straight (const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip)
{
    dIASSERT(rowCount != 0);

    /* compute all 4 x 1 blocks of X */
    unsigned blockStartRow = 0;
    bool subsequentPass = false;
    bool goForLoopX4 = rowCount >= 4;
    const unsigned loopX4LastRow = goForLoopX4 ? rowCount - 4 : 0;
    for (; goForLoopX4; subsequentPass = true, goForLoopX4 = (blockStartRow += 4) <= loopX4LastRow) 
    {
        /* declare variables - Z matrix, p and q vectors, etc */
        const dReal *ptrLElement;
        dReal *ptrBElement;

        dReal Z11, Z21, Z31, Z41;

        /* compute all 4 x 1 block of X, from rows i..i+4-1 */
        if (subsequentPass)
        {
            ptrLElement = L + (1 + blockStartRow) * rowSkip;
            ptrBElement = B;
            /* set the Z matrix to 0 */
            Z11 = 0; Z21 = 0; Z31 = 0; Z41 = 0;

            /* the inner loop that computes outer products and adds them to Z */
            for (unsigned columnCounter = blockStartRow; ; )
            {
                dReal q1, p1, p2, p3, p4;

                /* load p and q values */
                q1 = ptrBElement[0 * b_stride];
                p1 = (ptrLElement - rowSkip)[0];
                p2 = ptrLElement[0];
                ptrLElement += rowSkip;
                p3 = ptrLElement[0];
                p4 = ptrLElement[0 + rowSkip];

                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;

                /* load p and q values */
                q1 = ptrBElement[1 * b_stride];
                p3 = ptrLElement[1];
                p4 = ptrLElement[1 + rowSkip];
                ptrLElement -= rowSkip;
                p1 = (ptrLElement - rowSkip)[1];
                p2 = ptrLElement[1];

                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;

                /* load p and q values */
                q1 = ptrBElement[2 * b_stride];
                p1 = (ptrLElement - rowSkip)[2];
                p2 = ptrLElement[2];
                ptrLElement += rowSkip;
                p3 = ptrLElement[2];
                p4 = ptrLElement[2 + rowSkip];

                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;

                /* load p and q values */
                q1 = ptrBElement[3 * b_stride];
                p3 = ptrLElement[3];
                p4 = ptrLElement[3 + rowSkip];
                ptrLElement -= rowSkip;
                p1 = (ptrLElement - rowSkip)[3];
                p2 = ptrLElement[3];

                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z21 += p2 * q1;
                Z31 += p3 * q1;
                Z41 += p4 * q1;

                if (columnCounter > 12)
                {
                    columnCounter -= 12;

                    /* advance pointers */
                    ptrLElement += 12;
                    ptrBElement += 12 * b_stride;

                    /* load p and q values */
                    q1 = ptrBElement[-8 * (int)b_stride];
                    p1 = (ptrLElement - rowSkip)[-8];
                    p2 = ptrLElement[-8];
                    ptrLElement += rowSkip;
                    p3 = ptrLElement[-8];
                    p4 = ptrLElement[-8 + rowSkip];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z21 += p2 * q1;
                    Z31 += p3 * q1;
                    Z41 += p4 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-7 * (int)b_stride];
                    p3 = ptrLElement[-7];
                    p4 = ptrLElement[-7 + rowSkip];
                    ptrLElement -= rowSkip;
                    p1 = (ptrLElement - rowSkip)[-7];
                    p2 = ptrLElement[-7];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z21 += p2 * q1;
                    Z31 += p3 * q1;
                    Z41 += p4 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-6 * (int)b_stride];
                    p1 = (ptrLElement - rowSkip)[-6];
                    p2 = ptrLElement[-6];
                    ptrLElement += rowSkip;
                    p3 = ptrLElement[-6];
                    p4 = ptrLElement[-6 + rowSkip];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z21 += p2 * q1;
                    Z31 += p3 * q1;
                    Z41 += p4 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-5 * (int)b_stride];
                    p3 = ptrLElement[-5];
                    p4 = ptrLElement[-5 + rowSkip];
                    ptrLElement -= rowSkip;
                    p1 = (ptrLElement - rowSkip)[-5];
                    p2 = ptrLElement[-5];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z21 += p2 * q1;
                    Z31 += p3 * q1;
                    Z41 += p4 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-4 * (int)b_stride];
                    p1 = (ptrLElement - rowSkip)[-4];
                    p2 = ptrLElement[-4];
                    ptrLElement += rowSkip;
                    p3 = ptrLElement[-4];
                    p4 = ptrLElement[-4 + rowSkip];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z21 += p2 * q1;
                    Z31 += p3 * q1;
                    Z41 += p4 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-3 * (int)b_stride];
                    p3 = ptrLElement[-3];
                    p4 = ptrLElement[-3 + rowSkip];
                    ptrLElement -= rowSkip;
                    p1 = (ptrLElement - rowSkip)[-3];
                    p2 = ptrLElement[-3];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z21 += p2 * q1;
                    Z31 += p3 * q1;
                    Z41 += p4 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-2 * (int)b_stride];
                    p1 = (ptrLElement - rowSkip)[-2];
                    p2 = ptrLElement[-2];
                    ptrLElement += rowSkip;
                    p3 = ptrLElement[-2];
                    p4 = ptrLElement[-2 + rowSkip];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z21 += p2 * q1;
                    Z31 += p3 * q1;
                    Z41 += p4 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-1 * (int)b_stride];
                    p3 = ptrLElement[-1];
                    p4 = ptrLElement[-1 + rowSkip];
                    ptrLElement -= rowSkip;
                    p1 = (ptrLElement - rowSkip)[-1];
                    p2 = ptrLElement[-1];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z21 += p2 * q1;
                    Z31 += p3 * q1;
                    Z41 += p4 * q1;
                }
                else
                {
                    /* advance pointers */
                    ptrLElement += 4;
                    ptrBElement += 4 * b_stride;

                    if ((columnCounter -= 4) == 0)
                    {
                        break;
                    }
                }
                /* end of inner loop */
            }
        }
        else
        {
            ptrLElement = L + rowSkip/* + blockStartRow * rowSkip*/; dIASSERT(blockStartRow == 0);
            ptrBElement = B;
            /* set the Z matrix to 0 */
            Z11 = 0; Z21 = 0; Z31 = 0; Z41 = 0;
        }

        /* finish computing the X(i) block */
        dReal Y11, Y21, Y31, Y41;
        {
            Y11 = ptrBElement[0 * b_stride] - Z11;
            ptrBElement[0 * b_stride] = Y11;
        }
        {
            dReal p2 = ptrLElement[0];
            Y21 = ptrBElement[1 * b_stride] - Z21 - p2 * Y11;
            ptrBElement[1 * b_stride] = Y21;
        }
        ptrLElement += rowSkip;
        {
            dReal p3 = ptrLElement[0];
            dReal p3_1 = ptrLElement[1];
            Y31 = ptrBElement[2 * b_stride] - Z31 - p3 * Y11 - p3_1 * Y21;
            ptrBElement[2 * b_stride] = Y31;
        }
        {
            dReal p4 = ptrLElement[rowSkip];
            dReal p4_1 = ptrLElement[1 + rowSkip];
            dReal p4_2 = ptrLElement[2 + rowSkip];
            Y41 = ptrBElement[3 * b_stride] - Z41 - p4 * Y11 - p4_1 * Y21 - p4_2 * Y31;
            ptrBElement[3 * b_stride] = Y41;
        }
        /* end of outer loop */
    }

    /* compute rows at end that are not a multiple of block size */
    for (; !subsequentPass || blockStartRow < rowCount; subsequentPass = true, ++blockStartRow) 
    {
        /* compute all 1 x 1 block of X, from rows i..i+1-1 */
        dReal *ptrBElement;

        dReal Z11, Z12;

        if (subsequentPass)
        {
            ptrBElement = B;
            /* set the Z matrix to 0 */
            Z11 = 0; Z12 = 0;

            const dReal *ptrLElement = L + blockStartRow * rowSkip;

            /* the inner loop that computes outer products and adds them to Z */
            unsigned columnCounter = blockStartRow;
            for (bool exitLoop = columnCounter < 4; !exitLoop; exitLoop = false) 
            {
                dReal p1, p2, q1, q2;

                /* load p and q values */
                p1 = ptrLElement[0];
                p2 = ptrLElement[1];
                q1 = ptrBElement[0 * b_stride];
                q2 = ptrBElement[1 * b_stride];

                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z12 += p2 * q2;

                /* load p and q values */
                p1 = ptrLElement[2];
                p2 = ptrLElement[3];
                q1 = ptrBElement[2 * b_stride];
                q2 = ptrBElement[3 * b_stride];

                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z12 += p2 * q2;

                if (columnCounter >= (12 + 4))
                {
                    columnCounter -= 12;

                    /* advance pointers */
                    ptrLElement += 12;
                    ptrBElement += 12 * b_stride;

                    /* load p and q values */
                    p1 = ptrLElement[-8];
                    p2 = ptrLElement[-7];
                    q1 = ptrBElement[-8 * (int)b_stride];
                    q2 = ptrBElement[-7 * (int)b_stride];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z12 += p2 * q2;

                    /* load p and q values */
                    p1 = ptrLElement[-6];
                    p2 = ptrLElement[-5];
                    q1 = ptrBElement[-6 * (int)b_stride];
                    q2 = ptrBElement[-5 * (int)b_stride];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z12 += p2 * q2;

                    /* load p and q values */
                    p1 = ptrLElement[-4];
                    p2 = ptrLElement[-3];
                    q1 = ptrBElement[-4 * (int)b_stride];
                    q2 = ptrBElement[-3 * (int)b_stride];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z12 += p2 * q2;

                    /* load p and q values */
                    p1 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    q1 = ptrBElement[-2 * (int)b_stride];
                    q2 = ptrBElement[-1 * (int)b_stride];

                    /* compute outer product and add it to the Z matrix */
                    Z11 += p1 * q1;
                    Z12 += p2 * q2;
                }
                else
                {
                    /* advance pointers */
                    ptrLElement += 4;
                    ptrBElement += 4 * b_stride;

                    if ((columnCounter -= 4) < 4)
                    {
                        break;
                    }
                }
                /* end of inner loop */
            }

            /* compute left-over iterations */
            if ((columnCounter & 2) != 0) 
            {
                dReal p1, p2, q1, q2;

                /* load p and q values */
                p1 = ptrLElement[0];
                p2 = ptrLElement[1];
                q1 = ptrBElement[0 * b_stride];
                q2 = ptrBElement[1 * b_stride];

                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;
                Z12 += p2 * q2;

                /* advance pointers */
                ptrLElement += 2;
                ptrBElement += 2 * b_stride;
            }

            if ((columnCounter & 1) != 0)
            {
                dReal p1, q1;

                /* load p and q values */
                p1 = ptrLElement[0];
                q1 = ptrBElement[0 * b_stride];

                /* compute outer product and add it to the Z matrix */
                Z11 += p1 * q1;

                /* advance pointers */
                // ptrLElement += 1; -- not needed any more
                ptrBElement += 1 * b_stride;
            }

            /* finish computing the X(i) block */
            dReal Y11 = ptrBElement[0 * b_stride] - (Z11 + Z12);
            ptrBElement[0 * b_stride] = Y11;
        }
    }
}


template<unsigned int block_step>
/*static */
sizeint ThreadedEquationSolverLDLT::estimateCooperativelySolvingL1StraightMemoryRequirement(unsigned rowCount, SolvingL1StraightMemoryEstimates &ref_solvingMemoryEstimates)
{
    unsigned blockCount = deriveSolvingL1StraightBlockCount(rowCount, block_step);
    sizeint descriptorSizeRequired = dEFFICIENT_SIZE(sizeof(cellindexint) * blockCount);
    sizeint contextSizeRequired = dEFFICIENT_SIZE(sizeof(SolveL1StraightCellContext) * (CCI__MAX + 1) * blockCount);
    ref_solvingMemoryEstimates.assignData(descriptorSizeRequired, contextSizeRequired);

    sizeint totalSizeRequired = descriptorSizeRequired + contextSizeRequired;
    return totalSizeRequired;
}

template<unsigned int block_step>
/*static */
void ThreadedEquationSolverLDLT::initializeCooperativelySolveL1StraightMemoryStructures(unsigned rowCount, 
    atomicord32 &out_blockCompletionProgress, cellindexint *blockProgressDescriptors, SolveL1StraightCellContext *dUNUSED(cellContexts))
{
    unsigned blockCount = deriveSolvingL1StraightBlockCount(rowCount, block_step);

    out_blockCompletionProgress = 0;
    memset(blockProgressDescriptors, 0, blockCount * sizeof(*blockProgressDescriptors));
}

template<unsigned int block_step, unsigned int b_stride>
void ThreadedEquationSolverLDLT::participateSolvingL1Straight(const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip, 
    volatile atomicord32 &refBlockCompletionProgress/*=0*/, volatile cellindexint *blockProgressDescriptors/*=[blockCount]*/, 
    SolveL1StraightCellContext *cellContexts/*=[CCI__MAX x blockCount] + [blockCount]*/, unsigned ownThreadIndex)
{
    const unsigned lookaheadRange = 32;
    const unsigned blockCount = deriveSolvingL1StraightBlockCount(rowCount, block_step), lastBlock = blockCount - 1;
    /* compute rows at end that are not a multiple of block size */
    const unsigned loopX1RowCount = rowCount % block_step;

    BlockProcessingState blockProcessingState = BPS_NO_BLOCKS_PROCESSED;

    unsigned completedBlocks = refBlockCompletionProgress;
    unsigned currentBlock = completedBlocks;
    dIASSERT(completedBlocks <= blockCount);

    for (bool exitLoop = completedBlocks == blockCount; !exitLoop; exitLoop = false)
    {
        bool goForLockedBlockPrimaryCalculation = false, goForLockedBlockDuplicateCalculation = false;
        bool goAssigningTheResult = false, stayWithinTheBlock = false;

        dReal Z[block_step];
        dReal Y[block_step];

        dReal *ptrBElement;

        CellContextInstance previousContextInstance;
        unsigned completedColumnBlock;
        bool partialBlock;

        for (cellindexint testDescriptor = blockProgressDescriptors[currentBlock]; ; )
        {
            if (testDescriptor == INVALID_CELLDESCRIPTOR)
            {
                // Invalid descriptor is the indication that the row has been fully calculated
                // Test if this was the last row and break out if so.
                if (currentBlock + 1 == blockCount)
                {
                    exitLoop = true;
                    break;
                }

                // Treat detected row advancement as a row processed
                // blockProcessingState = BPS_SOME_BLOCKS_PROCESSED; <-- performs better without it
                break;
            }

            CooperativeAtomics::AtomicReadReorderBarrier();
            // It is necessary to read up to date completedBblocks value after the descriptor retrieval
            // as otherwise the logic below breaks
            completedBlocks = refBlockCompletionProgress;

            if (!GET_CELLDESCRIPTOR_ISLOCKED(testDescriptor))
            {
                completedColumnBlock = GET_CELLDESCRIPTOR_COLUMNINDEX(testDescriptor);
                dIASSERT(completedColumnBlock < currentBlock || (completedColumnBlock == currentBlock && currentBlock == 0)); // Otherwise, why would the calculation have had stopped if the final column is reachable???
                dIASSERT(completedColumnBlock <= completedBlocks); // Since the descriptor is not locked

                if (completedColumnBlock == completedBlocks && currentBlock != completedBlocks)
                {
                    dIASSERT(completedBlocks < currentBlock);
                    break;
                }

                if (CooperativeAtomics::AtomicCompareExchangeCellindexint(&blockProgressDescriptors[currentBlock], testDescriptor, MARK_CELLDESCRIPTOR_LOCKED(testDescriptor)))
                {
                    if (completedColumnBlock != 0)
                    {
                        CellContextInstance contextInstance = GET_CELLDESCRIPTOR_CONTEXTINSTANCE(testDescriptor);
                        previousContextInstance = contextInstance;

                        const SolveL1StraightCellContext &sourceContext = buildBlockContextRef(cellContexts, currentBlock, contextInstance);
                        sourceContext.loadPrecalculatedZs(Z);
                    }
                    else
                    {
                        previousContextInstance = CCI__MIN;
                        SolveL1StraightCellContext::initializePrecalculatedZs(Z);
                    }

                    goForLockedBlockPrimaryCalculation = true;
                    break;
                }

                if (blockProcessingState != BPS_COMPETING_FOR_A_BLOCK)
                {
                    break;
                }

                testDescriptor = blockProgressDescriptors[currentBlock];
            }
            else
            {
                if (blockProcessingState != BPS_COMPETING_FOR_A_BLOCK)
                {
                    break;
                }

                cellindexint verificativeDescriptor;
                bool verificationFailure = false;

                completedColumnBlock = GET_CELLDESCRIPTOR_COLUMNINDEX(testDescriptor);
                dIASSERT(completedColumnBlock != currentBlock || currentBlock == 0); // There is no reason for computations to stop at the very end other than being the initial value at the very first block

                if (completedColumnBlock != 0)
                {
                    CellContextInstance contextInstance = GET_CELLDESCRIPTOR_CONTEXTINSTANCE(testDescriptor);
                    const SolveL1StraightCellContext &sourceContext = buildBlockContextRef(cellContexts, currentBlock, contextInstance);
                    sourceContext.loadPrecalculatedZs(Z);
                }
                else
                {
                    SolveL1StraightCellContext::initializePrecalculatedZs(Z);
                }

                if (completedColumnBlock != 0 && completedColumnBlock <= currentBlock)
                {
                    // Make sure the descriptor is re-read after the precalculates
                    CooperativeAtomics::AtomicReadReorderBarrier();
                }

                if (completedColumnBlock <= currentBlock)
                {
                    verificativeDescriptor = blockProgressDescriptors[currentBlock];
                    verificationFailure = verificativeDescriptor != testDescriptor;
                }

                if (!verificationFailure)
                {
                    dIASSERT(completedColumnBlock <= currentBlock + 1);

                    goForLockedBlockDuplicateCalculation = true;
                    break;
                }

                testDescriptor = verificativeDescriptor;
            }
        }

        if (exitLoop)
        {
            break;
        }

        if (goForLockedBlockPrimaryCalculation)
        {
            blockProcessingState = BPS_SOME_BLOCKS_PROCESSED;

            // Declare and assign the variables at the top to not interfere with any branching -- the compiler is going to eliminate them anyway.
            bool handleComputationTakenOver = false, rowEndReached = false;

            const dReal *ptrLElement;
            unsigned finalColumnBlock;

            /* check if this is not the partial block of fewer rows */
            if (currentBlock != lastBlock || loopX1RowCount == 0)
            {
                partialBlock = false;

                if (currentBlock != 0)
                {
                    ptrLElement = L + (sizeint)(1 + currentBlock * block_step) * rowSkip + completedColumnBlock * block_step;
                    ptrBElement = B + (sizeint)(completedColumnBlock * block_step) * b_stride;

                    /* the inner loop that computes outer products and adds them to Z */
                    finalColumnBlock = dMACRO_MIN(currentBlock, completedBlocks);
                    dIASSERT(completedColumnBlock != finalColumnBlock/* || currentBlock == 0*/);

                    for (unsigned columnCounter = finalColumnBlock - completedColumnBlock; ; )
                    {
                        dReal q1, p1, p2, p3, p4;

                        /* load p and q values */
                        q1 = ptrBElement[0 * b_stride];
                        p1 = (ptrLElement - rowSkip)[0];
                        p2 = ptrLElement[0];
                        ptrLElement += rowSkip;
                        p3 = ptrLElement[0];
                        p4 = ptrLElement[0 + rowSkip];

                        /* compute outer product and add it to the Z matrix */
                        Z[0] += p1 * q1;
                        Z[1] += p2 * q1;
                        Z[2] += p3 * q1;
                        Z[3] += p4 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[1 * b_stride];
                        p3 = ptrLElement[1];
                        p4 = ptrLElement[1 + rowSkip];
                        ptrLElement -= rowSkip;
                        p1 = (ptrLElement - rowSkip)[1];
                        p2 = ptrLElement[1];

                        /* compute outer product and add it to the Z matrix */
                        Z[0] += p1 * q1;
                        Z[1] += p2 * q1;
                        Z[2] += p3 * q1;
                        Z[3] += p4 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[2 * b_stride];
                        p1 = (ptrLElement - rowSkip)[2];
                        p2 = ptrLElement[2];
                        ptrLElement += rowSkip;
                        p3 = ptrLElement[2];
                        p4 = ptrLElement[2 + rowSkip];

                        /* compute outer product and add it to the Z matrix */
                        Z[0] += p1 * q1;
                        Z[1] += p2 * q1;
                        Z[2] += p3 * q1;
                        Z[3] += p4 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[3 * b_stride];
                        p3 = ptrLElement[3];
                        p4 = ptrLElement[3 + rowSkip];
                        ptrLElement -= rowSkip;
                        p1 = (ptrLElement - rowSkip)[3];
                        p2 = ptrLElement[3];

                        /* compute outer product and add it to the Z matrix */
                        Z[0] += p1 * q1;
                        Z[1] += p2 * q1;
                        Z[2] += p3 * q1;
                        Z[3] += p4 * q1;
                        dSASSERT(block_step == 4);

                        if (columnCounter > 3)
                        {
                            columnCounter -= 3;

                            ptrLElement += 3 * block_step;
                            ptrBElement += 3 * block_step * b_stride;

                            /* load p and q values */
                            q1 = ptrBElement[-8 * (int)b_stride];
                            p1 = (ptrLElement - rowSkip)[-8];
                            p2 = ptrLElement[-8];
                            ptrLElement += rowSkip;
                            p3 = ptrLElement[-8];
                            p4 = ptrLElement[-8 + rowSkip];

                            /* compute outer product and add it to the Z matrix */
                            Z[0] += p1 * q1;
                            Z[1] += p2 * q1;
                            Z[2] += p3 * q1;
                            Z[3] += p4 * q1;

                            /* load p and q values */
                            q1 = ptrBElement[-7 * (int)b_stride];
                            p3 = ptrLElement[-7];
                            p4 = ptrLElement[-7 + rowSkip];
                            ptrLElement -= rowSkip;
                            p1 = (ptrLElement - rowSkip)[-7];
                            p2 = ptrLElement[-7];

                            /* compute outer product and add it to the Z matrix */
                            Z[0] += p1 * q1;
                            Z[1] += p2 * q1;
                            Z[2] += p3 * q1;
                            Z[3] += p4 * q1;

                            /* load p and q values */
                            q1 = ptrBElement[-6 * (int)b_stride];
                            p1 = (ptrLElement - rowSkip)[-6];
                            p2 = ptrLElement[-6];
                            ptrLElement += rowSkip;
                            p3 = ptrLElement[-6];
                            p4 = ptrLElement[-6 + rowSkip];

                            /* compute outer product and add it to the Z matrix */
                            Z[0] += p1 * q1;
                            Z[1] += p2 * q1;
                            Z[2] += p3 * q1;
                            Z[3] += p4 * q1;

                            /* load p and q values */
                            q1 = ptrBElement[-5 * (int)b_stride];
                            p3 = ptrLElement[-5];
                            p4 = ptrLElement[-5 + rowSkip];
                            ptrLElement -= rowSkip;
                            p1 = (ptrLElement - rowSkip)[-5];
                            p2 = ptrLElement[-5];

                            /* compute outer product and add it to the Z matrix */
                            Z[0] += p1 * q1;
                            Z[1] += p2 * q1;
                            Z[2] += p3 * q1;
                            Z[3] += p4 * q1;

                            /* load p and q values */
                            q1 = ptrBElement[-4 * (int)b_stride];
                            p1 = (ptrLElement - rowSkip)[-4];
                            p2 = ptrLElement[-4];
                            ptrLElement += rowSkip;
                            p3 = ptrLElement[-4];
                            p4 = ptrLElement[-4 + rowSkip];

                            /* compute outer product and add it to the Z matrix */
                            Z[0] += p1 * q1;
                            Z[1] += p2 * q1;
                            Z[2] += p3 * q1;
                            Z[3] += p4 * q1;

                            /* load p and q values */
                            q1 = ptrBElement[-3 * (int)b_stride];
                            p3 = ptrLElement[-3];
                            p4 = ptrLElement[-3 + rowSkip];
                            ptrLElement -= rowSkip;
                            p1 = (ptrLElement - rowSkip)[-3];
                            p2 = ptrLElement[-3];

                            /* compute outer product and add it to the Z matrix */
                            Z[0] += p1 * q1;
                            Z[1] += p2 * q1;
                            Z[2] += p3 * q1;
                            Z[3] += p4 * q1;

                            /* load p and q values */
                            q1 = ptrBElement[-2 * (int)b_stride];
                            p1 = (ptrLElement - rowSkip)[-2];
                            p2 = ptrLElement[-2];
                            ptrLElement += rowSkip;
                            p3 = ptrLElement[-2];
                            p4 = ptrLElement[-2 + rowSkip];

                            /* compute outer product and add it to the Z matrix */
                            Z[0] += p1 * q1;
                            Z[1] += p2 * q1;
                            Z[2] += p3 * q1;
                            Z[3] += p4 * q1;

                            /* load p and q values */
                            q1 = ptrBElement[-1 * (int)b_stride];
                            p3 = ptrLElement[-1];
                            p4 = ptrLElement[-1 + rowSkip];
                            ptrLElement -= rowSkip;
                            p1 = (ptrLElement - rowSkip)[-1];
                            p2 = ptrLElement[-1];

                            /* compute outer product and add it to the Z matrix */
                            Z[0] += p1 * q1;
                            Z[1] += p2 * q1;
                            Z[2] += p3 * q1;
                            Z[3] += p4 * q1;
                            dSASSERT(block_step == 4);
                        }
                        else
                        {
                            ptrLElement += block_step;
                            ptrBElement += block_step * b_stride;

                            if (--columnCounter == 0)
                            {
                                if (finalColumnBlock == currentBlock)
                                {
                                    rowEndReached = true;
                                    break;
                                }

                                // Take a look if any more columns have been completed...
                                completedBlocks = refBlockCompletionProgress;
                                dIASSERT(completedBlocks >= finalColumnBlock);

                                if (completedBlocks == finalColumnBlock)
                                {
                                    break;
                                }

                                // ...continue if so.
                                unsigned columnCompletedSoFar = finalColumnBlock;
                                finalColumnBlock = dMACRO_MIN(currentBlock, completedBlocks);
                                columnCounter = finalColumnBlock - columnCompletedSoFar;
                            }
                        }
                        /* end of inner loop */
                    }
                }
                else
                {
                    ptrLElement = L + (sizeint)(1/* + currentBlock * block_step*/) * rowSkip/* + completedColumnBlock * block_step*/;
                    ptrBElement = B/* + (sizeint)(completedColumnBlock * block_step) * b_stride*/;
                    dIASSERT(completedColumnBlock == 0);

                    rowEndReached = true;
                }
            }
            else
            {
                partialBlock = true;

                if (currentBlock != 0)
                {
                    dReal tempZ[dMACRO_MAX(block_step - 1U, 1U)] = { REAL(0.0), };

                    ptrLElement = L + (sizeint)(/*1 + */currentBlock * block_step) * rowSkip + completedColumnBlock * block_step;
                    ptrBElement = B + (sizeint)(completedColumnBlock * block_step) * b_stride;

                    /* the inner loop that computes outer products and adds them to Z */
                    finalColumnBlock = dMACRO_MIN(currentBlock, completedBlocks);
                    dIASSERT(completedColumnBlock != finalColumnBlock/* || currentBlock == 0*/);

                    for (unsigned partialRow = 0, columnCompletedSoFar = completedColumnBlock; ; )
                    {
                        dReal Z1 = 0, Z2 = 0, Z3 = 0, Z4 = 0;

                        for (unsigned columnCounter = finalColumnBlock - columnCompletedSoFar; ; )
                        {
                            dReal q1, q2, q3, q4, p1, p2, p3, p4;

                            /* load p and q values */
                            q1 = ptrBElement[0 * b_stride];
                            q2 = ptrBElement[1 * b_stride];
                            q3 = ptrBElement[2 * b_stride];
                            q4 = ptrBElement[3 * b_stride];
                            p1 = ptrLElement[0];
                            p2 = ptrLElement[1];
                            p3 = ptrLElement[2];
                            p4 = ptrLElement[3];

                            /* compute outer product and add it to the Z matrix */
                            Z1 += p1 * q1;
                            Z2 += p2 * q2;
                            Z3 += p3 * q3;
                            Z4 += p4 * q4;
                            dSASSERT(block_step == 4);

                            if (columnCounter > 3)
                            {
                                columnCounter -= 3;

                                ptrLElement += 3 * block_step;
                                ptrBElement += 3 * block_step * b_stride;
                            
                                /* load p and q values */
                                q1 = ptrBElement[-8 * (int)b_stride];
                                q2 = ptrBElement[-7 * (int)b_stride];
                                q3 = ptrBElement[-6 * (int)b_stride];
                                q4 = ptrBElement[-5 * (int)b_stride];
                                p1 = ptrLElement[-8];
                                p2 = ptrLElement[-7];
                                p3 = ptrLElement[-6];
                                p4 = ptrLElement[-5];

                                /* compute outer product and add it to the Z matrix */
                                Z1 += p1 * q1;
                                Z2 += p2 * q2;
                                Z3 += p3 * q3;
                                Z4 += p4 * q4;

                                /* load p and q values */
                                q1 = ptrBElement[-4 * (int)b_stride];
                                q2 = ptrBElement[-3 * (int)b_stride];
                                q3 = ptrBElement[-2 * (int)b_stride];
                                q4 = ptrBElement[-1 * (int)b_stride];
                                p1 = ptrLElement[-4];
                                p2 = ptrLElement[-3];
                                p3 = ptrLElement[-2];
                                p4 = ptrLElement[-1];

                                /* compute outer product and add it to the Z matrix */
                                Z1 += p1 * q1;
                                Z2 += p2 * q2;
                                Z3 += p3 * q3;
                                Z4 += p4 * q4;
                                dSASSERT(block_step == 4);
                            }
                            else
                            {
                                ptrLElement += block_step;
                                ptrBElement += block_step * b_stride;

                                if (--columnCounter == 0)
                                {
                                    break;
                                }
                            }
                            /* end of inner loop */
                        }

                        tempZ[partialRow] += Z1 + Z2 + Z3 + Z4;

                        if (++partialRow == loopX1RowCount)
                        {
                            // Here switch is used to avoid accessing Z by parametrized index. 
                            // So far all the accesses were performed by explicit constants
                            // what lets the compiler treat Z elements as individual variables 
                            // rather than array elements.
                            Z[0] += tempZ[0];

                            if (loopX1RowCount >= 2)
                            {
                                Z[1] += tempZ[1];

                                if (loopX1RowCount > 2)
                                {
                                    Z[2] += tempZ[2];
                                }
                            }
                            dSASSERT(block_step == 4);

                            if (finalColumnBlock == currentBlock)
                            {
                                if (loopX1RowCount > 2)
                                {
                                    // Correct the LElement so that it points to the second row
                                    //
                                    // Note, that ff there is just one partial row, it does not matter that 
                                    // the LElement will remain pointing at the first row, 
                                    // since the former is not going to be used in that case.
                                    ptrLElement -= /*(sizeint)*/rowSkip/* * (loopX1RowCount - 2)*/; dIASSERT(loopX1RowCount == 3);
                                }
                                dSASSERT(block_step == 4);

                                rowEndReached = true;
                                break;
                            }

                            // Take a look if any more columns have been completed...
                            completedBlocks = refBlockCompletionProgress;
                            dIASSERT(completedBlocks >= finalColumnBlock);

                            if (completedBlocks == finalColumnBlock)
                            {
                                break;
                            }

                            std::fill(tempZ, tempZ + loopX1RowCount, REAL(0.0));
                            partialRow = 0;

                            // Correct the LElement pointer to continue at the first partial row
                            ptrLElement -= (sizeint)rowSkip * (loopX1RowCount - 1);

                            // ...continue if so.
                            columnCompletedSoFar = finalColumnBlock;
                            finalColumnBlock = dMACRO_MIN(currentBlock, completedBlocks);
                        }
                        else
                        {
                            ptrLElement += rowSkip - (finalColumnBlock - columnCompletedSoFar) * block_step;
                            ptrBElement -= (sizeint)((finalColumnBlock - columnCompletedSoFar) * block_step) * b_stride;
                        }
                        /* end of loop by individual rows */
                    }
                }
                else
                {
                    ptrLElement = L + (sizeint)(1/* + currentBlock * block_step*/) * rowSkip/* + completedColumnBlock * block_step*/;
                    ptrBElement = B/* + (sizeint)(completedColumnBlock * block_step) * b_stride*/;
                    dIASSERT(completedColumnBlock == 0);

                    rowEndReached = true;
                }
            }

            if (rowEndReached)
            {
                // Check whether there is still a need to proceed or if the computation has been taken over by another thread
                cellindexint oldDescriptor = MAKE_CELLDESCRIPTOR(completedColumnBlock, previousContextInstance, true);

                if (blockProgressDescriptors[currentBlock] == oldDescriptor)
                {
                    /* finish computing the X(i) block */
                    if (!partialBlock)
                    {
                        Y[0] = ptrBElement[0 * b_stride] - Z[0];

                        dReal p2 = ptrLElement[0];
                        Y[1] = ptrBElement[1 * b_stride] - Z[1] - p2 * Y[0];

                        ptrLElement += rowSkip;

                        dReal p3 = ptrLElement[0];
                        dReal p3_1 = ptrLElement[1];
                        Y[2] = ptrBElement[2 * b_stride] - Z[2] - p3 * Y[0] - p3_1 * Y[1];

                        dReal p4 = ptrLElement[rowSkip];
                        dReal p4_1 = ptrLElement[1 + rowSkip];
                        dReal p4_2 = ptrLElement[2 + rowSkip];
                        Y[3] = ptrBElement[3 * b_stride] - Z[3] - p4 * Y[0] - p4_1 * Y[1] - p4_2 * Y[2];
                        dSASSERT(block_step == 4);
                    }
                    else
                    {
                        Y[0] = ptrBElement[0 * b_stride] - Z[0];

                        if (loopX1RowCount >= 2)
                        {
                            dReal p2 = ptrLElement[0];
                            Y[1] = ptrBElement[1 * b_stride] - Z[1] - p2 * Y[0];

                            if (loopX1RowCount > 2)
                            {
                                dReal p3 = ptrLElement[0 + rowSkip];
                                dReal p3_1 = ptrLElement[1 + rowSkip];
                                Y[2] = ptrBElement[2 * b_stride] - Z[2] - p3 * Y[0] - p3_1 * Y[1];
                            }
                        }
                        dSASSERT(block_step == 4);
                    }

                    // Use atomic memory barrier to make sure memory reads of ptrBElement[] and blockProgressDescriptors[] are not swapped
                    CooperativeAtomics::AtomicReadReorderBarrier();

                    // The descriptor has not been altered yet - this means the ptrBElement[] values used above were not modified yet 
                    // and the computation result is valid.
                    if (blockProgressDescriptors[currentBlock] == oldDescriptor)
                    {
                        // Assign the results to the result context (possibly in parallel with other threads 
                        // that could and ought to be assigning exactly the same values)
                        SolveL1StraightCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
                        resultContext.storePrecalculatedZs(Y);

                        // Assign the result assignment progress descriptor
                        cellindexint newDescriptor = MAKE_CELLDESCRIPTOR(currentBlock + 1, CCI__MIN, true);
                        CooperativeAtomics::AtomicCompareExchangeCellindexint(&blockProgressDescriptors[currentBlock], oldDescriptor, newDescriptor); // the result is to be ignored

                        // Whether succeeded or not, the result is valid, so go on trying to assign it to the matrix
                        goAssigningTheResult = true;
                    }
                    else
                    {
                        // Otherwise, go on competing for copying the results
                        handleComputationTakenOver = true;
                    }
                }
                else
                {
                    handleComputationTakenOver = true;
                }
            }
            else
            {
                // If the final column has not been reached yet, store current values to the context.
                // Select the other context instance as the previous one might be read by other threads.
                CellContextInstance nextContextInstance = buildNextContextInstance(previousContextInstance);
                SolveL1StraightCellContext &destinationContext = buildBlockContextRef(cellContexts, currentBlock, nextContextInstance);
                destinationContext.storePrecalculatedZs(Z);

                // Unlock the row until more columns can be used
                cellindexint oldDescriptor = MAKE_CELLDESCRIPTOR(completedColumnBlock, previousContextInstance, true);
                cellindexint newDescriptor = MAKE_CELLDESCRIPTOR(finalColumnBlock, nextContextInstance, false);
                // The descriptor might have been updated by a competing thread
                if (!CooperativeAtomics::AtomicCompareExchangeCellindexint(&blockProgressDescriptors[currentBlock], oldDescriptor, newDescriptor))
                {
                    // Adjust the ptrBElement to point to the result area...
                    ptrBElement = B + (sizeint)(currentBlock * block_step) * b_stride;
                    // ...and go on handling the case
                    handleComputationTakenOver = true;
                }
            }

            if (handleComputationTakenOver)
            {
                cellindexint existingDescriptor = blockProgressDescriptors[currentBlock];
                // This can only happen if the row was (has become) the uppermost not fully completed one
                // and the competing thread is at final stage of calculation (i.e., it has reached the currentBlock column).
                if (existingDescriptor != INVALID_CELLDESCRIPTOR)
                {
                    // If not fully completed this must be the final stage of the result assignment into the matrix
                    dIASSERT(existingDescriptor == MAKE_CELLDESCRIPTOR(currentBlock + 1, CCI__MIN, true));

                    // Go on competing copying the result as anyway the block is the topmost not completed one
                    // and since there was competition for it, there is no other work that can be done right now.
                    const SolveL1StraightCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
                    resultContext.loadPrecalculatedZs(Y);

                    goAssigningTheResult = true;
                }
                else 
                {
                    // everything is over -- just go handling next blocks
                }
            }
        }
        else if (goForLockedBlockDuplicateCalculation)
        {
            blockProcessingState = BPS_SOME_BLOCKS_PROCESSED;

            bool skipToHandlingSubsequentRows = false, skiptoCopyingResult = false;

            /* declare variables */
            const dReal *ptrLElement;

            if (completedColumnBlock < currentBlock)
            {
                /* check if this is not the partial block of fewer rows */
                if (currentBlock != lastBlock || loopX1RowCount == 0)
                {
                    partialBlock = false;

                    ptrLElement = L + (sizeint)(1 + currentBlock * block_step) * rowSkip + completedColumnBlock * block_step;
                    ptrBElement = B + (sizeint)(completedColumnBlock * block_step) * b_stride;

                    /* the inner loop that computes outer products and adds them to Z */
                    unsigned finalColumnBlock = currentBlock;
                    dIASSERT(currentBlock == completedBlocks); // Why would we be competing for a row otherwise?

                    unsigned lastCompletedColumn = completedColumnBlock;
                    unsigned columnCounter = finalColumnBlock - completedColumnBlock;
                    for (bool exitInnerLoop = false; !exitInnerLoop; exitInnerLoop = --columnCounter == 0)
                    {
                        dReal q1, p1, p2, p3, p4;

                        /* load p and q values */
                        q1 = ptrBElement[0 * b_stride];
                        p1 = (ptrLElement - rowSkip)[0];
                        p2 = ptrLElement[0];
                        ptrLElement += rowSkip;
                        p3 = ptrLElement[0];
                        p4 = ptrLElement[0 + rowSkip];

                        /* compute outer product and add it to the Z matrix */
                        Z[0] += p1 * q1;
                        Z[1] += p2 * q1;
                        Z[2] += p3 * q1;
                        Z[3] += p4 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[1 * b_stride];
                        p3 = ptrLElement[1];
                        p4 = ptrLElement[1 + rowSkip];
                        ptrLElement -= rowSkip;
                        p1 = (ptrLElement - rowSkip)[1];
                        p2 = ptrLElement[1];

                        /* compute outer product and add it to the Z matrix */
                        Z[0] += p1 * q1;
                        Z[1] += p2 * q1;
                        Z[2] += p3 * q1;
                        Z[3] += p4 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[2 * b_stride];
                        p1 = (ptrLElement - rowSkip)[2];
                        p2 = ptrLElement[2];
                        ptrLElement += rowSkip;
                        p3 = ptrLElement[2];
                        p4 = ptrLElement[2 + rowSkip];

                        /* compute outer product and add it to the Z matrix */
                        Z[0] += p1 * q1;
                        Z[1] += p2 * q1;
                        Z[2] += p3 * q1;
                        Z[3] += p4 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[3 * b_stride];
                        p3 = ptrLElement[3];
                        p4 = ptrLElement[3 + rowSkip];
                        ptrLElement -= rowSkip;
                        p1 = (ptrLElement - rowSkip)[3];
                        p2 = ptrLElement[3];

                        /* compute outer product and add it to the Z matrix */
                        Z[0] += p1 * q1;
                        Z[1] += p2 * q1;
                        Z[2] += p3 * q1;
                        Z[3] += p4 * q1;
                        dSASSERT(block_step == 4);

                        // Check if the primary solver thread has not made any progress
                        cellindexint descriptorVerification = blockProgressDescriptors[currentBlock];
                        unsigned newCompletedColumn = GET_CELLDESCRIPTOR_COLUMNINDEX(descriptorVerification);

                        if (newCompletedColumn != lastCompletedColumn)
                        {
                            // Check, this is the first change the current thread detects.
                            // There is absolutely no reason in code for the computation to stop/resume twice 
                            // while the current thread is competing.
                            dIASSERT(lastCompletedColumn == completedColumnBlock);

                            if (descriptorVerification == INVALID_CELLDESCRIPTOR)
                            {
                                skipToHandlingSubsequentRows = true;
                                break;
                            }

                            if (newCompletedColumn == currentBlock + 1)
                            {
                                skiptoCopyingResult = true;
                                break;
                            }

                            // Check if the current thread is behind
                            if (newCompletedColumn > finalColumnBlock - columnCounter)
                            {
                                // If so, go starting over one more time
                                blockProcessingState = BPS_COMPETING_FOR_A_BLOCK;
                                stayWithinTheBlock = true;
                                skipToHandlingSubsequentRows = true;
                                break;
                            }

                            // If current thread is ahead, just save new completed column for further comparisons and go on calculating
                            lastCompletedColumn = newCompletedColumn;
                        }

                        /* advance pointers */
                        ptrLElement += block_step;
                        ptrBElement += block_step * b_stride;
                        /* end of inner loop */
                    }
                }
                else
                {
                    partialBlock = true;

                    dReal tempZ[dMACRO_MAX(block_step - 1U, 1U)] = { REAL(0.0), };

                    ptrLElement = L + (sizeint)(/*1 + */currentBlock * block_step) * rowSkip + completedColumnBlock * block_step;
                    ptrBElement = B + (sizeint)(completedColumnBlock * block_step) * b_stride;

                    /* the inner loop that computes outer products and adds them to Z */
                    unsigned finalColumnBlock = currentBlock;
                    dIASSERT(currentBlock == completedBlocks); // Why would we be competing for a row otherwise?

                    unsigned lastCompletedColumn = completedColumnBlock;
                    for (unsigned columnCounter = finalColumnBlock - completedColumnBlock; ; )
                    {
                        dReal q1, q2, q3, q4;

                        /* load q values */
                        q1 = ptrBElement[0 * b_stride];
                        q2 = ptrBElement[1 * b_stride];
                        q3 = ptrBElement[2 * b_stride];
                        q4 = ptrBElement[3 * b_stride];

                        for (unsigned partialRow = 0; ; )
                        {
                            dReal p1, p2, p3, p4;

                            /* load p values */
                            p1 = ptrLElement[0];
                            p2 = ptrLElement[1];
                            p3 = ptrLElement[2];
                            p4 = ptrLElement[3];

                            /* compute outer product and add it to the Z matrix */
                            tempZ[partialRow] += p1 * q1 + p2 * q2 + p3 * q3 + p4 * q4;
                            dSASSERT(block_step == 4);

                            if (++partialRow == loopX1RowCount)
                            {
                                break;
                            }

                            ptrLElement += rowSkip;
                        }

                        // Check if the primary solver thread has not made any progress
                        cellindexint descriptorVerification = blockProgressDescriptors[currentBlock];
                        unsigned newCompletedColumn = GET_CELLDESCRIPTOR_COLUMNINDEX(descriptorVerification);

                        if (newCompletedColumn != lastCompletedColumn)
                        {
                            // Check, this is the first change the current thread detects.
                            // There is absolutely no reason in code for the computation to stop/resume twice 
                            // while the current thread is competing.
                            dIASSERT(lastCompletedColumn == completedColumnBlock);

                            if (descriptorVerification == INVALID_CELLDESCRIPTOR)
                            {
                                skipToHandlingSubsequentRows = true;
                                break;
                            }

                            if (newCompletedColumn == currentBlock + 1)
                            {
                                skiptoCopyingResult = true;
                                break;
                            }

                            // Check if the current thread is behind
                            if (newCompletedColumn > finalColumnBlock - columnCounter)
                            {
                                // If so, go starting over one more time
                                blockProcessingState = BPS_COMPETING_FOR_A_BLOCK;
                                stayWithinTheBlock = true;
                                skipToHandlingSubsequentRows = true;
                                break;
                            }

                            // If current thread is ahead, just save new completed column for further comparisons and go on calculating
                            lastCompletedColumn = newCompletedColumn;
                        }

                        ptrLElement += block_step;
                        ptrBElement += block_step * b_stride;

                        if (--columnCounter == 0)
                        {
                            // Here switch is used to avoid accessing Z by parametrized index. 
                            // So far all the accesses were performed by explicit constants
                            // what lets the compiler treat Z elements as individual variables 
                            // rather than array elements.
                            Z[0] += tempZ[0];

                            if (loopX1RowCount >= 2)
                            {
                                Z[1] += tempZ[1];

                                if (loopX1RowCount > 2)
                                {
                                    Z[2] += tempZ[2];

                                    // Correct the LElement so that it points to the second row
                                    //
                                    // Note, that if there is just one partial row, it does not matter that 
                                    // the LElement will remain pointing at the first row, 
                                    // since the former is not going to be used in that case.
                                    ptrLElement -= /*(sizeint)*/rowSkip/* * (loopX1RowCount - 2)*/; dIASSERT(loopX1RowCount == 3);
                                }
                            }
                            dSASSERT(block_step == 4);

                            break;
                        }

                        /* advance pointers */
                        ptrLElement -= (sizeint)rowSkip * (loopX1RowCount - 1);
                        /* end of inner loop */
                    }
                }
            }
            else if (completedColumnBlock > currentBlock)
            {
                dIASSERT(completedColumnBlock == currentBlock + 1);

                partialBlock = currentBlock == lastBlock && loopX1RowCount != 0;

                skiptoCopyingResult = true;
            }
            else
            {
                dIASSERT(currentBlock == 0); // Execution can get here within the very first block only

                partialBlock = rowCount < block_step;

                /* assign the pointers appropriately and go on computing the results */
                ptrLElement = L + (sizeint)(1/* + currentBlock * block_step*/) * rowSkip/* + completedColumnBlock * block_step*/;
                ptrBElement = B/* + (sizeint)(completedColumnBlock * block_step) * b_stride*/;
            }

            if (!skipToHandlingSubsequentRows)
            {
                if (!skiptoCopyingResult)
                {
                    if (!partialBlock)
                    {
                        Y[0] = ptrBElement[0 * b_stride] - Z[0];

                        dReal p2 = ptrLElement[0];
                        Y[1] = ptrBElement[1 * b_stride] - Z[1] - p2 * Y[0];

                        ptrLElement += rowSkip;

                        dReal p3 = ptrLElement[0];
                        dReal p3_1 = ptrLElement[1];
                        Y[2] = ptrBElement[2 * b_stride] - Z[2] - p3 * Y[0] - p3_1 * Y[1];

                        dReal p4 = ptrLElement[rowSkip];
                        dReal p4_1 = ptrLElement[1 + rowSkip];
                        dReal p4_2 = ptrLElement[2 + rowSkip];
                        Y[3] = ptrBElement[3 * b_stride] - Z[3] - p4 * Y[0] - p4_1 * Y[1] - p4_2 * Y[2];
                        dSASSERT(block_step == 4);
                    }
                    else
                    {
                        Y[0] = ptrBElement[0 * b_stride] - Z[0];

                        if (loopX1RowCount >= 2)
                        {
                            dReal p2 = ptrLElement[0];
                            Y[1] = ptrBElement[1 * b_stride] - Z[1] - p2 * Y[0];

                            if (loopX1RowCount > 2)
                            {
                                dReal p3 = ptrLElement[0 + rowSkip];
                                dReal p3_1 = ptrLElement[1 + rowSkip];
                                Y[2] = ptrBElement[2 * b_stride] - Z[2] - p3 * Y[0] - p3_1 * Y[1];
                            }
                        }
                        dSASSERT(block_step == 4);
                    }

                    CooperativeAtomics::AtomicReadReorderBarrier();

                    // Use atomic load to make sure memory reads of ptrBElement[] and blockProgressDescriptors[] are not swapped
                    cellindexint existingDescriptor = blockProgressDescriptors[currentBlock];

                    if (existingDescriptor == INVALID_CELLDESCRIPTOR)
                    {
                        // Everything is over -- proceed to subsequent rows
                        skipToHandlingSubsequentRows = true;
                    }
                    else if (existingDescriptor == MAKE_CELLDESCRIPTOR(currentBlock + 1, CCI__MIN, true))
                    {
                        // The values computed above may not be valid. Copy the values already in the result context.
                        skiptoCopyingResult = true;
                    }
                    else
                    {
                        // The descriptor has not been altered yet - this means the ptrBElement[] values used above were not modified yet 
                        // and the computation result is valid.
                        cellindexint newDescriptor = MAKE_CELLDESCRIPTOR(currentBlock + 1, CCI__MIN, true); // put the computation at the top so that the evaluation result from the expression above is reused

                        // Assign the results to the result context (possibly in parallel with other threads 
                        // that could and ought to be assigning exactly the same values)
                        SolveL1StraightCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
                        resultContext.storePrecalculatedZs(Y);

                        // Assign the result assignment progress descriptor
                        CooperativeAtomics::AtomicCompareExchangeCellindexint(&blockProgressDescriptors[currentBlock], existingDescriptor, newDescriptor); // the result is to be ignored

                        // Whether succeeded or not, the result is valid, so go on trying to assign it to the matrix
                    }
                }

                if (!skipToHandlingSubsequentRows)
                {
                    if (skiptoCopyingResult)
                    {
                        // Extract the result values stored in the result context
                        const SolveL1StraightCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
                        resultContext.loadPrecalculatedZs(Y);

                        ptrBElement = B + (sizeint)(currentBlock * block_step) * b_stride;
                    }

                    goAssigningTheResult = true;
                }
            }
        }

        if (goAssigningTheResult)
        {
            cellindexint existingDescriptor = blockProgressDescriptors[currentBlock];
            // Check if the assignment has not been completed yet
            if (existingDescriptor != INVALID_CELLDESCRIPTOR)
            {
                // Assign the computation results to the B vector
                if (!partialBlock)
                {
                    ptrBElement[0 * b_stride] = Y[0];
                    ptrBElement[1 * b_stride] = Y[1];
                    ptrBElement[2 * b_stride] = Y[2];
                    ptrBElement[3 * b_stride] = Y[3];
                    dSASSERT(block_step == 4);
                }
                else
                {
                    ptrBElement[0 * b_stride] = Y[0];

                    if (loopX1RowCount >= 2)
                    {
                        ptrBElement[1 * b_stride] = Y[1];

                        if (loopX1RowCount > 2)
                        {
                            ptrBElement[2 * b_stride] = Y[2];
                        }
                    }
                    dSASSERT(block_step == 4);
                }

                ThrsafeIncrementIntUpToLimit(&refBlockCompletionProgress, currentBlock + 1);
                dIASSERT(refBlockCompletionProgress >= currentBlock + 1);

                // And assign the completed status no matter what
                CooperativeAtomics::AtomicStoreCellindexint(&blockProgressDescriptors[currentBlock], INVALID_CELLDESCRIPTOR);
            }
            else 
            {
                // everything is over -- just go handling next blocks
            }
        }

        if (!stayWithinTheBlock)
        {
            completedBlocks = refBlockCompletionProgress;

            if (completedBlocks == blockCount)
            {
                break;
            }

            currentBlock += 1;

            bool lookaheadBoundaryReached = false;

            if (currentBlock == blockCount || completedBlocks == 0)
            {
                lookaheadBoundaryReached = true;
            }
            else if (currentBlock >= completedBlocks + lookaheadRange)
            {
                lookaheadBoundaryReached = blockProcessingState > BPS_NO_BLOCKS_PROCESSED;
            }
            else if (currentBlock < completedBlocks)
            {
                // Treat detected row advancement as a row processed
                // blockProcessingState = BPS_SOME_BLOCKS_PROCESSED; <-- performs better without it

                currentBlock = completedBlocks;
            }

            if (lookaheadBoundaryReached)
            {
                dIASSERT(blockProcessingState != BPS_COMPETING_FOR_A_BLOCK); // Why did not we compete???

                // If no row has been processed in the previous pass, compete for the next row to avoid cycling uselessly
                if (blockProcessingState <= BPS_NO_BLOCKS_PROCESSED)
                {
                    // Abandon job if too few blocks remain
                    if (blockCount - completedBlocks <= ownThreadIndex)
                    {
                        break;
                    }

                    blockProcessingState = BPS_COMPETING_FOR_A_BLOCK;
                }
                else
                {
                    // If there was some progress, just continue to the next pass
                    blockProcessingState = BPS_NO_BLOCKS_PROCESSED;
                }

                currentBlock = completedBlocks;
            }
        }
    }
}


#endif
