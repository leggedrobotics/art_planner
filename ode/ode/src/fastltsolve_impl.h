

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
 * L1Transposed cooperative solving code of ThreadedEquationSolverLDLT copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")  
 */


#ifndef _ODE_FASTLTSOLVE_IMPL_H_
#define _ODE_FASTLTSOLVE_IMPL_H_


/* solve L^T * x=b, with b containing 1 right hand side.
 * L is an n*n lower triangular matrix with ones on the diagonal.
 * L is stored by rows and its leading dimension is rowSkip.
 * b is an n*1 matrix that contains the right hand side.
 * b is overwritten with x.
 * this processes blocks of 4.
 */

template<unsigned int b_stride>
void solveL1Transposed(const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip)
{
    dIASSERT(rowCount != 0);

    /* special handling for L and B because we're solving L1 *transpose* */
    const dReal *lastLElement = L + (sizeint)(rowCount - 1) * (rowSkip + 1);
    dReal *lastBElement = B + (sizeint)(rowCount - 1) * b_stride;

    /* compute rows at end that are not a multiple of block size */
    const unsigned loopX1RowCount = rowCount % 4;
    
    unsigned blockStartRow = loopX1RowCount;
    bool subsequentPass  = false;

    /* compute rightmost bottom X(i) block */
    if (loopX1RowCount != 0)
    {
        subsequentPass = true;

        const dReal *ptrLElement = lastLElement;
        dReal *ptrBElement = lastBElement;

        dReal Y11 = ptrBElement[0 * b_stride]/* - Z11*/;
        // ptrBElement[0 * b_stride] = Y11; -- unchanged

        if (loopX1RowCount >= 2)
        {
            dReal p2 = ptrLElement[-1];
            dReal Y21 = ptrBElement[-1 * (int)b_stride]/* - Z21 */- p2 * Y11;
            ptrBElement[-1 * (int)b_stride] = Y21;
            
            if (loopX1RowCount > 2)
            {
                dReal p3 = ptrLElement[-2];
                dReal p3_1 = (ptrLElement - rowSkip)[-2];
                dReal Y31 = ptrBElement[-2 * (int)b_stride]/* - Z31 */- p3 * Y11 - p3_1 * Y21;
                ptrBElement[-2 * (int)b_stride] = Y31;
            }
        }
    }
    
    /* compute all 4 x 1 blocks of X */
    for (; !subsequentPass || blockStartRow < rowCount; subsequentPass = true, blockStartRow += 4)
    {
        /* compute all 4 x 1 block of X, from rows i..i+4-1 */

        /* declare variables - Z matrix, p and q vectors, etc */
        const dReal *ptrLElement;
        dReal *ptrBElement;

        dReal Z41, Z31, Z21, Z11;

        if (subsequentPass)
        {
            ptrLElement = lastLElement - blockStartRow;
            ptrBElement = lastBElement;

            /* set the Z matrix to 0 */
            Z41 = 0; Z31 = 0; Z21 = 0; Z11 = 0;

            unsigned rowCounter = blockStartRow;

            if (rowCounter % 2 != 0)
            {
                dReal q1, p4, p3, p2, p1;

                /* load p and q values */
                q1 = ptrBElement[0 * (int)b_stride];
                p4 = ptrLElement[-3];
                p3 = ptrLElement[-2];
                p2 = ptrLElement[-1];
                p1 = ptrLElement[0];
                ptrLElement -= rowSkip;

                /* compute outer product and add it to the Z matrix */
                Z41 += p4 * q1;
                Z31 += p3 * q1;
                Z21 += p2 * q1;
                Z11 += p1 * q1;

                ptrBElement -= 1 * b_stride;
                rowCounter -= 1;
            }

            if (rowCounter % 4 != 0)
            {
                dReal q1, p4, p3, p2, p1;

                /* load p and q values */
                q1 = ptrBElement[0 * (int)b_stride];
                p4 = ptrLElement[-3];
                p3 = ptrLElement[-2];
                p2 = ptrLElement[-1];
                p1 = ptrLElement[0];
                ptrLElement -= rowSkip;

                /* compute outer product and add it to the Z matrix */
                Z41 += p4 * q1;
                Z31 += p3 * q1;
                Z21 += p2 * q1;
                Z11 += p1 * q1;

                /* load p and q values */
                q1 = ptrBElement[-1 * (int)b_stride];
                p4 = ptrLElement[-3];
                p3 = ptrLElement[-2];
                p2 = ptrLElement[-1];
                p1 = ptrLElement[0];
                ptrLElement -= rowSkip;

                /* compute outer product and add it to the Z matrix */
                Z41 += p4 * q1;
                Z31 += p3 * q1;
                Z21 += p2 * q1;
                Z11 += p1 * q1;

                ptrBElement -= 2 * b_stride;
                rowCounter -= 2;
            }

            /* the inner loop that computes outer products and adds them to Z */
            for (bool exitLoop = rowCounter == 0; !exitLoop; exitLoop = false)
            {
                dReal q1, p4, p3, p2, p1;

                /* load p and q values */
                q1 = ptrBElement[0 * (int)b_stride];
                p4 = ptrLElement[-3];
                p3 = ptrLElement[-2];
                p2 = ptrLElement[-1];
                p1 = ptrLElement[0];
                ptrLElement -= rowSkip;

                /* compute outer product and add it to the Z matrix */
                Z41 += p4 * q1;
                Z31 += p3 * q1;
                Z21 += p2 * q1;
                Z11 += p1 * q1;

                /* load p and q values */
                q1 = ptrBElement[-1 * (int)b_stride];
                p4 = ptrLElement[-3];
                p3 = ptrLElement[-2];
                p2 = ptrLElement[-1];
                p1 = ptrLElement[0];
                ptrLElement -= rowSkip;

                /* compute outer product and add it to the Z matrix */
                Z41 += p4 * q1;
                Z31 += p3 * q1;
                Z21 += p2 * q1;
                Z11 += p1 * q1;

                /* load p and q values */
                q1 = ptrBElement[-2 * (int)b_stride];
                p4 = ptrLElement[-3];
                p3 = ptrLElement[-2];
                p2 = ptrLElement[-1];
                p1 = ptrLElement[0];
                ptrLElement -= rowSkip;

                /* compute outer product and add it to the Z matrix */
                Z41 += p4 * q1;
                Z31 += p3 * q1;
                Z21 += p2 * q1;
                Z11 += p1 * q1;

                /* load p and q values */
                q1 = ptrBElement[-3 * (int)b_stride];
                p4 = ptrLElement[-3];
                p3 = ptrLElement[-2];
                p2 = ptrLElement[-1];
                p1 = ptrLElement[0];
                ptrLElement -= rowSkip;

                /* compute outer product and add it to the Z matrix */
                Z41 += p4 * q1;
                Z31 += p3 * q1;
                Z21 += p2 * q1;
                Z11 += p1 * q1;

                if (rowCounter > 12)
                {
                    rowCounter -= 12;

                    ptrBElement -= 12 * b_stride;

                    /* load p and q values */
                    q1 = ptrBElement[8 * b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z41 += p4 * q1;
                    Z31 += p3 * q1;
                    Z21 += p2 * q1;
                    Z11 += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[7 * b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z41 += p4 * q1;
                    Z31 += p3 * q1;
                    Z21 += p2 * q1;
                    Z11 += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[6 * b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z41 += p4 * q1;
                    Z31 += p3 * q1;
                    Z21 += p2 * q1;
                    Z11 += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[5 * b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z41 += p4 * q1;
                    Z31 += p3 * q1;
                    Z21 += p2 * q1;
                    Z11 += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[4 * b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z41 += p4 * q1;
                    Z31 += p3 * q1;
                    Z21 += p2 * q1;
                    Z11 += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[3 * b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z41 += p4 * q1;
                    Z31 += p3 * q1;
                    Z21 += p2 * q1;
                    Z11 += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[2 * b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z41 += p4 * q1;
                    Z31 += p3 * q1;
                    Z21 += p2 * q1;
                    Z11 += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[1 * b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z41 += p4 * q1;
                    Z31 += p3 * q1;
                    Z21 += p2 * q1;
                    Z11 += p1 * q1;
                }
                else
                {
                    ptrBElement -= 4 * b_stride;

                    if ((rowCounter -= 4) == 0)
                    {
                        break;
                    }
                }
                /* end of inner loop */
            }
        }
        else
        {
            ptrLElement = lastLElement/* - blockStartRow*/; dIASSERT(blockStartRow == 0);
            ptrBElement = lastBElement;

            /* set the Z matrix to 0 */
            Z41 = 0; Z31 = 0; Z21 = 0; Z11 = 0;
        }

        /* finish computing the X(i) block */
        dReal Y11, Y21, Y31, Y41;
        {
            Y11 = ptrBElement[0 * b_stride] - Z11;
            ptrBElement[0 * b_stride] = Y11;
        }
        {
            dReal p2 = ptrLElement[-1];
            Y21 = ptrBElement[-1 * (int)b_stride] - Z21 - p2 * Y11;
            ptrBElement[-1 * (int)b_stride] = Y21;
        }
        {
            dReal p3 = ptrLElement[-2];
            dReal p3_1 = (ptrLElement - rowSkip)[-2];
            Y31 = ptrBElement[-2 * (int)b_stride] - Z31 - p3 * Y11 - p3_1 * Y21;
            ptrBElement[-2 * (int)b_stride] = Y31;
        }
        {
            dReal p4 = ptrLElement[-3];
            dReal p4_1 = (ptrLElement - rowSkip)[-3];
            dReal p4_2 = (ptrLElement - rowSkip * 2)[-3];
            Y41 = ptrBElement[-3 * (int)b_stride] - Z41 - p4 * Y11 - p4_1 * Y21 - p4_2 * Y31;
            ptrBElement[-3 * (int)b_stride] = Y41;
        }
        /* end of outer loop */
    }
}



template<unsigned int block_step>
/*static */
sizeint ThreadedEquationSolverLDLT::estimateCooperativelySolvingL1TransposedMemoryRequirement(unsigned rowCount, SolvingL1TransposedMemoryEstimates &ref_solvingMemoryEstimates)
{
    unsigned blockCount = deriveSolvingL1TransposedBlockCount(rowCount, block_step);
    sizeint descriptorSizeRequired = dEFFICIENT_SIZE(sizeof(cellindexint) * blockCount);
    sizeint contextSizeRequired = dEFFICIENT_SIZE(sizeof(SolveL1TransposedCellContext) * (CCI__MAX + 1) * blockCount);
    ref_solvingMemoryEstimates.assignData(descriptorSizeRequired, contextSizeRequired);

    sizeint totalSizeRequired = descriptorSizeRequired + contextSizeRequired;
    return totalSizeRequired;
}

template<unsigned int block_step>
/*static */
void ThreadedEquationSolverLDLT::initializeCooperativelySolveL1TransposedMemoryStructures(unsigned rowCount, 
    atomicord32 &out_blockCompletionProgress, cellindexint *blockProgressDescriptors, SolveL1TransposedCellContext *dUNUSED(cellContexts))
{
    unsigned blockCount = deriveSolvingL1TransposedBlockCount(rowCount, block_step);

    out_blockCompletionProgress = 0;
    memset(blockProgressDescriptors, 0, blockCount * sizeof(*blockProgressDescriptors));
}

template<unsigned int block_step, unsigned int b_stride>
/*static */
void ThreadedEquationSolverLDLT::participateSolvingL1Transposed(const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip, 
    volatile atomicord32 &refBlockCompletionProgress/*=0*/, volatile cellindexint *blockProgressDescriptors/*=[blockCount]*/, 
    SolveL1TransposedCellContext *cellContexts/*=[CCI__MAX x blockCount] + [blockCount]*/, unsigned ownThreadIndex)
{
    const unsigned lookaheadRange = 32;
    const unsigned blockCount = deriveSolvingL1TransposedBlockCount(rowCount, block_step);
    /* compute rows at end that are not a multiple of block size */
    const unsigned loopX1RowCount = rowCount % block_step;

    /* special handling for L and B because we're solving L1 *transpose* */
    const dReal *lastLElement = L + (rowCount - 1) * ((sizeint)rowSkip + 1);
    dReal *lastBElement = B + (rowCount - 1) * (sizeint)b_stride;

    /* elements adjusted as if the last block was full block_step elements */
    unsigned x1AdjustmentElements = (block_step - loopX1RowCount) % block_step;
    const dReal *columnAdjustedLastLElement = lastLElement + x1AdjustmentElements;
    const dReal *fullyAdjustedLastLElement = columnAdjustedLastLElement + (sizeint)rowSkip * x1AdjustmentElements;
    dReal *adjustedLastBElement = lastBElement + b_stride * x1AdjustmentElements;

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
        unsigned completedRowBlock;
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
                completedRowBlock = GET_CELLDESCRIPTOR_COLUMNINDEX(testDescriptor);
                dIASSERT(completedRowBlock < currentBlock || (completedRowBlock == currentBlock && currentBlock == 0)); // Otherwise, why would the calculation have had stopped if the final column is reachable???
                dIASSERT(completedRowBlock <= completedBlocks); // Since the descriptor is not locked

                if (completedRowBlock == completedBlocks && currentBlock != completedBlocks)
                {
                    dIASSERT(completedBlocks < currentBlock);
                    break;
                }

                if (CooperativeAtomics::AtomicCompareExchangeCellindexint(&blockProgressDescriptors[currentBlock], testDescriptor, MARK_CELLDESCRIPTOR_LOCKED(testDescriptor)))
                {
                    if (completedRowBlock != 0)
                    {
                        CellContextInstance contextInstance = GET_CELLDESCRIPTOR_CONTEXTINSTANCE(testDescriptor);
                        previousContextInstance = contextInstance;

                        const SolveL1TransposedCellContext &sourceContext = buildBlockContextRef(cellContexts, currentBlock, contextInstance);
                        sourceContext.loadPrecalculatedZs(Z);
                    }
                    else
                    {
                        previousContextInstance = CCI__MIN;
                        SolveL1TransposedCellContext::initializePrecalculatedZs(Z);
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

                completedRowBlock = GET_CELLDESCRIPTOR_COLUMNINDEX(testDescriptor);
                dIASSERT(completedRowBlock != currentBlock || currentBlock == 0); // There is no reason for computations to stop at the very end other than being the initial value at the very first block

                if (completedRowBlock != 0)
                {
                    CellContextInstance contextInstance = GET_CELLDESCRIPTOR_CONTEXTINSTANCE(testDescriptor);
                    const SolveL1TransposedCellContext &sourceContext = buildBlockContextRef(cellContexts, currentBlock, contextInstance);
                    sourceContext.loadPrecalculatedZs(Z);
                }
                else
                {
                    SolveL1TransposedCellContext::initializePrecalculatedZs(Z);
                }

                if (completedRowBlock != 0 && completedRowBlock <= currentBlock)
                {
                    // Make sure the descriptor is re-read after the precalculates
                    CooperativeAtomics::AtomicReadReorderBarrier();
                }

                if (completedRowBlock <= currentBlock)
                {
                    verificativeDescriptor = blockProgressDescriptors[currentBlock];
                    verificationFailure = verificativeDescriptor != testDescriptor;
                }

                if (!verificationFailure)
                {
                    dIASSERT(completedRowBlock <= currentBlock + 1);

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
            bool handleComputationTakenOver = false, columnEndReached = false;
            
            const dReal *ptrLElement;
            unsigned finalRowBlock;

            /* check if this is not the partial block of fewer rows */
            if (currentBlock != 0 || loopX1RowCount == 0)
            {
                partialBlock = false;

                ptrLElement = completedRowBlock != 0 
                    ? fullyAdjustedLastLElement - currentBlock * block_step - (sizeint)(completedRowBlock * block_step) * rowSkip 
                    : columnAdjustedLastLElement - currentBlock * block_step;
                ptrBElement = completedRowBlock != 0 
                    ? adjustedLastBElement - (sizeint)(completedRowBlock * block_step) * b_stride 
                    : lastBElement;

                finalRowBlock = dMACRO_MIN(currentBlock, completedBlocks);
                dIASSERT(finalRowBlock != completedRowBlock || finalRowBlock == 0);

                unsigned rowCounter = finalRowBlock - completedRowBlock;
                bool exitLoop = rowCounter == 0;

                if (exitLoop)
                {
                    columnEndReached = true;
                }
                else if (completedRowBlock == 0 && currentBlock != 0 && loopX1RowCount != 0)
                {
                    if ((loopX1RowCount & 1) != 0)
                    {
                        dReal q1, p4, p3, p2, p1;

                        /* load p and q values */
                        q1 = ptrBElement[0 * (int)b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        ptrBElement -= 1 * b_stride;
                    }

                    if ((loopX1RowCount & 2) != 0)
                    {
                        dReal q1, p4, p3, p2, p1;

                        /* load p and q values */
                        q1 = ptrBElement[0 * (int)b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[-1 * (int)b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        ptrBElement -= 2 * b_stride;
                    }
                    dSASSERT(block_step == 4);

                    if (--rowCounter == 0)
                    {
                        do 
                        {
                            if (finalRowBlock == currentBlock)
                            {
                                columnEndReached = true;
                                exitLoop = true;
                                break;
                            }

                            // Take a look if any more columns have been completed...
                            completedBlocks = refBlockCompletionProgress;
                            dIASSERT(completedBlocks >= finalRowBlock);

                            if (completedBlocks == finalRowBlock)
                            {
                                exitLoop = true;
                                break;
                            }

                            // ...continue if so.
                            unsigned rowCompletedSoFar = finalRowBlock;
                            finalRowBlock = dMACRO_MIN(currentBlock, completedBlocks);
                            rowCounter = finalRowBlock - rowCompletedSoFar;
                        }
                        while (false);
                    }
                }

                for (; !exitLoop; exitLoop = false)
                {
                    dReal q1, p4, p3, p2, p1;

                    /* load p and q values */
                    q1 = ptrBElement[0 * (int)b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z[3] += p4 * q1;
                    Z[2] += p3 * q1;
                    Z[1] += p2 * q1;
                    Z[0] += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-1 * (int)b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z[3] += p4 * q1;
                    Z[2] += p3 * q1;
                    Z[1] += p2 * q1;
                    Z[0] += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-2 * (int)b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z[3] += p4 * q1;
                    Z[2] += p3 * q1;
                    Z[1] += p2 * q1;
                    Z[0] += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-3 * (int)b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z[3] += p4 * q1;
                    Z[2] += p3 * q1;
                    Z[1] += p2 * q1;
                    Z[0] += p1 * q1;
                    dSASSERT(block_step == 4);

                    if (rowCounter > 3)
                    {
                        rowCounter -= 3;

                        ptrBElement -= 3 * block_step * b_stride;

                        /* load p and q values */
                        q1 = ptrBElement[8 * b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[7 * b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[6 * b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[5 * b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[4 * b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[3 * b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[2 * b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[1 * b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;
                        dSASSERT(block_step == 4);
                    }
                    else
                    {
                        ptrBElement -= block_step * b_stride;

                        if (--rowCounter == 0)
                        {
                            if (finalRowBlock == currentBlock)
                            {
                                columnEndReached = true;
                                break;
                            }

                            // Take a look if any more columns have been completed...
                            completedBlocks = refBlockCompletionProgress;
                            dIASSERT(completedBlocks >= finalRowBlock);

                            if (completedBlocks == finalRowBlock)
                            {
                                break;
                            }

                            // ...continue if so.
                            unsigned rowCompletedSoFar = finalRowBlock;
                            finalRowBlock = dMACRO_MIN(currentBlock, completedBlocks);
                            rowCounter = finalRowBlock - rowCompletedSoFar;
                        }
                    }
                    /* end of inner loop */
                }
            }
            else /* compute rightmost bottom X(i) block */
            {
                partialBlock = true;

                ptrLElement = lastLElement;
                ptrBElement = lastBElement;
                dIASSERT(completedRowBlock == 0);

                columnEndReached = true;
            }

            if (columnEndReached)
            {
                // Check whether there is still a need to proceed or if the computation has been taken over by another thread
                cellindexint oldDescriptor = MAKE_CELLDESCRIPTOR(completedRowBlock, previousContextInstance, true);

                if (blockProgressDescriptors[currentBlock] == oldDescriptor)
                {
                    if (partialBlock)
                    {
                        Y[0] = ptrBElement[0 * b_stride]/* - Z[0]*/;

                        if (loopX1RowCount >= 2)
                        {
                            dReal p2 = ptrLElement[-1];
                            Y[1] = ptrBElement[-1 * (int)b_stride]/* - Z[1] */- p2 * Y[0];

                            if (loopX1RowCount > 2)
                            {
                                dReal p3 = ptrLElement[-2];
                                dReal p3_1 = (ptrLElement - rowSkip)[-2];
                                Y[2] = ptrBElement[-2 * (int)b_stride]/* - Z[2] */- p3 * Y[0] - p3_1 * Y[1];
                            }
                        }

                        dSASSERT(block_step == 4);
                    }
                    else
                    {
                        Y[0] = ptrBElement[0 * b_stride] - Z[0];

                        dReal p2 = ptrLElement[-1];
                        Y[1] = ptrBElement[-1 * (int)b_stride] - Z[1] - p2 * Y[0];

                        dReal p3 = ptrLElement[-2];
                        dReal p3_1 = (ptrLElement - rowSkip)[-2];
                        Y[2] = ptrBElement[-2 * (int)b_stride] - Z[2] - p3 * Y[0] - p3_1 * Y[1];

                        dReal p4 = ptrLElement[-3];
                        dReal p4_1 = (ptrLElement - rowSkip)[-3];
                        dReal p4_2 = (ptrLElement - rowSkip * 2)[-3];
                        Y[3] = ptrBElement[-3 * (int)b_stride] - Z[3] - p4 * Y[0] - p4_1 * Y[1] - p4_2 * Y[2];
                        
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
                        SolveL1TransposedCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
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
                SolveL1TransposedCellContext &destinationContext = buildBlockContextRef(cellContexts, currentBlock, nextContextInstance);
                destinationContext.storePrecalculatedZs(Z);

                // Unlock the row until more columns can be used
                cellindexint oldDescriptor = MAKE_CELLDESCRIPTOR(completedRowBlock, previousContextInstance, true);
                cellindexint newDescriptor = MAKE_CELLDESCRIPTOR(finalRowBlock, nextContextInstance, false);
                // The descriptor might have been updated by a competing thread
                if (!CooperativeAtomics::AtomicCompareExchangeCellindexint(&blockProgressDescriptors[currentBlock], oldDescriptor, newDescriptor))
                {
                    // Adjust the ptrBElement to point to the result area...
                    ptrBElement = adjustedLastBElement - (sizeint)(currentBlock * block_step) * b_stride;
                    dIASSERT(currentBlock != 0 || adjustedLastBElement == lastBElement);
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
                    const SolveL1TransposedCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
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

            if (completedRowBlock < currentBlock)
            {
                partialBlock = false;

                ptrLElement = completedRowBlock != 0 
                    ? fullyAdjustedLastLElement - currentBlock * block_step - (sizeint)(completedRowBlock * block_step) * rowSkip
                    : columnAdjustedLastLElement - currentBlock * block_step;
                ptrBElement = completedRowBlock != 0 
                    ? adjustedLastBElement - (sizeint)(completedRowBlock * block_step) * b_stride 
                    : lastBElement;

                unsigned finalRowBlock = currentBlock/*std::min(currentBlock, completedBlocks)*/;
                dIASSERT(currentBlock == completedBlocks); // Why would we be competing for a row otherwise?

                bool exitInnerLoop = false;
                unsigned lastCompletedRow = completedRowBlock;
                unsigned rowCounter = finalRowBlock - completedRowBlock;

                if (completedRowBlock == 0/* && currentBlock != 0 */&& loopX1RowCount != 0)
                {
                    if ((loopX1RowCount & 1) != 0)
                    {
                        dReal q1, p4, p3, p2, p1;

                        /* load p and q values */
                        q1 = ptrBElement[0 * (int)b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        ptrBElement -= 1 * b_stride;
                    }

                    if ((loopX1RowCount & 2) != 0)
                    {
                        dReal q1, p4, p3, p2, p1;

                        /* load p and q values */
                        q1 = ptrBElement[0 * (int)b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        /* load p and q values */
                        q1 = ptrBElement[-1 * (int)b_stride];
                        p4 = ptrLElement[-3];
                        p3 = ptrLElement[-2];
                        p2 = ptrLElement[-1];
                        p1 = ptrLElement[0];
                        ptrLElement -= rowSkip;

                        /* compute outer product and add it to the Z matrix */
                        Z[3] += p4 * q1;
                        Z[2] += p3 * q1;
                        Z[1] += p2 * q1;
                        Z[0] += p1 * q1;

                        ptrBElement -= 2 * b_stride;
                    }
                    dSASSERT(block_step == 4);

                    if (--rowCounter == 0)
                    {
                        exitInnerLoop = true;
                    }
                }

                for (; !exitInnerLoop; exitInnerLoop = --rowCounter == 0)
                {
                    dReal q1, p4, p3, p2, p1;

                    /* load p and q values */
                    q1 = ptrBElement[0 * (int)b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z[3] += p4 * q1;
                    Z[2] += p3 * q1;
                    Z[1] += p2 * q1;
                    Z[0] += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-1 * (int)b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z[3] += p4 * q1;
                    Z[2] += p3 * q1;
                    Z[1] += p2 * q1;
                    Z[0] += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-2 * (int)b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z[3] += p4 * q1;
                    Z[2] += p3 * q1;
                    Z[1] += p2 * q1;
                    Z[0] += p1 * q1;

                    /* load p and q values */
                    q1 = ptrBElement[-3 * (int)b_stride];
                    p4 = ptrLElement[-3];
                    p3 = ptrLElement[-2];
                    p2 = ptrLElement[-1];
                    p1 = ptrLElement[0];
                    ptrLElement -= rowSkip;

                    /* compute outer product and add it to the Z matrix */
                    Z[3] += p4 * q1;
                    Z[2] += p3 * q1;
                    Z[1] += p2 * q1;
                    Z[0] += p1 * q1;
                    dSASSERT(block_step == 4);

                    // Check if the primary solver thread has not made any progress
                    cellindexint descriptorVerification = blockProgressDescriptors[currentBlock];
                    unsigned newCompletedRow = GET_CELLDESCRIPTOR_COLUMNINDEX(descriptorVerification);

                    if (newCompletedRow != lastCompletedRow)
                    {
                        // Check, this is the first change the current thread detects.
                        // There is absolutely no reason in code for the computation to stop/resume twice 
                        // while the current thread is competing.
                        dIASSERT(lastCompletedRow == completedRowBlock);

                        if (descriptorVerification == INVALID_CELLDESCRIPTOR)
                        {
                            skipToHandlingSubsequentRows = true;
                            break;
                        }

                        if (newCompletedRow == currentBlock + 1)
                        {
                            skiptoCopyingResult = true;
                            break;
                        }

                        // Check if the current thread is behind
                        if (newCompletedRow > finalRowBlock - rowCounter)
                        {
                            // If so, go starting over one more time
                            blockProcessingState = BPS_COMPETING_FOR_A_BLOCK;
                            stayWithinTheBlock = true;
                            skipToHandlingSubsequentRows = true;
                            break;
                        }

                        // If current thread is ahead, just save new completed column for further comparisons and go on calculating
                        lastCompletedRow = newCompletedRow;
                    }

                    /* advance pointers */
                    ptrBElement -= block_step * b_stride;
                    /* end of inner loop */
                }
            }
            else if (completedRowBlock > currentBlock)
            {
                dIASSERT(completedRowBlock == currentBlock + 1);

                partialBlock = currentBlock == 0 && loopX1RowCount != 0;

                skiptoCopyingResult = true;
            }
            else
            {
                dIASSERT(currentBlock == 0); // Execution can get here within the very first block only
                
                partialBlock = /*currentBlock == 0 && */loopX1RowCount != 0;

                /* just assign the pointers appropriately and go on computing the results */
                ptrLElement = lastLElement;
                ptrBElement = lastBElement;
            }

            if (!skipToHandlingSubsequentRows)
            {
                if (!skiptoCopyingResult)
                {
                    if (partialBlock)
                    {
                        Y[0] = ptrBElement[0 * b_stride]/* - Z[0]*/;

                        if (loopX1RowCount >= 2)
                        {
                            dReal p2 = ptrLElement[-1];
                            Y[1] = ptrBElement[-1 * (int)b_stride]/* - Z[1] */- p2 * Y[0];

                            if (loopX1RowCount > 2)
                            {
                                dReal p3 = ptrLElement[-2];
                                dReal p3_1 = (ptrLElement - rowSkip)[-2];
                                Y[2] = ptrBElement[-2 * (int)b_stride]/* - Z[2] */- p3 * Y[0] - p3_1 * Y[1];
                            }
                        }

                        dSASSERT(block_step == 4);
                    }
                    else
                    {
                        Y[0] = ptrBElement[0 * b_stride] - Z[0];

                        dReal p2 = ptrLElement[-1];
                        Y[1] = ptrBElement[-1 * (int)b_stride] - Z[1] - p2 * Y[0];

                        dReal p3 = ptrLElement[-2];
                        dReal p3_1 = (ptrLElement - rowSkip)[-2];
                        Y[2] = ptrBElement[-2 * (int)b_stride] - Z[2] - p3 * Y[0] - p3_1 * Y[1];

                        dReal p4 = ptrLElement[-3];
                        dReal p4_1 = (ptrLElement - rowSkip)[-3];
                        dReal p4_2 = (ptrLElement - rowSkip * 2)[-3];
                        Y[3] = ptrBElement[-3 * (int)b_stride] - Z[3] - p4 * Y[0] - p4_1 * Y[1] - p4_2 * Y[2];
                        
                        dSASSERT(block_step == 4);
                    }

                    // Use atomic memory barrier to make sure memory reads of ptrBElement[] and blockProgressDescriptors[] are not swapped
                    CooperativeAtomics::AtomicReadReorderBarrier();

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
                        SolveL1TransposedCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
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
                        const SolveL1TransposedCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
                        resultContext.loadPrecalculatedZs(Y);

                        ptrBElement = currentBlock != 0 ? adjustedLastBElement - (sizeint)(currentBlock * block_step) * b_stride : lastBElement;
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
                // Assign the computation results to B vector
                if (partialBlock)
                {
                    // ptrBElement[0 * b_stride] = Y[0]; -- unchanged

                    if (loopX1RowCount >= 2)
                    {
                        ptrBElement[-1 * (int)b_stride] = Y[1];

                        if (loopX1RowCount > 2)
                        {
                            ptrBElement[-2 * (int)b_stride] = Y[2];
                        }
                    }
                    dSASSERT(block_step == 4);
                }
                else
                {
                    ptrBElement[0 * b_stride] = Y[0];
                    ptrBElement[-1 * (int)b_stride] = Y[1];
                    ptrBElement[-2 * (int)b_stride] = Y[2];
                    ptrBElement[-3 * (int)b_stride] = Y[3];
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
