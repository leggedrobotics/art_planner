

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
 * LDLT cooperative factorization code of ThreadedEquationSolverLDLT copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")  
 */

#ifndef _ODE_FASTLDLT_IMPL_H_
#define _ODE_FASTLDLT_IMPL_H_


#include "error.h"
#include "common.h"


static void solveL1Stripe_2 (const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip);
template<unsigned int d_stride>
void scaleAndFactorizeL1Stripe_2(dReal *ARow, dReal *d, unsigned rowIndex, unsigned rowSkip);
template<unsigned int d_stride>
inline void scaleAndFactorizeL1FirstRowStripe_2(dReal *ARow, dReal *d, unsigned rowSkip);

static void solveStripeL1_1 (const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip);
template<unsigned int d_stride>
void scaleAndFactorizeL1Stripe_1(dReal *ARow, dReal *d, unsigned rowIndex);
template<unsigned int d_stride>
inline void scaleAndFactorizeL1FirstRowStripe_1(dReal *ARow, dReal *d);


template<unsigned int d_stride>
void factorMatrixAsLDLT(dReal *A, dReal *d, unsigned rowCount, unsigned rowSkip)
{
    if (rowCount < 1) return;

    dReal *ARow = A;
    unsigned blockStartRow = 0;

    const unsigned blockStep = 2;
    const unsigned lastRowIndex = rowCount >= blockStep ? rowCount - blockStep + 1 : 0;

    /* compute blocks of 2 rows */
    bool subsequentPass = false;
    for (; blockStartRow < lastRowIndex; subsequentPass = true, ARow += blockStep * rowSkip, blockStartRow += blockStep) 
    {
        if (subsequentPass)
        {
            /* solve L*(D*l)=a, l is scaled elements in 2 x i block at A(i,0) */
            solveL1Stripe_2(A, ARow, blockStartRow, rowSkip);
            scaleAndFactorizeL1Stripe_2<d_stride>(ARow, d, blockStartRow, rowSkip);
        }
        else
        {
            scaleAndFactorizeL1FirstRowStripe_2<d_stride>(ARow, d, rowSkip);
        }
        dSASSERT(blockStep == 2);
        /* done factorizing 2 x 2 block */
    }

    /* compute the (less than 2) rows at the bottom */
    if (!subsequentPass || blockStartRow == lastRowIndex)
    {
        dSASSERT(blockStep == 2); // for the blockStartRow == lastRowIndex comparison above

        if (subsequentPass)
        {
            solveStripeL1_1(A, ARow, blockStartRow, rowSkip);
            scaleAndFactorizeL1Stripe_1<d_stride>(ARow, d, blockStartRow);
        }
        else
        {
            scaleAndFactorizeL1FirstRowStripe_1<d_stride>(ARow, d);
        }
        dSASSERT(blockStep == 2);
        /* done factorizing 1 x 1 block */
    }
}

/* solve L*X=B, with B containing 2 right hand sides.
 * L is an n*n lower triangular matrix with ones on the diagonal.
 * L is stored by rows and its leading dimension is rowSkip.
 * B is an n*2 matrix that contains the right hand sides.
 * B is stored by columns and its leading dimension is also rowSkip.
 * B is overwritten with X.
 * this processes blocks of 2*2.
 * if this is in the factorizer source file, n must be a multiple of 2.
 */
static 
void solveL1Stripe_2(const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip)
{
    dIASSERT(rowCount != 0);
    dIASSERT(rowCount % 2 == 0);

    /* compute all 2 x 2 blocks of X */
    unsigned blockStartRow = 0;
    for (bool exitLoop = false, subsequentPass = false; !exitLoop; subsequentPass = true, exitLoop = (blockStartRow += 2) == rowCount) 
    {
        const dReal *ptrLElement;
        dReal *ptrBElement;

        /* declare variables - Z matrix */
        dReal Z11, Z12, Z21, Z22;

        /* compute all 2 x 2 block of X, from rows i..i+2-1 */
        if (subsequentPass)
        {
            ptrLElement = L + blockStartRow * rowSkip;
            ptrBElement = B;

            /* set Z matrix to 0 */
            Z11 = 0; Z12 = 0; Z21 = 0; Z22 = 0;

            /* the inner loop that computes outer products and adds them to Z */
            // The iteration starts with even number and decreases it by 2. So, it must end in zero
            for (unsigned columnCounter = blockStartRow; ;) 
            {
                /* declare p and q vectors, etc */
                dReal p1, q1, p2, q2;

                /* compute outer product and add it to the Z matrix */
                p1 = ptrLElement[0];
                q1 = ptrBElement[0];
                Z11 += p1 * q1;
                q2 = ptrBElement[rowSkip];
                Z12 += p1 * q2;
                p2 = ptrLElement[rowSkip];
                Z21 += p2 * q1;
                Z22 += p2 * q2;

                /* compute outer product and add it to the Z matrix */
                p1 = ptrLElement[1];
                q1 = ptrBElement[1];
                Z11 += p1 * q1;
                q2 = ptrBElement[1 + rowSkip];
                Z12 += p1 * q2;
                p2 = ptrLElement[1 + rowSkip];
                Z21 += p2 * q1;
                Z22 += p2 * q2;

                if (columnCounter > 6)
                {
                    columnCounter -= 6;

                    /* advance pointers */
                    ptrLElement += 6;
                    ptrBElement += 6;

                    /* compute outer product and add it to the Z matrix */
                    p1 = ptrLElement[-4];
                    q1 = ptrBElement[-4];
                    Z11 += p1 * q1;
                    q2 = ptrBElement[-4 + rowSkip];
                    Z12 += p1 * q2;
                    p2 = ptrLElement[-4 + rowSkip];
                    Z21 += p2 * q1;
                    Z22 += p2 * q2;

                    /* compute outer product and add it to the Z matrix */
                    p1 = ptrLElement[-3];
                    q1 = ptrBElement[-3];
                    Z11 += p1 * q1;
                    q2 = ptrBElement[-3 + rowSkip];
                    Z12 += p1 * q2;
                    p2 = ptrLElement[-3 + rowSkip];
                    Z21 += p2 * q1;
                    Z22 += p2 * q2;

                    /* compute outer product and add it to the Z matrix */
                    p1 = ptrLElement[-2];
                    q1 = ptrBElement[-2];
                    Z11 += p1 * q1;
                    q2 = ptrBElement[-2 + rowSkip];
                    Z12 += p1 * q2;
                    p2 = ptrLElement[-2 + rowSkip];
                    Z21 += p2 * q1;
                    Z22 += p2 * q2;

                    /* compute outer product and add it to the Z matrix */
                    p1 = ptrLElement[-1];
                    q1 = ptrBElement[-1];
                    Z11 += p1 * q1;
                    q2 = ptrBElement[-1 + rowSkip];
                    Z12 += p1 * q2;
                    p2 = ptrLElement[-1 + rowSkip];
                    Z21 += p2 * q1;
                    Z22 += p2 * q2;
                }
                else
                {
                    /* advance pointers */
                    ptrLElement += 2;
                    ptrBElement += 2;

                    if ((columnCounter -= 2) == 0)
                    {
                        break;
                    }
                }
                /* end of inner loop */
            }
        }
        else
        {
            ptrLElement = L/* + blockStartRow * rowSkip*/; dIASSERT(blockStartRow == 0);
            ptrBElement = B;

            /* set Z matrix to 0 */
            Z11 = 0; Z12 = 0; Z21 = 0; Z22 = 0;
        }

        /* finish computing the X(i) block */
        
        dReal Y11 = ptrBElement[0] - Z11;
        dReal Y12 = ptrBElement[rowSkip] - Z12;

        dReal p2 = ptrLElement[rowSkip];

        ptrBElement[0] = Y11;
        ptrBElement[rowSkip] = Y12;

        dReal Y21 = ptrBElement[1] - Z21 - p2 * Y11;
        dReal Y22 = ptrBElement[1 + rowSkip] - Z22 - p2 * Y12;

        ptrBElement[1] = Y21;
        ptrBElement[1 + rowSkip] = Y22;
        /* end of outer loop */
    }
}

template<unsigned int d_stride>
void scaleAndFactorizeL1Stripe_2(dReal *ARow, dReal *d, unsigned factorizationRow, unsigned rowSkip)
{
    dIASSERT(factorizationRow != 0);
    dIASSERT(factorizationRow % 2 == 0);

    dReal *ptrAElement = ARow;
    dReal *ptrDElement = d;

    /* scale the elements in a 2 x i block at A(i,0), and also */
    /* compute Z = the outer product matrix that we'll need. */
    dReal Z11 = 0, Z21 = 0, Z22 = 0;

    for (unsigned columnCounter = factorizationRow; ; ) 
    {
        dReal p1, q1, p2, q2, dd;

        p1 = ptrAElement[0];
        p2 = ptrAElement[rowSkip];
        dd = ptrDElement[0 * d_stride];
        q1 = p1 * dd;
        q2 = p2 * dd;
        ptrAElement[0] = q1;
        ptrAElement[rowSkip] = q2;
        Z11 += p1 * q1;
        Z21 += p2 * q1;
        Z22 += p2 * q2;

        p1 = ptrAElement[1];
        p2 = ptrAElement[1 + rowSkip];
        dd = ptrDElement[1 * d_stride];
        q1 = p1 * dd;
        q2 = p2 * dd;
        ptrAElement[1] = q1;
        ptrAElement[1 + rowSkip] = q2;
        Z11 += p1 * q1;
        Z21 += p2 * q1;
        Z22 += p2 * q2;

        if (columnCounter > 6)
        {
            columnCounter -= 6;

            ptrAElement += 6;
            ptrDElement += 6 * d_stride;

            p1 = ptrAElement[-4];
            p2 = ptrAElement[-4 + rowSkip];
            dd = ptrDElement[-4 * (int)d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[-4] = q1;
            ptrAElement[-4 + rowSkip] = q2;
            Z11 += p1 * q1;
            Z21 += p2 * q1;
            Z22 += p2 * q2;

            p1 = ptrAElement[-3];
            p2 = ptrAElement[-3 + rowSkip];
            dd = ptrDElement[-3 * (int)d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[-3] = q1;
            ptrAElement[-3 + rowSkip] = q2;
            Z11 += p1 * q1;
            Z21 += p2 * q1;
            Z22 += p2 * q2;

            p1 = ptrAElement[-2];
            p2 = ptrAElement[-2 + rowSkip];
            dd = ptrDElement[-2 * (int)d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[-2] = q1;
            ptrAElement[-2 + rowSkip] = q2;
            Z11 += p1 * q1;
            Z21 += p2 * q1;
            Z22 += p2 * q2;

            p1 = ptrAElement[-1];
            p2 = ptrAElement[-1 + rowSkip];
            dd = ptrDElement[-1 * (int)d_stride];
            q1 = p1 * dd;
            q2 = p2 * dd;
            ptrAElement[-1] = q1;
            ptrAElement[-1 + rowSkip] = q2;
            Z11 += p1 * q1;
            Z21 += p2 * q1;
            Z22 += p2 * q2;
        }
        else
        {
            ptrAElement += 2;
            ptrDElement += 2 * d_stride;

            if ((columnCounter -= 2) == 0)
            {
                break;
            }
        }
    }

    /* solve for diagonal 2 x 2 block at A(i,i) */
    dReal Y11 = ptrAElement[0] - Z11;
    dReal Y21 = ptrAElement[rowSkip] - Z21;
    dReal Y22 = ptrAElement[1 + rowSkip] - Z22;

    /* factorize 2 x 2 block Y, ptrDElement */
    /* factorize row 1 */
    dReal dd = dRecip(Y11);

    ptrDElement[0 * d_stride] = dd;
    dIASSERT(ptrDElement == d + (sizeint)factorizationRow * d_stride);

    /* factorize row 2 */
    dReal q2 = Y21 * dd;
    ptrAElement[rowSkip] = q2;

    dReal sum = Y21 * q2;
    ptrDElement[1 * d_stride] = dRecip(Y22 - sum);
}

template<unsigned int d_stride>
void scaleAndFactorizeL1FirstRowStripe_2(dReal *ARow, dReal *d, unsigned rowSkip)
{
    dReal *ptrAElement = ARow;
    dReal *ptrDElement = d;

    /* solve for diagonal 2 x 2 block at A(0,0) */
    dReal Y11 = ptrAElement[0]/* - Z11*/;
    dReal Y21 = ptrAElement[rowSkip]/* - Z21*/;
    dReal Y22 = ptrAElement[1 + rowSkip]/* - Z22*/;

    /* factorize 2 x 2 block Y, ptrDElement */
    /* factorize row 1 */
    dReal dd = dRecip(Y11);

    ptrDElement[0 * d_stride] = dd;
    dIASSERT(ptrDElement == d/* + (sizeint)factorizationRow * d_stride*/);

    /* factorize row 2 */
    dReal q2 = Y21 * dd;
    ptrAElement[rowSkip] = q2;

    dReal sum = Y21 * q2;
    ptrDElement[1 * d_stride] = dRecip(Y22 - sum);
}


/* solve L*X=B, with B containing 1 right hand sides.
 * L is an n*n lower triangular matrix with ones on the diagonal.
 * L is stored by rows and its leading dimension is lskip.
 * B is an n*1 matrix that contains the right hand sides.
 * B is stored by columns and its leading dimension is also lskip.
 * B is overwritten with X.
 * this processes blocks of 2*2.
 * if this is in the factorizer source file, n must be a multiple of 2.
 */
static 
void solveStripeL1_1(const dReal *L, dReal *B, unsigned rowCount, unsigned rowSkip)
{
    dIASSERT(rowCount != 0);
    dIASSERT(rowCount % 2 == 0);

    /* compute all 2 x 1 blocks of X */
    unsigned blockStartRow = 0;
    for (bool exitLoop = false, subsequentPass = false; !exitLoop; subsequentPass = true, exitLoop = (blockStartRow += 2) == rowCount) 
    {
        const dReal *ptrLElement;
        dReal *ptrBElement;

        /* declare variables - Z matrix */
        dReal Z11, Z21;

        if (subsequentPass)
        {
            ptrLElement = L + (sizeint)blockStartRow * rowSkip;
            ptrBElement = B;

            /* set the Z matrix to 0 */
            Z11 = 0; Z21 = 0;

            /* compute all 2 x 1 block of X, from rows i..i+2-1 */
            
            /* the inner loop that computes outer products and adds them to Z */
            // The iteration starts with even number and decreases it by 2. So, it must end in zero
            for (unsigned columnCounter = blockStartRow; ; ) 
            {
                /* declare p and q vectors, etc */
                dReal p1, q1, p2;

                /* compute outer product and add it to the Z matrix */
                p1 = ptrLElement[0];
                q1 = ptrBElement[0];
                Z11 += p1 * q1;
                p2 = ptrLElement[rowSkip];
                Z21 += p2 * q1;
                
                /* compute outer product and add it to the Z matrix */
                p1 = ptrLElement[1];
                q1 = ptrBElement[1];
                Z11 += p1 * q1;
                p2 = ptrLElement[1 + rowSkip];
                Z21 += p2 * q1;

                if (columnCounter > 6)
                {
                    columnCounter -= 6;

                    /* advance pointers */
                    ptrLElement += 6;
                    ptrBElement += 6;

                    /* compute outer product and add it to the Z matrix */
                    p1 = ptrLElement[-4];
                    q1 = ptrBElement[-4];
                    Z11 += p1 * q1;
                    p2 = ptrLElement[-4 + rowSkip];
                    Z21 += p2 * q1;

                    /* compute outer product and add it to the Z matrix */
                    p1 = ptrLElement[-3];
                    q1 = ptrBElement[-3];
                    Z11 += p1 * q1;
                    p2 = ptrLElement[-3 + rowSkip];
                    Z21 += p2 * q1;

                    /* compute outer product and add it to the Z matrix */
                    p1 = ptrLElement[-2];
                    q1 = ptrBElement[-2];
                    Z11 += p1 * q1;
                    p2 = ptrLElement[-2 + rowSkip];
                    Z21 += p2 * q1;

                    /* compute outer product and add it to the Z matrix */
                    p1 = ptrLElement[-1];
                    q1 = ptrBElement[-1];
                    Z11 += p1 * q1;
                    p2 = ptrLElement[-1 + rowSkip];
                    Z21 += p2 * q1;
                }
                else
                {
                    /* advance pointers */
                    ptrLElement += 2;
                    ptrBElement += 2;

                    if ((columnCounter -= 2) == 0)
                    {
                        break;
                    }
                }
                /* end of inner loop */
            }
        }
        else
        {
            ptrLElement = L/* + (sizeint)blockStartRow * rowSkip*/; dIASSERT(blockStartRow == 0);
            ptrBElement = B;

            /* set the Z matrix to 0 */
            Z11 = 0; Z21 = 0;
        }
        
        /* finish computing the X(i) block */
        dReal p2 = ptrLElement[rowSkip];

        dReal Y11 = ptrBElement[0] - Z11;
        dReal Y21 = ptrBElement[1] - Z21 - p2 * Y11;

        ptrBElement[0] = Y11;
        ptrBElement[1] = Y21;
        /* end of outer loop */
    }
}

template<unsigned int d_stride>
void scaleAndFactorizeL1Stripe_1(dReal *ARow, dReal *d, unsigned factorizationRow)
{
    dReal *ptrAElement = ARow;
    dReal *ptrDElement = d;

    /* scale the elements in a 1 x i block at A(i,0), and also */
    /* compute Z = the outer product matrix that we'll need. */
    dReal Z11 = 0, Z22 = 0;

    for (unsigned columnCounter = factorizationRow; ; ) 
    {
        dReal p1, p2, q1, q2, dd1, dd2;

        p1 = ptrAElement[0];
        p2 = ptrAElement[1];
        dd1 = ptrDElement[0 * d_stride];
        dd2 = ptrDElement[1 * d_stride];
        q1 = p1 * dd1;
        q2 = p2 * dd2;
        ptrAElement[0] = q1;
        ptrAElement[1] = q2;
        Z11 += p1 * q1;
        Z22 += p2 * q2;

        if (columnCounter > 6)
        {
            columnCounter -= 6;

            ptrAElement += 6;
            ptrDElement += 6 * d_stride;

            p1 = ptrAElement[-4];
            p2 = ptrAElement[-3];
            dd1 = ptrDElement[-4 * (int)d_stride];
            dd2 = ptrDElement[-3 * (int)d_stride];
            q1 = p1 * dd1;
            q2 = p2 * dd2;
            ptrAElement[-4] = q1;
            ptrAElement[-3] = q2;
            Z11 += p1 * q1;
            Z22 += p2 * q2;

            p1 = ptrAElement[-2];
            p2 = ptrAElement[-1];
            dd1 = ptrDElement[-2 * (int)d_stride];
            dd2 = ptrDElement[-1 * (int)d_stride];
            q1 = p1 * dd1;
            q2 = p2 * dd2;
            ptrAElement[-2] = q1;
            ptrAElement[-1] = q2;
            Z11 += p1 * q1;
            Z22 += p2 * q2;
        }
        else
        {
            ptrAElement += 2;
            ptrDElement += 2 * d_stride;

            if ((columnCounter -= 2) == 0)
            {
                break;
            }
        }
    }

    dReal Y11 = ptrAElement[0] - (Z11 + Z22);

    /* solve for diagonal 1 x 1 block at A(i,i) */
    dIASSERT(ptrDElement == d + (sizeint)factorizationRow * d_stride);
    /* factorize 1 x 1 block Y, ptrDElement */
    /* factorize row 1 */
    ptrDElement[0 * d_stride] = dRecip(Y11);
}

template<unsigned int d_stride>
void scaleAndFactorizeL1FirstRowStripe_1(dReal *ARow, dReal *d)
{
    dReal *ptrAElement = ARow;
    dReal *ptrDElement = d;

    dReal Y11 = ptrAElement[0];

    /* solve for diagonal 1 x 1 block at A(0,0) */
    /* factorize 1 x 1 block Y, ptrDElement */
    /* factorize row 1 */
    ptrDElement[0 * d_stride] = dRecip(Y11);
}




template<unsigned int block_step, unsigned int b_rows>
/*static */
void ThreadedEquationSolverLDLT::participateSolvingL1Stripe_X(const dReal *L, dReal *B, unsigned blockCount, unsigned rowSkip, 
    volatile atomicord32 &refBlockCompletionProgress/*=0*/, volatile cellindexint *blockProgressDescriptors/*=[blockCount]*/, 
    FactorizationSolveL1StripeCellContext *cellContexts/*=[CCI__MAX x blockCount] + [blockCount]*/, unsigned ownThreadIndex)
{
    const unsigned lookaheadRange = 64;
    BlockProcessingState blockProcessingState = BPS_NO_BLOCKS_PROCESSED;

    unsigned completedBlocks = refBlockCompletionProgress;
    unsigned currentBlock = completedBlocks;
    dIASSERT(completedBlocks <= blockCount);

    for (bool exitLoop = completedBlocks == blockCount; !exitLoop; exitLoop = false)
    {
        bool goForLockedBlockPrimaryCalculation = false, goForLockedBlockDuplicateCalculation = false;
        bool goAssigningTheResult = false, stayWithinTheBlock = false;

        dReal Z[block_step][b_rows];
        dReal Y[block_step][b_rows];

        dReal *ptrBElement;

        CellContextInstance previousContextInstance;
        unsigned completedColumnBlock;

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

                        const FactorizationSolveL1StripeCellContext &sourceContext = buildBlockContextRef(cellContexts, currentBlock, contextInstance);
                        sourceContext.loadPrecalculatedZs(Z);
                    }
                    else
                    {
                        previousContextInstance = CCI__MIN;
                        FactorizationSolveL1StripeCellContext::initializePrecalculatedZs(Z);
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
                    const FactorizationSolveL1StripeCellContext &sourceContext = buildBlockContextRef(cellContexts, currentBlock, contextInstance);
                    sourceContext.loadPrecalculatedZs(Z);
                }
                else
                {
                    FactorizationSolveL1StripeCellContext::initializePrecalculatedZs(Z);
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

            if (currentBlock != 0)
            {
                /* compute all 2 x 2 block of X, from rows i..i+2-1 */
                ptrLElement = L + (currentBlock * rowSkip + completedColumnBlock) * block_step;
                ptrBElement = B + completedColumnBlock * block_step;

                /* the inner loop that computes outer products and adds them to Z */
                finalColumnBlock = dMACRO_MIN(currentBlock, completedBlocks);
                dIASSERT(completedColumnBlock != finalColumnBlock/* || currentBlock == 0*/);

                // The iteration starts with even number and decreases it by 2. So, it must end in zero
                for (unsigned columnCounter = finalColumnBlock - completedColumnBlock; ; )
                {
                    /* declare p and q vectors, etc */
                    dReal p[block_step], q[b_rows];

                    /* compute outer product and add it to the Z matrix */
                    p[0] = ptrLElement[0];
                    q[0] = ptrBElement[0];
                    Z[0][0] += p[0] * q[0];
                    if (b_rows >= 2)
                    {
                        q[1] = ptrBElement[rowSkip];
                        Z[0][1] += p[0] * q[1];
                    }
                    p[1] = ptrLElement[rowSkip];
                    Z[1][0] += p[1] * q[0];
                    if (b_rows >= 2)
                    {
                        Z[1][1] += p[1] * q[1];
                    }

                    /* compute outer product and add it to the Z matrix */
                    p[0] = ptrLElement[1];
                    q[0] = ptrBElement[1];
                    Z[0][0] += p[0] * q[0];
                    if (b_rows >= 2)
                    {
                        q[1] = ptrBElement[1 + rowSkip];
                        Z[0][1] += p[0] * q[1];
                    }
                    p[1] = ptrLElement[1 + rowSkip];
                    Z[1][0] += p[1] * q[0];
                    if (b_rows >= 2)
                    {
                        Z[1][1] += p[1] * q[1];
                    }
                    
                    dSASSERT(block_step == 2);
                    dSASSERT(b_rows >= 1 && b_rows <= 2);

                    if (columnCounter > 2)
                    {
                        /* compute outer product and add it to the Z matrix */
                        p[0] = ptrLElement[2];
                        q[0] = ptrBElement[2];
                        Z[0][0] += p[0] * q[0];
                        if (b_rows >= 2)
                        {
                            q[1] = ptrBElement[2 + rowSkip];
                            Z[0][1] += p[0] * q[1];
                        }
                        p[1] = ptrLElement[2 + rowSkip];
                        Z[1][0] += p[1] * q[0];
                        if (b_rows >= 2)
                        {
                            Z[1][1] += p[1] * q[1];
                        }

                        /* compute outer product and add it to the Z matrix */
                        p[0] = ptrLElement[3];
                        q[0] = ptrBElement[3];
                        Z[0][0] += p[0] * q[0];
                        if (b_rows >= 2)
                        {
                            q[1] = ptrBElement[3 + rowSkip];
                            Z[0][1] += p[0] * q[1];
                        }
                        p[1] = ptrLElement[3 + rowSkip];
                        Z[1][0] += p[1] * q[0];
                        if (b_rows >= 2)
                        {
                            Z[1][1] += p[1] * q[1];
                        }

                        dSASSERT(block_step == 2);
                        dSASSERT(b_rows >= 1 && b_rows <= 2);

                        /* advance pointers */
                        ptrLElement += 2 * block_step;
                        ptrBElement += 2 * block_step;
                        columnCounter -= 2;
                    }
                    else
                    {
                        /* advance pointers */
                        ptrLElement += block_step;
                        ptrBElement += block_step;
                        /* end of inner loop */

                        if (--columnCounter == 0)
                        {
                            if (finalColumnBlock == currentBlock)
                            {
                                rowEndReached = true;
                                break;
                            }

                            // Take a look if any more rows have been completed...
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
                }
            }
            else
            {
                ptrLElement = L/* + (currentBlock * rowSkip + completedColumnBlock) * block_step*/;
                ptrBElement = B/* + completedColumnBlock * block_step*/;

                rowEndReached = true;
            }

            if (rowEndReached)
            {
                // Check whether there is still a need to proceed or if the computation has been taken over by another thread
                cellindexint oldDescriptor = MAKE_CELLDESCRIPTOR(completedColumnBlock, previousContextInstance, true);
                
                if (blockProgressDescriptors[currentBlock] == oldDescriptor)
                {
                    /* finish computing the X(i) block */
                    Y[0][0] = ptrBElement[0] - Z[0][0];
                    if (b_rows >= 2)
                    {
                        Y[0][1] = ptrBElement[rowSkip] - Z[0][1];
                    }

                    dReal p2 = ptrLElement[rowSkip];

                    Y[1][0] = ptrBElement[1] - Z[1][0] - p2 * Y[0][0];
                    if (b_rows >= 2)
                    {
                        Y[1][1] = ptrBElement[1 + rowSkip] - Z[1][1] - p2 * Y[0][1];
                    }

                    dSASSERT(block_step == 2);
                    dSASSERT(b_rows >= 1 && b_rows <= 2);

                    // Use atomic memory barrier to make sure memory reads of ptrBElement[] and blockProgressDescriptors[] are not swapped
                    CooperativeAtomics::AtomicReadReorderBarrier();
                    
                    // The descriptor has not been altered yet - this means the ptrBElement[] values used above were not modified yet 
                    // and the computation result is valid.
                    if (blockProgressDescriptors[currentBlock] == oldDescriptor)
                    {
                        // Assign the results to the result context (possibly in parallel with other threads 
                        // that could and ought to be assigning exactly the same values)
                        FactorizationSolveL1StripeCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
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
                FactorizationSolveL1StripeCellContext &destinationContext = buildBlockContextRef(cellContexts, currentBlock, nextContextInstance);
                destinationContext.storePrecalculatedZs(Z);

                // Unlock the row until more columns can be used
                cellindexint oldDescriptor = MAKE_CELLDESCRIPTOR(completedColumnBlock, previousContextInstance, true);
                cellindexint newDescriptor = MAKE_CELLDESCRIPTOR(finalColumnBlock, nextContextInstance, false);
                // The descriptor might have been updated by a competing thread
                if (!CooperativeAtomics::AtomicCompareExchangeCellindexint(&blockProgressDescriptors[currentBlock], oldDescriptor, newDescriptor))
                {
                    // Adjust the ptrBElement to point to the result area...
                    ptrBElement = B + currentBlock * block_step;
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
                    const FactorizationSolveL1StripeCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
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
                /* compute all 2 x 2 block of X, from rows i..i+2-1 */
                ptrLElement = L + (currentBlock * rowSkip + completedColumnBlock) * block_step;
                ptrBElement = B + completedColumnBlock * block_step;

                /* the inner loop that computes outer products and adds them to Z */
                // The iteration starts with even number and decreases it by 2. So, it must end in zero
                const unsigned finalColumnBlock = currentBlock;
                dIASSERT(currentBlock == completedBlocks); // Why would we be competing for a row otherwise?

                unsigned lastCompletedColumn = completedColumnBlock;
                unsigned columnCounter = finalColumnBlock - completedColumnBlock;
                for (bool exitInnerLoop = false; !exitInnerLoop; exitInnerLoop = --columnCounter == 0)
                {
                    /* declare p and q vectors, etc */
                    dReal p[block_step], q[b_rows];

                    /* compute outer product and add it to the Z matrix */
                    p[0] = ptrLElement[0];
                    q[0] = ptrBElement[0];
                    Z[0][0] += p[0] * q[0];
                    if (b_rows >= 2)
                    {
                        q[1] = ptrBElement[rowSkip];
                        Z[0][1] += p[0] * q[1];
                    }
                    p[1] = ptrLElement[rowSkip];
                    Z[1][0] += p[1] * q[0];
                    if (b_rows >= 2)
                    {
                        Z[1][1] += p[1] * q[1];
                    }

                    /* compute outer product and add it to the Z matrix */
                    p[0] = ptrLElement[1];
                    q[0] = ptrBElement[1];
                    Z[0][0] += p[0] * q[0];
                    if (b_rows >= 2)
                    {
                        q[1] = ptrBElement[1 + rowSkip];
                        Z[0][1] += p[0] * q[1];
                    }
                    p[1] = ptrLElement[1 + rowSkip];
                    Z[1][0] += p[1] * q[0];
                    if (b_rows >= 2)
                    {
                        Z[1][1] += p[1] * q[1];
                    }

                    dSASSERT(block_step == 2);
                    dSASSERT(b_rows >= 1 && b_rows <= 2);

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
                    ptrBElement += block_step;
                    /* end of inner loop */
                }
            }
            else if (completedColumnBlock > currentBlock)
            {
                dIASSERT(completedColumnBlock == currentBlock + 1);

                skiptoCopyingResult = true;
            }
            else
            {
                dIASSERT(currentBlock == 0); // Execution can get here within the very first block only

                /* assign the pointers appropriately and go on computing the results */
                ptrLElement = L/* + (currentBlock * rowSkip + completedColumnBlock) * block_step*/;
                ptrBElement = B/* + completedColumnBlock * block_step*/;
            }

            if (!skipToHandlingSubsequentRows)
            {
                if (!skiptoCopyingResult)
                {
                    /* finish computing the X(i) block */
                    Y[0][0] = ptrBElement[0] - Z[0][0];
                    if (b_rows >= 2)
                    {
                        Y[0][1] = ptrBElement[rowSkip] - Z[0][1];
                    }

                    dReal p2 = ptrLElement[rowSkip];

                    Y[1][0] = ptrBElement[1] - Z[1][0] - p2 * Y[0][0];
                    if (b_rows >= 2)
                    {
                        Y[1][1] = ptrBElement[1 + rowSkip] - Z[1][1] - p2 * Y[0][1];
                    }

                    dSASSERT(block_step == 2);
                    dSASSERT(b_rows >= 1 && b_rows <= 2);

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
                        FactorizationSolveL1StripeCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
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
                        const FactorizationSolveL1StripeCellContext &resultContext = buildResultContextRef(cellContexts, currentBlock, blockCount);
                        resultContext.loadPrecalculatedZs(Y);

                        ptrBElement = B + currentBlock * block_step;
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
                // Assign the computation results to their places in the matrix
                ptrBElement[0] = Y[0][0];
                ptrBElement[1] = Y[1][0];
                if (b_rows >= 2)
                {
                    ptrBElement[rowSkip] = Y[0][1];
                    ptrBElement[1 + rowSkip] = Y[1][1];
                }

                dSASSERT(block_step == 2);
                dSASSERT(b_rows >= 1 && b_rows <= 2);

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


template<unsigned int a_rows, unsigned int d_stride>
/*static */
void ThreadedEquationSolverLDLT::participateScalingAndFactorizingL1Stripe_X(dReal *ARow, dReal *d, unsigned factorizationRow, unsigned rowSkip,
    FactorizationFactorizeL1StripeContext *factorizationContext, unsigned ownThreadIndex)
{
    dIASSERT(factorizationRow != 0);
    dIASSERT(factorizationRow % 2 == 0);

    /* scale the elements in a 2 x i block at A(i,0), and also */
    /* compute Z = the outer product matrix that we'll need. */
    dReal sameZ[a_rows] = { REAL(0.0), }, mixedZ[dMACRO_MAX(a_rows - 1, 1)] = { REAL(0.0), };
    bool doneAnything = false;

    const unsigned blockSize = deriveScalingAndFactorizingL1StripeBlockSize(a_rows);

    const unsigned blockCount = deriveScalingAndFactorizingL1StripeBlockCountFromFactorizationRow(factorizationRow, blockSize);
    dIASSERT(blockCount != 0);

    unsigned blockIndex;
    while ((blockIndex = ThrsafeIncrementIntUpToLimit(&factorizationContext->m_nextColumnIndex, blockCount)) != blockCount) 
    {
        doneAnything = true;
        unsigned blockStartRow = blockIndex * blockSize;

        dReal *ptrAElement = ARow + blockStartRow;
        dReal *ptrDElement = d + blockStartRow * d_stride;
        for (unsigned columnCounter = blockIndex != blockCount - 1 ? blockSize : factorizationRow - blockStartRow; ; )
        {
            dReal p1, q1, p2, q2, dd;

            p1 = ptrAElement[0];
            if (a_rows >= 2)
            {
                p2 = ptrAElement[rowSkip];
            }
            dd = ptrDElement[0 * d_stride];
            q1 = p1 * dd;
            if (a_rows >= 2)
            {
                q2 = p2 * dd;
            }
            ptrAElement[0] = q1;
            if (a_rows >= 2)
            {
                ptrAElement[rowSkip] = q2;
            }
            sameZ[0] += p1 * q1;
            if (a_rows >= 2)
            {
                sameZ[1] += p2 * q2;
                mixedZ[0] += p2 * q1;
            }

            p1 = ptrAElement[1];
            if (a_rows >= 2)
            {
                p2 = ptrAElement[1 + rowSkip];
            }
            dd = ptrDElement[1 * d_stride];
            q1 = p1 * dd;
            if (a_rows >= 2)
            {
                q2 = p2 * dd;
            }
            ptrAElement[1] = q1;
            if (a_rows >= 2)
            {
                ptrAElement[1 + rowSkip] = q2;
            }
            sameZ[0] += p1 * q1;
            if (a_rows >= 2)
            {
                sameZ[1] += p2 * q2;
                mixedZ[0] += p2 * q1;
            }

            if (columnCounter > 6)
            {
                columnCounter -= 6;

                ptrAElement += 6;
                ptrDElement += 6 * d_stride;

                p1 = ptrAElement[-4];
                if (a_rows >= 2)
                {
                    p2 = ptrAElement[-4 + rowSkip];
                }
                dd = ptrDElement[-4 * (int)d_stride];
                q1 = p1 * dd;
                if (a_rows >= 2)
                {
                    q2 = p2 * dd;
                }
                ptrAElement[-4] = q1;
                if (a_rows >= 2)
                {
                    ptrAElement[-4 + rowSkip] = q2;
                }
                sameZ[0] += p1 * q1;
                if (a_rows >= 2)
                {
                    sameZ[1] += p2 * q2;
                    mixedZ[0] += p2 * q1;
                }

                p1 = ptrAElement[-3];
                if (a_rows >= 2)
                {
                    p2 = ptrAElement[-3 + rowSkip];
                }
                dd = ptrDElement[-3 * (int)d_stride];
                q1 = p1 * dd;
                if (a_rows >= 2)
                {
                    q2 = p2 * dd;
                }
                ptrAElement[-3] = q1;
                if (a_rows >= 2)
                {
                    ptrAElement[-3 + rowSkip] = q2;
                }
                sameZ[0] += p1 * q1;
                if (a_rows >= 2)
                {
                    sameZ[1] += p2 * q2;
                    mixedZ[0] += p2 * q1;
                }

                p1 = ptrAElement[-2];
                if (a_rows >= 2)
                {
                    p2 = ptrAElement[-2 + rowSkip];
                }
                dd = ptrDElement[-2 * (int)d_stride];
                q1 = p1 * dd;
                if (a_rows >= 2)
                {
                    q2 = p2 * dd;
                }
                ptrAElement[-2] = q1;
                if (a_rows >= 2)
                {
                    ptrAElement[-2 + rowSkip] = q2;
                }
                sameZ[0] += p1 * q1;
                if (a_rows >= 2)
                {
                    sameZ[1] += p2 * q2;
                    mixedZ[0] += p2 * q1;
                }

                p1 = ptrAElement[-1];
                if (a_rows >= 2)
                {
                    p2 = ptrAElement[-1 + rowSkip];
                }
                dd = ptrDElement[-1 * (int)d_stride];
                q1 = p1 * dd;
                if (a_rows >= 2)
                {
                    q2 = p2 * dd;
                }
                ptrAElement[-1] = q1;
                if (a_rows >= 2)
                {
                    ptrAElement[-1 + rowSkip] = q2;
                }
                sameZ[0] += p1 * q1;
                if (a_rows >= 2)
                {
                    sameZ[1] += p2 * q2;
                    mixedZ[0] += p2 * q1;
                }
            }
            else
            {
                ptrAElement += 2;
                ptrDElement += 2 * d_stride;

                if ((columnCounter -= 2) == 0)
                {
                    break;
                }
            }
        }
    }

    if (doneAnything)
    {
        unsigned partialSumThreadIndex;
        for (bool exitLoop = false; !exitLoop; exitLoop = CooperativeAtomics::AtomicCompareExchangeUint32(&factorizationContext->m_sumThreadIndex, partialSumThreadIndex, ownThreadIndex + 1))
        {
            partialSumThreadIndex = factorizationContext->m_sumThreadIndex;
            
            if (partialSumThreadIndex != 0)
            {
                const FactorizationFactorizeL1StripeThreadContext &partialSumContext = factorizationContext->m_threadContexts[partialSumThreadIndex - 1];
                factorizationContext->m_threadContexts[ownThreadIndex].assignDataSum<a_rows>(sameZ, mixedZ, partialSumContext);
            }
            else
            {
                factorizationContext->m_threadContexts[ownThreadIndex].assignDataAlone<a_rows>(sameZ, mixedZ);
            }
        }
    }

    unsigned threadExitIndex = CooperativeAtomics::AtomicDecrementUint32(&factorizationContext->m_threadsRunning);
    dIASSERT(threadExitIndex + 1U != 0);

    if (threadExitIndex == 0)
    {
        // Let the last thread retrieve the sum and perform final computations
        unsigned sumThreadIndex = factorizationContext->m_sumThreadIndex;
        dIASSERT(sumThreadIndex != 0); // The rowIndex was asserted to be not zero, so at least one thread must have done something

        const FactorizationFactorizeL1StripeThreadContext &sumContext = factorizationContext->m_threadContexts[sumThreadIndex - 1];
        sumContext.retrieveData<a_rows>(sameZ, mixedZ);

        dReal *ptrAElement = ARow + factorizationRow;
        dReal *ptrDElement = d + factorizationRow * d_stride;

        /* solve for diagonal 2 x 2 block at A(i,i) */
        dReal Y11, Y21, Y22;
        
        Y11 = ptrAElement[0] - sameZ[0];
        if (a_rows >= 2)
        {
            Y21 = ptrAElement[rowSkip] - mixedZ[0];
            Y22 = ptrAElement[1 + rowSkip] - sameZ[1];
        }

        /* factorize 2 x 2 block Y, ptrDElement */
        /* factorize row 1 */
        dReal dd = dRecip(Y11);

        ptrDElement[0 * d_stride] = dd;
        dIASSERT(ptrDElement == d + (sizeint)factorizationRow * d_stride);

        if (a_rows >= 2)
        {
            /* factorize row 2 */
            dReal q2 = Y21 * dd;
            ptrAElement[rowSkip] = q2;

            dReal sum = Y21 * q2;
            ptrDElement[1 * d_stride] = dRecip(Y22 - sum);
        }
    }
}


#endif // #ifndef _ODE_FASTLDLT_IMPL_H_
