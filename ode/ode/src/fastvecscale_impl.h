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
 * Vector scaling function implementation
 * Improvements and cooperative implementation copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")  
 */

#ifndef _ODE_FASTVECSCALE_IMPL_H_
#define _ODE_FASTVECSCALE_IMPL_H_



template<unsigned int a_stride, unsigned int d_stride>
void scaleLargeVector(dReal *aStart, const dReal *dStart, unsigned elementCount)
{
    dAASSERT (aStart && dStart && elementCount >= 0);
    
    const unsigned step = 4;

    dReal *ptrA = aStart;
    const dReal *ptrD = dStart;
    const dReal *const dStepsEnd = dStart + (sizeint)(elementCount & ~(step - 1)) * d_stride;
    for (; ptrD != dStepsEnd; ptrA += step * a_stride, ptrD += step * d_stride) 
    {
        dReal a0 = ptrA[0], a1 = ptrA[1 * a_stride], a2 = ptrA[2 * a_stride], a3 = ptrA[3 * a_stride];
        dReal d0 = ptrD[0], d1 = ptrD[1 * d_stride], d2 = ptrD[2 * d_stride], d3 = ptrD[3 * d_stride];
        a0 *= d0;
        a1 *= d1;
        a2 *= d2;
        a3 *= d3;
        ptrA[0] = a0; ptrA[1 * a_stride] = a1; ptrA[2 * a_stride] = a2; ptrA[3 * a_stride] = a3;
        dSASSERT(step == 4);
    }

    switch (elementCount & (step - 1))
    {
        case 3:
        {
            dReal a2 = ptrA[2 * a_stride];
            dReal d2 = ptrD[2 * d_stride];
            ptrA[2 * a_stride] = a2 * d2;
            // break; -- proceed to case 2
        }

        case 2:
        {
            dReal a1 = ptrA[1 * a_stride];
            dReal d1 = ptrD[1 * d_stride];
            ptrA[1 * a_stride] = a1 * d1;
            // break; -- proceed to case 1
        }

        case 1:
        {
            dReal a0 = ptrA[0];
            dReal d0 = ptrD[0];
            ptrA[0] = a0 * d0;
            break;
        }
    }
    dSASSERT(step == 4);
}


template<unsigned int block_step, unsigned int a_stride, unsigned int d_stride>
/*static */
void ThreadedEquationSolverLDLT::participateScalingVector(dReal *ptrAStart, const dReal *ptrDStart, const unsigned elementCount,
    volatile atomicord32 &refBlockCompletionProgress/*=0*/)
{
    dAASSERT (ptrAStart != NULL);
    dAASSERT(ptrDStart != NULL);
    dAASSERT(elementCount >= 0);

    const unsigned wrapSize = 4;
    dSASSERT(block_step % wrapSize == 0);

    const unsigned completeBlockCount = elementCount / block_step;
    const unsigned trailingBlockElements = elementCount % block_step;

    unsigned blockIndex;
    while ((blockIndex = ThrsafeIncrementIntUpToLimit(&refBlockCompletionProgress, completeBlockCount)) != completeBlockCount)
    {
        dReal *ptrAElement = ptrAStart + (sizeint)(blockIndex * block_step) * a_stride;
        const dReal *ptrDElement = ptrDStart + (sizeint)(blockIndex * block_step) * d_stride;
        const dReal *const ptrDBlockEnd = ptrDElement + block_step * d_stride;
        dSASSERT((sizeint)block_step * a_stride < UINT_MAX);
        dSASSERT((sizeint)block_step * d_stride < UINT_MAX);

        for (; ptrDElement != ptrDBlockEnd; ptrAElement += wrapSize * a_stride, ptrDElement += wrapSize * d_stride)
        {
            dReal a0 = ptrAElement[0], a1 = ptrAElement[1 * a_stride], a2 = ptrAElement[2 * a_stride], a3 = ptrAElement[3 * a_stride];
            dReal d0 = ptrDElement[0], d1 = ptrDElement[1 * d_stride], d2 = ptrDElement[2 * d_stride], d3 = ptrDElement[3 * d_stride];
            a0 *= d0;
            a1 *= d1;
            a2 *= d2;
            a3 *= d3;
            ptrAElement[0] = a0; ptrAElement[1 * a_stride] = a1; ptrAElement[2 * a_stride] = a2; ptrAElement[3 * a_stride] = a3;
            dSASSERT(wrapSize == 4);
        }
    }

    if (trailingBlockElements != 0 && (blockIndex = ThrsafeIncrementIntUpToLimit(&refBlockCompletionProgress, completeBlockCount + 1)) != completeBlockCount + 1)
    {
        dReal *ptrAElement = ptrAStart + (sizeint)(completeBlockCount * block_step) * a_stride;
        const dReal *ptrDElement = ptrDStart + (sizeint)(completeBlockCount * block_step) * d_stride;
        const dReal *const ptrDBlockEnd = ptrDElement + (trailingBlockElements & ~(wrapSize - 1)) * d_stride;

        for (; ptrDElement != ptrDBlockEnd; ptrAElement += wrapSize * a_stride, ptrDElement += wrapSize * d_stride)
        {
            dReal a0 = ptrAElement[0], a1 = ptrAElement[1 * a_stride], a2 = ptrAElement[2 * a_stride], a3 = ptrAElement[3 * a_stride];
            dReal d0 = ptrDElement[0], d1 = ptrDElement[1 * d_stride], d2 = ptrDElement[2 * d_stride], d3 = ptrDElement[3 * d_stride];
            a0 *= d0;
            a1 *= d1;
            a2 *= d2;
            a3 *= d3;
            ptrAElement[0] = a0; ptrAElement[1 * a_stride] = a1; ptrAElement[2 * a_stride] = a2; ptrAElement[3 * a_stride] = a3;
            dSASSERT(wrapSize == 4);
        }

        switch (trailingBlockElements & (wrapSize - 1))
        {
            case 3:
            {
                dReal a2 = ptrAElement[2 * a_stride];
                dReal d2 = ptrDElement[2 * d_stride];
                ptrAElement[2 * a_stride] = a2 * d2;
                // break; -- proceed to case 2
            }

            case 2:
            {
                dReal a1 = ptrAElement[1 * a_stride];
                dReal d1 = ptrDElement[1 * d_stride];
                ptrAElement[1 * a_stride] = a1 * d1;
                // break; -- proceed to case 1
            }

            case 1:
            {
                dReal a0 = ptrAElement[0];
                dReal d0 = ptrDElement[0];
                ptrAElement[0] = a0 * d0;
                break;
            }
        }
        dSASSERT(wrapSize == 4);
    }
}


#endif
