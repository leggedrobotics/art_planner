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

// Cooperative matrix algorithm types
// Copyright (C) 2017-2019 Oleh Derevenko (odar@eleks.com - change all "a" to "e")


#ifndef _ODE_COOP_MATRIX_TYPES_H_
#define _ODE_COOP_MATRIX_TYPES_H_



#include "threadingutils.h"
#include "common.h"
#include "error.h"


#ifndef dCOOPERATIVE_ENABLED

#if dATOMICS_ENABLED && !dTHREADING_INTF_DISABLED

#define dCOOPERATIVE_ENABLED 1


#endif // #if dATOMICS_ENABLED && !dTHREADING_INTF_DISABLED


#endif // #ifndef dCOOPERATIVE_ENABLED


enum
{
    COOP_THREAD_DATA_ALIGNMENT_SIZE      = 64, // Typical size of a cache line
};


typedef uintptr cellindexint;


enum CellContextInstance
{
    CCI__MIN,

    CCI_FIRST = CCI__MIN,
    CCI_SECOND,

    CCI__MAX,
    CCI__LOG2_OF_MAX = 1,

    CCI__DEFAULT = CCI__MIN,
};
dSASSERT(1 << CCI__LOG2_OF_MAX >= CCI__MAX);

static inline 
CellContextInstance buildNextContextInstance(CellContextInstance instance)
{
    dIASSERT(dIN_RANGE(instance, CCI__MIN, CCI__MAX));
    dSASSERT(CCI__MAX == 2);

    return (CellContextInstance)(CCI_FIRST + CCI_SECOND - instance);
}


enum
{
    CELLDESC_CCI_BITMASK    = (1 << CCI__LOG2_OF_MAX) - 1,
    CELLDESC_LOCK_BIT       = 1 << CCI__LOG2_OF_MAX,
    CELLDESC__HELPER_BITS   = CELLDESC_CCI_BITMASK | CELLDESC_LOCK_BIT,
    CELLDESC__COLINDEX_BASE = CELLDESC__HELPER_BITS + 1,
};

#define MAKE_CELLDESCRIPTOR(columnIndex, contextInstance, locked) ((cellindexint)((cellindexint)(columnIndex) * CELLDESC__COLINDEX_BASE + (contextInstance) + ((locked) ? CELLDESC_LOCK_BIT : 0)))
#define MARK_CELLDESCRIPTOR_LOCKED(descriptor) ((cellindexint)((descriptor) | CELLDESC_LOCK_BIT))
#define GET_CELLDESCRIPTOR_COLUMNINDEX(descriptor) ((unsigned int)((cellindexint)(descriptor) / CELLDESC__COLINDEX_BASE))
#define GET_CELLDESCRIPTOR_CONTEXTINSTANCE(descriptor) ((CellContextInstance)((descriptor) & CELLDESC_CCI_BITMASK))
#define GET_CELLDESCRIPTOR_ISLOCKED(descriptor) (((descriptor) & CELLDESC_LOCK_BIT) != 0)

#define INVALID_CELLDESCRIPTOR      MAKE_CELLDESCRIPTOR(GET_CELLDESCRIPTOR_COLUMNINDEX(-1), CCI__MAX - 1, true)


enum BlockProcessingState
{
    BPS_COMPETING_FOR_A_BLOCK = -1,
    BPS_NO_BLOCKS_PROCESSED,
    BPS_SOME_BLOCKS_PROCESSED,
};


class CooperativeAtomics
{
public:
    static atomicord32 AtomicDecrementUint32(volatile atomicord32 *paoDestination)
    {
#if dCOOPERATIVE_ENABLED
        return ::AtomicDecrement(paoDestination);
#else
        dIASSERT(false); return 0; // The function is not supposed to be called in this case
#endif // #if dCOOPERATIVE_ENABLED
    }

    static bool AtomicCompareExchangeUint32(volatile atomicord32 *paoDestination, atomicord32 aoComparand, atomicord32 aoExchange)
    {
#if dCOOPERATIVE_ENABLED
        return ::AtomicCompareExchange(paoDestination, aoComparand, aoExchange);
#else
        dIASSERT(false); return false; // The function is not supposed to be called in this case
#endif // #if dCOOPERATIVE_ENABLED
    }

    static bool AtomicCompareExchangeCellindexint(volatile cellindexint *destination, cellindexint comparand, cellindexint exchange)
    {
#if dCOOPERATIVE_ENABLED
        return ::AtomicCompareExchangePointer((volatile atomicptr *)destination, (atomicptr)comparand, (atomicptr)exchange);
#else
        dIASSERT(false); return false; // The function is not supposed to be called in this case
#endif // #if dCOOPERATIVE_ENABLED
    }

    static void AtomicStoreCellindexint(volatile cellindexint *destination, cellindexint value)
    {
#if dCOOPERATIVE_ENABLED
        ::AtomicStorePointer((volatile atomicptr *)destination, (atomicptr)value);
#else
        dIASSERT(false); // The function is not supposed to be called in this case
#endif // #if dCOOPERATIVE_ENABLED
    }

    static void AtomicReadReorderBarrier()
    {
#if dCOOPERATIVE_ENABLED
        ::AtomicReadReorderBarrier();
#else
        dIASSERT(false); // The function is not supposed to be called in this case
#endif // #if dCOOPERATIVE_ENABLED
    }
};


#endif // #ifndef _ODE_COOP_MATRIX_TYPES_H_
