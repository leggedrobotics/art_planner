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

#ifndef _ODE_PRIVATE_COMMON_H_
#define _ODE_PRIVATE_COMMON_H_


#include "typedefs.h"
#include "error.h"
#include <ode/memory.h>
#include <algorithm>


#ifndef SIZE_MAX
#define SIZE_MAX  ((sizeint)(-1))
#endif

#define dMACRO_MAX(a, b) ((a) > (b) ? (a) : (b))
#define dMACRO_MIN(a, b) ((a) < (b) ? (a) : (b))


#ifdef dSINGLE
#define dEpsilon  FLT_EPSILON
#else
#define dEpsilon  DBL_EPSILON
#endif


#ifdef dSINGLE

#if !defined(FLT_MANT_DIG)
#define FLT_MANT_DIG 24
#endif

#define dMaxExact   ((float)((1UL << FLT_MANT_DIG) - 1))
#define dMinExact   ((float)(-dMaxExact))


#else // #ifndef dSINGLE

#if !defined(DBL_MANT_DIG)
#define DBL_MANT_DIG 53
#endif

#define dMaxExact   (double)((1ULL << DBL_MANT_DIG) - 1)
#define dMinExact   ((double)(-dMaxExact))


#endif // #ifndef dSINGLE


#define dMaxIntExact dMACRO_MIN(dMaxExact, (dReal)INT_MAX)
#define dMinIntExact dMACRO_MAX(dMinExact, (dReal)INT_MIN)


#ifndef offsetof
#define offsetof(s, m) ((sizeint)&(((s *)8)->m) - (sizeint)8)
#endif
#ifndef membersize
#define membersize(s, m) (sizeof(((s *)8)->m))
#endif
#ifndef endoffsetof
#define endoffsetof(s, m)   ((sizeint)((sizeint)&(((s *)8)->m) - (sizeint)8) + sizeof(((s *)8)->m))
#endif


/* the efficient alignment. most platforms align data structures to some
 * number of bytes, but this is not always the most efficient alignment.
 * for example, many x86 compilers align to 4 bytes, but on a pentium it
 * is important to align doubles to 8 byte boundaries (for speed), and
 * the 4 floats in a SIMD register to 16 byte boundaries. many other
 * platforms have similar behavior. setting a larger alignment can waste
 * a (very) small amount of memory. NOTE: this number must be a power of
 * two. this is set to 16 by default.
 */
#ifndef EFFICIENT_ALIGNMENT
#define EFFICIENT_ALIGNMENT 16
#endif

#define dALIGN_SIZE(buf_size, alignment) (((buf_size) + (alignment - 1)) & (int)(~(alignment - 1))) // Casting the mask to int ensures sign-extension to larger integer sizes
#define dALIGN_PTR(buf_ptr, alignment) ((void *)(((uintptr)(buf_ptr) + ((alignment) - 1)) & (int)(~(alignment - 1)))) // Casting the mask to int ensures sign-extension to larger integer sizes

/* round something up to be a multiple of the EFFICIENT_ALIGNMENT */
#define dEFFICIENT_SIZE(x) dALIGN_SIZE(x, EFFICIENT_ALIGNMENT)
#define dEFFICIENT_PTR(p) dALIGN_PTR(p, EFFICIENT_ALIGNMENT)
#define dOFFSET_EFFICIENTLY(p, b) ((void *)((uintptr)(p) + dEFFICIENT_SIZE(b)))

#define dOVERALIGNED_SIZE(size, alignment) dEFFICIENT_SIZE((size) + ((alignment) - EFFICIENT_ALIGNMENT))
#define dOVERALIGNED_PTR(buf_ptr, alignment) dALIGN_PTR(buf_ptr, alignment)
#define dOFFSET_OVERALIGNEDLY(buf_ptr, size, alignment) ((void *)((uintptr)(buf_ptr) + dOVERALIGNED_SIZE(size, alignment)))



#define dDERIVE_SIZE_UNION_PADDING_ELEMENTS(DataSize, ElementType) (((DataSize) + sizeof(ElementType) - 1) / sizeof(ElementType))
#define dDERIVE_TYPE_UNION_PADDING_ELEMENTS(DataType, ElementType) dDERIVE_SIZE_UNION_PADDING_ELEMENTS(sizeof(DataType), ElementType)
#define dDERIVE_SIZE_EXTRA_PADDING_ELEMENTS(DataSize, AlignmentSize, ElementType) (((dALIGN_SIZE(DataSize, dMACRO_MAX(AlignmentSize, sizeof(ElementType))) - (DataSize)) / sizeof(ElementType))



/* alloca aligned to the EFFICIENT_ALIGNMENT. note that this can waste
 * up to 15 bytes per allocation, depending on what alloca() returns.
 */
#define dALLOCA16(n) \
    dEFFICIENT_PTR(alloca((n)+(EFFICIENT_ALIGNMENT)))


class dxAlignedAllocation
{
public:
    dxAlignedAllocation(): m_userAreaPointer(NULL), m_bufferAllocated(NULL), m_sizeUsed(0) {}
    ~dxAlignedAllocation() { freeAllocation(); }

    void *allocAligned(sizeint sizeRequired, unsigned alignmentRequired)
    {
        dIASSERT((alignmentRequired & (alignmentRequired - 1)) == 0);
        dIASSERT(alignmentRequired <= SIZE_MAX - sizeRequired);

        sizeint sizeToUse = sizeRequired + alignmentRequired;
        void *bufferPointer = dAlloc(sizeToUse);
        void *userAreaPointer = bufferPointer != NULL && alignmentRequired != 0 ? dALIGN_PTR(bufferPointer, alignmentRequired) : bufferPointer;
        assignData(userAreaPointer, bufferPointer, sizeToUse);

        return userAreaPointer;
    }

    void *getUserAreaPointer() const { return m_userAreaPointer; }
    sizeint getUserAreaSize() const { return m_sizeUsed - ((uint8 *)m_userAreaPointer - (uint8 *)m_bufferAllocated); }

    void freeAllocation()
    {
        sizeint sizeUsed;
        void *bufferPointer = extractData(sizeUsed);
        
        if (bufferPointer != NULL)
        {
            dFree(bufferPointer, sizeUsed);
        }
    }

private:
    void assignData(void *userAreaPointer, void *bufferAllocated, sizeint sizeUsed)
    {
        dIASSERT(m_userAreaPointer == NULL);
        dIASSERT(m_bufferAllocated == NULL);
        dIASSERT(m_sizeUsed == 0);

        m_userAreaPointer = userAreaPointer;
        m_bufferAllocated = bufferAllocated;
        m_sizeUsed = sizeUsed;
    }

    void *extractData(sizeint &out_sizeUsed)
    {
        void *bufferPointer = m_bufferAllocated;

        if (bufferPointer != NULL)
        {
            out_sizeUsed = m_sizeUsed;

            m_userAreaPointer = NULL;
            m_bufferAllocated = NULL;
            m_sizeUsed = 0;
        }

        return bufferPointer;
    }

private:
    void *m_userAreaPointer;
    void *m_bufferAllocated;
    sizeint m_sizeUsed;
};


template<typename DstType, typename SrcType>
inline 
bool _cast_to_smaller(DstType &dtOutResult, const SrcType &stArgument)
{
    return (SrcType)(dtOutResult = (DstType)stArgument) == stArgument;
}

#if defined(__GNUC__)

#define dCAST_TO_SMALLER(TargetType, SourceValue) ({ TargetType ttCastSmallerValue; dIVERIFY(_cast_to_smaller(ttCastSmallerValue, SourceValue)); ttCastSmallerValue; })


#else // #if !defined(__GNUC__)

#define dCAST_TO_SMALLER(TargetType, SourceValue) templateCAST_TO_SMALLER<TargetType>(SourceValue)

template <typename TTargetType, typename TSourceType>
inline TTargetType templateCAST_TO_SMALLER(const TSourceType &stSourceValue)
{
    TTargetType ttCastSmallerValue;
    dIVERIFY(_cast_to_smaller(ttCastSmallerValue, stSourceValue));
    return ttCastSmallerValue;
}


#endif // #if !defined(__GNUC__)


template<typename value_type>
inline 
void dxSwap(value_type &one, value_type &another)
{
    std::swap(one, another);
}

template<typename value_type, typename lo_type, typename hi_type>
inline 
value_type dxClamp(const value_type &value, const lo_type &lo, const hi_type &hi)
{
    return value < lo ? (value_type)lo : value > hi ? (value_type)hi : value;
}


template <typename Type>
union _const_type_cast_union
{
    explicit _const_type_cast_union(const void *psvCharBuffer): m_psvCharBuffer(psvCharBuffer) {}

    operator const Type *() const { return m_pstTypedPointer; }
    const Type &operator *() const { return *m_pstTypedPointer; }
    const Type *operator ->() const { return m_pstTypedPointer; }
    const Type &operator [](diffint diElementIndex) const { return m_pstTypedPointer[diElementIndex]; }
    const Type &operator [](sizeint siElementIndex) const { return m_pstTypedPointer[siElementIndex]; }

    const void 		*m_psvCharBuffer;
    const Type		*m_pstTypedPointer;
};

template <typename Type>
union _type_cast_union
{
    explicit _type_cast_union(void *psvCharBuffer): m_psvCharBuffer(psvCharBuffer) {}

    operator Type *() const { return m_pstTypedPointer; }
    Type &operator *() const { return *m_pstTypedPointer; }
    Type *operator ->() const { return m_pstTypedPointer; }
    Type &operator [](diffint diElementIndex) const { return m_pstTypedPointer[diElementIndex]; }
    Type &operator [](sizeint siElementIndex) const { return m_pstTypedPointer[siElementIndex]; }

    void			*m_psvCharBuffer;
    Type			*m_pstTypedPointer;
};


template<sizeint tsiTypeSize>
struct _sized_signed;

template<>
struct _sized_signed<sizeof(uint8)>
{
    typedef int8 type;
};

template<>
struct _sized_signed<sizeof(uint16)>
{
    typedef int16 type;
};

template<>
struct _sized_signed<sizeof(uint32)>
{
    typedef int32 type;
};

template<>
struct _sized_signed<sizeof(uint64)>
{
    typedef int64 type;
};

template<typename tintergraltype>
struct _make_signed
{
    typedef typename _sized_signed<sizeof(tintergraltype)>::type type;
};


template<sizeint tsiTypeSize>
struct _sized_unsigned;

template<>
struct _sized_unsigned<sizeof(int8)>
{
    typedef uint8 type;
};

template<>
struct _sized_unsigned<sizeof(int16)>
{
    typedef uint16 type;
};

template<>
struct _sized_unsigned<sizeof(int32)>
{
    typedef uint32 type;
};

template<>
struct _sized_unsigned<sizeof(int64)>
{
    typedef uint64 type;
};

template<typename tintergraltype>
struct _make_unsigned
{
    typedef typename _sized_unsigned<sizeof(tintergraltype)>::type type;
};


// template<typename tvalueint, typename tminint, typename tmaxint>
// inline 
// bool dxInRange(tvalueint viValue, tminint miMin, tmaxint miMax)
// {
//     return (typename _sized_unsigned<dMACRO_MAX(sizeof(tvalueint), sizeof(tminint))>::type)(viValue - miMin) < (typename _sized_unsigned<dMACRO_MAX(sizeof(tmaxint), sizeof(tminint))>::type)(miMax - miMin);
// }
// #define dIN_RANGE(aval, amin, amax) dxInRange(aval, amin, amax)

#define dIN_RANGE(aval, amin, amax) ((_sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)((_sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(aval) - (_sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(amin)) < (_sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)((_sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amax) - (_sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amin)))
#define dTMPL_IN_RANGE(aval, amin, amax) ((typename _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)((typename _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(aval) - (typename _sized_unsigned<dMACRO_MAX(sizeof(aval), sizeof(amin))>::type)(amin)) < (typename _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)((typename _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amax) - (typename _sized_unsigned<dMACRO_MAX(sizeof(amax), sizeof(amin))>::type)(amin)))
#define dCLAMP(aval, alo, ahi) dxClamp(aval, alo, ahi)
#define dARRAY_SIZE(aarr) (sizeof(aarr) / sizeof((aarr)[0]))
#define dSTATIC_ARRAY_SIZE(aclass, aarr) dARRAY_SIZE(((aclass *)sizeof(void *))->aarr)


#endif
