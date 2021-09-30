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

#include <ode/common.h>
#include "config.h"
#include "matrix.h"
#include "objects.h"
#include "threaded_solver_ldlt.h"

#include <ode/memory.h>


// misc defines
#define ALLOCA dALLOCA16
#define STACK_ALLOC_MAX 8192U

/*extern */
void dxMultiply0(dReal *A, const dReal *B, const dReal *C, unsigned p, unsigned q, unsigned r)
{
    dAASSERT (A && B && C && p>0 && q>0 && r>0);
    const unsigned qskip = dPAD(q);
    const unsigned rskip = dPAD(r);
    dReal *aa = A;
    const dReal *bb = B;
    for (unsigned i = p; i != 0; aa+=rskip, bb+=qskip, --i) {
        dReal *a = aa;
        const dReal *cc = C, *ccend = C + r;
        for (; cc != ccend; ++a, ++cc) {
            dReal sum = REAL(0.0);
            const dReal *c = cc;
            const dReal *b = bb, *bend = bb + q;
            for (; b != bend; c+=rskip, ++b) {
                sum += (*b) * (*c);
            }
            (*a) = sum; 
        }
    }
}


/*extern */
void dxMultiply1(dReal *A, const dReal *B, const dReal *C, unsigned p, unsigned q, unsigned r)
{
    dAASSERT (A && B && C && p>0 && q>0 && r>0);
    const unsigned pskip = dPAD(p);
    const unsigned rskip = dPAD(r);
    dReal *aa = A;
    const dReal *bb = B, *bbend = B + p;
    for (; bb != bbend; aa += rskip, ++bb) {
        dReal *a = aa;
        const dReal *cc = C, *ccend = C + r;
        for (; cc != ccend; ++a, ++cc) {
            dReal sum = REAL(0.0);
            const dReal *b = bb, *c = cc;
            for (unsigned k = q; k != 0; b += pskip, c += rskip, --k) {
                sum += (*b) * (*c);
            }
            (*a) = sum;
        }
    }
}


/*extern */
void dxMultiply2(dReal *A, const dReal *B, const dReal *C, unsigned p, unsigned q, unsigned r)
{
    dAASSERT (A && B && C && p>0 && q>0 && r>0);
    const unsigned rskip = dPAD(r);
    const unsigned qskip = dPAD(q);
    dReal *aa = A;
    const dReal *bb = B;
    for (unsigned i = p; i != 0; aa += rskip, bb += qskip, --i) {
        dReal *a = aa, *aend = aa + r;
        const dReal *cc = C;
        for (; a != aend; cc+=qskip, ++a) {
            dReal sum = REAL(0.0);
            const dReal *b = bb, *c = cc, *cend = cc + q;
            for (; c != cend; ++b, ++c) {
                sum += (*b) * (*c);
            }
            (*a) = sum; 
        }
    }
}


/*extern */
int dxFactorCholesky(dReal *A, unsigned n, void *tmpBuf/*[n]*/)
{
    dAASSERT (n > 0 && A);
    bool failure = false;
    
    dReal *alloctedBuf = NULL;
    sizeint allocatedSize;

    const unsigned nskip = dPAD (n);
    
    dReal *recip = (dReal *)tmpBuf;
    if (tmpBuf == NULL) {
        allocatedSize = n * sizeof(dReal);
        alloctedBuf = allocatedSize > STACK_ALLOC_MAX ? (dReal *)dAlloc(allocatedSize) : NULL;
        recip = alloctedBuf != NULL ? alloctedBuf : (dReal*)ALLOCA(allocatedSize);
    }

    dReal *aa = A;
    for (unsigned i = 0; i < n; aa += nskip, ++i) {
        dReal *cc = aa;
        {
            const dReal *bb = A;
            for (unsigned j = 0; j < i; bb += nskip, ++cc, ++j) {
                dReal sum = *cc;
                const dReal *a = aa, *b = bb, *bend = bb + j;
                for (; b != bend; ++a, ++b) {
                    sum -= (*a) * (*b);
                }
                *cc = sum * recip[j];
            }
        }
        {
            dReal sum = *cc;
            dReal *a = aa, *aend = aa + i;
            for (; a != aend; ++a) {
                sum -= (*a)*(*a);
            }
            if (sum <= REAL(0.0)) {
                failure = true;
                break;
            }
            dReal sumsqrt = dSqrt(sum);
            *cc = sumsqrt;
            recip[i] = dRecip (sumsqrt);
        }
    }
    
    if (alloctedBuf != NULL) {
        dFree(alloctedBuf, allocatedSize);
    }

    return failure ? 0 : 1;
}


/*extern */
void dxSolveCholesky(const dReal *L, dReal *b, unsigned n, void *tmpBuf/*[n]*/)
{
    dAASSERT (n > 0 && L && b);

    dReal *alloctedBuf = NULL;
    sizeint allocatedSize;

    const unsigned nskip = dPAD (n);

    dReal *y = (dReal *)tmpBuf;
    if (tmpBuf == NULL) {
        allocatedSize = n * sizeof(dReal);
        alloctedBuf = allocatedSize > STACK_ALLOC_MAX ? (dReal *)dAlloc(allocatedSize) : NULL;
        y = alloctedBuf != NULL ? alloctedBuf : (dReal*)ALLOCA(allocatedSize);
    }

    {
        const dReal *ll = L;
        for (unsigned i = 0; i < n; ll += nskip, ++i) {
            dReal sum = REAL(0.0);
            for (unsigned k = 0; k < i; ++k) {
                sum += ll[k] * y[k];
            }
            dIASSERT(ll[i] != dReal(0.0));
            y[i] = (b[i] - sum) / ll[i];
        }
    }
    {
        const dReal *ll = L + (n - 1) * (nskip + 1);
        for (unsigned i = n; i > 0; ll -= nskip + 1) {
            --i;
            dReal sum = REAL(0.0);
            const dReal *l = ll + nskip;
            for (unsigned k = i + 1; k < n; l += nskip, ++k) {
                sum += (*l) * b[k];
            }
            dIASSERT(*ll != dReal(0.0));
            b[i] = (y[i] - sum) / (*ll);
        }
    }

    if (alloctedBuf != NULL) {
        dFree(alloctedBuf, allocatedSize);
    }
}


/*extern */
int dxInvertPDMatrix(const dReal *A, dReal *Ainv, unsigned n, void *tmpBuf/*[nskip*(n+2)]*/)
{
    dAASSERT (n > 0 && A && Ainv);
    bool success = false;

    dReal *alloctedBuf = NULL;
    sizeint allocatedSize;

    sizeint choleskyFactorSize = dxEstimateFactorCholeskyTmpbufSize(n);
    sizeint choleskySolveSize = dxEstimateSolveCholeskyTmpbufSize(n);
    sizeint choleskyMaxSize = dMACRO_MAX(choleskyFactorSize, choleskySolveSize);
    dIASSERT(choleskyMaxSize % sizeof(dReal) == 0);

    const unsigned nskip = dPAD (n);
    const sizeint nskip_mul_n = (sizeint)nskip * n;
    
    dReal *tmp = (dReal *)tmpBuf;
    if (tmpBuf == NULL) {
        allocatedSize = choleskyMaxSize + (nskip + nskip_mul_n) * sizeof(dReal);
        alloctedBuf = allocatedSize > STACK_ALLOC_MAX ? (dReal *)dAlloc(allocatedSize) : NULL;
        tmp = alloctedBuf != NULL ? alloctedBuf : (dReal*)ALLOCA(allocatedSize);
    }

    dReal *X = (dReal *)((char *)tmp + choleskyMaxSize);
    dReal *L = X + nskip;
    memcpy (L, A, nskip_mul_n * sizeof(dReal));
    if (dxFactorCholesky(L, n, tmp)) {
        dSetZero (Ainv, nskip_mul_n);	// make sure all padding elements set to 0
        dReal *aa = Ainv, *xi = X, *xiend = X + n;
        for (; xi != xiend; ++aa, ++xi) {
            dSetZero(X, n);
            *xi = REAL(1.0);
            dxSolveCholesky(L, X, n, tmp);
            dReal *a = aa;
            const dReal *x = X, *xend = X + n;
            for (; x != xend; a += nskip, ++x) {
                *a = *x;
            }
        }
        success = true;
    }

    if (alloctedBuf != NULL) {
        dFree(alloctedBuf, allocatedSize);
    }

    return success ? 1 : 0;
}


/*extern */
int dxIsPositiveDefinite(const dReal *A, unsigned n, void *tmpBuf/*[nskip*(n+1)]*/)
{
    dAASSERT (n > 0 && A);

    dReal *alloctedBuf = NULL;
    sizeint allocatedSize;

    sizeint choleskyFactorSize = dxEstimateFactorCholeskyTmpbufSize(n);
    dIASSERT(choleskyFactorSize % sizeof(dReal) == 0);

    const unsigned nskip = dPAD (n);
    const sizeint nskip_mul_n = (sizeint)nskip * n;
    
    dReal *tmp = (dReal *)tmpBuf;
    if (tmpBuf == NULL) {
        allocatedSize = choleskyFactorSize + nskip_mul_n * sizeof(dReal);
        alloctedBuf = allocatedSize > STACK_ALLOC_MAX ? (dReal *)dAlloc(allocatedSize) : NULL;
        tmp = alloctedBuf != NULL ? alloctedBuf : (dReal*)ALLOCA(allocatedSize);
    }

    dReal *Acopy = (dReal *)((char *)tmp + choleskyFactorSize);
    memcpy(Acopy, A, nskip_mul_n * sizeof(dReal));
    int factorResult = dxFactorCholesky (Acopy, n, tmp);

    if (alloctedBuf != NULL) {
        dFree(alloctedBuf, allocatedSize);
    }

    return factorResult;
}


/*extern */
void dxLDLTAddTL(dReal *L, dReal *d, const dReal *a, unsigned n, unsigned nskip, void *tmpBuf/*[2*nskip]*/)
{
    dAASSERT(L && d && a && n > 0 && nskip >= n);

    if (n < 2) return;

    dReal *alloctedBuf = NULL;
    sizeint allocatedSize;

    dReal *W1 = (dReal *)tmpBuf;
    if (tmpBuf == NULL) {
        allocatedSize = nskip * (2 * sizeof(dReal));
        alloctedBuf = allocatedSize > STACK_ALLOC_MAX ? (dReal *)dAlloc(allocatedSize) : NULL;
        W1 = alloctedBuf != NULL ? alloctedBuf : (dReal*)ALLOCA(allocatedSize);
    }

    dReal *W2 = W1 + nskip;

    W1[0] = REAL(0.0);
    W2[0] = REAL(0.0);
    for (unsigned j = 1; j < n; ++j) {
        W1[j] = W2[j] = (dReal) (a[j] * M_SQRT1_2);
    }
    dReal W11 = (dReal) ((REAL(0.5)*a[0]+1)*M_SQRT1_2);
    dReal W21 = (dReal) ((REAL(0.5)*a[0]-1)*M_SQRT1_2);

    dReal alpha1 = REAL(1.0);
    dReal alpha2 = REAL(1.0);

    {
        dReal dee = d[0];
        dReal alphanew = alpha1 + (W11*W11)*dee;
        dIASSERT(alphanew != dReal(0.0));
        dee /= alphanew;
        dReal gamma1 = W11 * dee;
        dee *= alpha1;
        alpha1 = alphanew;
        alphanew = alpha2 - (W21*W21)*dee;
        dee /= alphanew;
        //dReal gamma2 = W21 * dee;
        alpha2 = alphanew;
        dReal k1 = REAL(1.0) - W21*gamma1;
        dReal k2 = W21*gamma1*W11 - W21;
        dReal *ll = L + nskip;
        for (unsigned p = 1; p < n; ll += nskip, ++p) {
            dReal Wp = W1[p];
            dReal ell = *ll;
            W1[p] =    Wp - W11*ell;
            W2[p] = k1*Wp +  k2*ell;
        }
    }

    dReal *ll = L + (nskip + 1);
    for (unsigned j = 1; j < n; ll += nskip + 1, ++j) {
        dReal k1 = W1[j];
        dReal k2 = W2[j];

        dReal dee = d[j];
        dReal alphanew = alpha1 + (k1*k1)*dee;
        dIASSERT(alphanew != dReal(0.0));
        dee /= alphanew;
        dReal gamma1 = k1 * dee;
        dee *= alpha1;
        alpha1 = alphanew;
        alphanew = alpha2 - (k2*k2)*dee;
        dee /= alphanew;
        dReal gamma2 = k2 * dee;
        dee *= alpha2;
        d[j] = dee;
        alpha2 = alphanew;

        dReal *l = ll + nskip;
        for (unsigned p = j + 1; p < n; l += nskip, ++p) {
            dReal ell = *l;
            dReal Wp = W1[p] - k1 * ell;
            ell += gamma1 * Wp;
            W1[p] = Wp;
            Wp = W2[p] - k2 * ell;
            ell -= gamma2 * Wp;
            W2[p] = Wp;
            *l = ell;
        }
    }

    if (alloctedBuf != NULL) {
        dFree(alloctedBuf, allocatedSize);
    }
}


// macros for dLDLTRemove() for accessing A - either access the matrix
// directly or access it via row pointers. we are only supposed to reference
// the lower triangle of A (it is symmetric), but indexes i and j come from
// permutation vectors so they are not predictable. so do a test on the
// indexes - this should not slow things down too much, as we don't do this
// in an inner loop.

#define _GETA(i,j) (A[i][j])
//#define _GETA(i,j) (A[(i)*nskip+(j)])
#define GETA(i,j) ((i > j) ? _GETA(i,j) : _GETA(j,i))


/*extern */
void dxLDLTRemove(dReal **A, const unsigned *p, dReal *L, dReal *d,
    unsigned n1, unsigned n2, unsigned r, unsigned nskip, void *tmpBuf/*n2 + 2*nskip*/)
{
    dAASSERT(A && p && L && d && n1 > 0 && n2 > 0 /*&& r >= 0 */&& r < n2 &&
        n1 >= n2 && nskip >= n1);
#ifndef dNODEBUG
    for (unsigned i = 0; i < n2; ++i) dIASSERT(p[i] >= 0 && p[i] < n1);
#endif

    if (r == n2 - 1) {
        return;		// deleting the last row/col is easy
    }

    dReal *alloctedBuf = NULL;
    sizeint allocatedSize;

    sizeint LDLTAddTLSize = dxEstimateLDLTAddTLTmpbufSize(nskip);
    dIASSERT(LDLTAddTLSize % sizeof(dReal) == 0);
    
    dReal *tmp = (dReal *)tmpBuf;
    if (tmpBuf == NULL) {
        allocatedSize = LDLTAddTLSize + n2 * sizeof(dReal);
        alloctedBuf = allocatedSize > STACK_ALLOC_MAX ? (dReal *)dAlloc(allocatedSize) : NULL;
        tmp = alloctedBuf != NULL ? alloctedBuf : (dReal*)ALLOCA(allocatedSize);
    }
    
    if (r == 0) {
        dReal *a = (dReal *)((char *)tmp + LDLTAddTLSize);
        const unsigned p_0 = p[0];
        for (unsigned i = 0; i < n2; ++i) {
            a[i] = -GETA(p[i],p_0);
        }
        a[0] += REAL(1.0);
        dxLDLTAddTL (L, d, a, n2, nskip, tmp);
    }
    else {
        dReal *t = (dReal *)((char *)tmp + LDLTAddTLSize);
        {
            dReal *Lcurr = L + r*nskip;
            for (unsigned i = 0; i < r; ++Lcurr, ++i) {
                dIASSERT(d[i] != dReal(0.0));
                t[i] = *Lcurr / d[i];
            }
        }
        dReal *a = t + r;
        {
            dReal *Lcurr = L + r * nskip;
            const unsigned *pp_r = p + r, p_r = *pp_r;
            const unsigned n2_minus_r = n2 - r;
            for (unsigned i = 0; i < n2_minus_r; Lcurr += nskip, ++i) {
                a[i] = dDot(Lcurr, t, r) - GETA(pp_r[i], p_r);
            }
        }
        a[0] += REAL(1.0);
        dxLDLTAddTL (L + (sizeint)(nskip + 1) * r, d + r, a, n2 - r, nskip, tmp);
    }

    // snip out row/column r from L and d
    dxRemoveRowCol (L, n2, nskip, r);
    if (r < (n2 - 1)) memmove (d + r, d + r + 1, (n2 - r - 1) * sizeof(dReal));

    if (alloctedBuf != NULL) {
        dFree(alloctedBuf, allocatedSize);
    }
}


/*extern */
void dxRemoveRowCol(dReal *A, unsigned n, unsigned nskip, unsigned r)
{
    dAASSERT(A && n > 0 && nskip >= n && r >= 0 && r < n);
    if (r >= n - 1) return;
    if (r > 0) {
        {
            const sizeint move_size = (n - r - 1) * sizeof(dReal);
            dReal *Adst = A + r;
            for (unsigned i = 0; i < r; Adst += nskip, ++i) {
                dReal *Asrc = Adst + 1;
                memmove (Adst, Asrc, move_size);
            }
        }
        {
            const sizeint cpy_size = r * sizeof(dReal);
            dReal *Adst = A + (sizeint)nskip * r;
            unsigned n1 = n - 1;
            for (unsigned i = r; i < n1; ++i) {
                dReal *Asrc = Adst + nskip;
                memcpy (Adst, Asrc, cpy_size);
                Adst = Asrc;
            }
        }
    }
    {
        const sizeint cpy_size = (n - r - 1) * sizeof(dReal);
        dReal *Adst = A + (sizeint)(nskip + 1) * r;
        unsigned n1 = n - 1;
        for (unsigned i = r; i < n1; ++i) {
            dReal *Asrc = Adst + (nskip + 1);
            memcpy (Adst, Asrc, cpy_size);
            Adst = Asrc - 1;
        }
    }
}


#undef dSetZero
#undef dSetValue
//#undef dDot
#undef dMultiply0
#undef dMultiply1
#undef dMultiply2
#undef dFactorCholesky
#undef dSolveCholesky
#undef dInvertPDMatrix
#undef dIsPositiveDefinite
#undef dLDLTAddTL
#undef dLDLTRemove
#undef dRemoveRowCol


/*extern ODE_API */
void dSetZero(dReal *a, int n)
{
    dxSetZero(a, n);
}

/*extern ODE_API */
void dSetValue(dReal *a, int n, dReal value)
{
    dxSetValue(a, n, value);
}

// dReal dDot (const dReal *a, const dReal *b, int n);

/*extern ODE_API */
void dMultiply0(dReal *A, const dReal *B, const dReal *C, int p,int q,int r)
{
    dxMultiply0(A, B, C, p, q, r);
}

/*extern ODE_API */
void dMultiply1(dReal *A, const dReal *B, const dReal *C, int p,int q,int r)
{
    dxMultiply1(A, B, C, p, q, r);
}

/*extern ODE_API */
void dMultiply2(dReal *A, const dReal *B, const dReal *C, int p,int q,int r)
{
    dxMultiply2(A, B, C, p, q, r);
}

/*extern ODE_API */
int dFactorCholesky(dReal *A, int n)
{
    return dxFactorCholesky(A, n, NULL);
}

/*extern ODE_API */
void dSolveCholesky(const dReal *L, dReal *b, int n)
{
    dxSolveCholesky(L, b, n, NULL);
}

/*extern ODE_API */
int dInvertPDMatrix (const dReal *A, dReal *Ainv, int n)
{
    return dxInvertPDMatrix(A, Ainv, n, NULL);
}

/*extern ODE_API */
int dIsPositiveDefinite(const dReal *A, int n)
{
    return dxIsPositiveDefinite(A, n, NULL);
}


/*extern ODE_API */
void dLDLTAddTL(dReal *L, dReal *d, const dReal *a, int n, int nskip)
{
    dxLDLTAddTL(L, d, a, n, nskip, NULL);
}

/*extern ODE_API */
void dLDLTRemove(dReal **A, const int *p, dReal *L, dReal *d, int n1, int n2, int r, int nskip)
{
    dxLDLTRemove(A, (const unsigned *)p, L, d, n1, n2, r, nskip, NULL);
    dSASSERT(sizeof(unsigned) == sizeof(*p));
}

/*extern ODE_API */
void dRemoveRowCol(dReal *A, int n, int nskip, int r)
{
    dxRemoveRowCol(A, n, nskip, r);
}

