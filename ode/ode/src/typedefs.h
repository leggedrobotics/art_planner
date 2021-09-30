/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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

#ifndef _ODE_TYPEDEFS_H_
#define _ODE_TYPEDEFS_H_

#include <ode/odeconfig.h>

#include "error.h"


/*
 * Internal typedefs to map public types into more convenient private types
 */


typedef dint64 int64;
dSASSERT(sizeof(int64) == 8);

typedef duint64 uint64;
dSASSERT(sizeof(uint64) == 8);

typedef dint32 int32;
dSASSERT(sizeof(int32) == 4);

typedef duint32 uint32;
dSASSERT(sizeof(uint32) == 4);

typedef dint16 int16;
dSASSERT(sizeof(int16) == 2);

typedef duint16 uint16;
dSASSERT(sizeof(uint16) == 2);

typedef dint8 int8;
dSASSERT(sizeof(int8) == 1);

typedef duint8 uint8;
dSASSERT(sizeof(uint8) == 1);


typedef dintptr intptr;
dSASSERT(sizeof(intptr) == sizeof(void *));

typedef duintptr uintptr;
dSASSERT(sizeof(uintptr) == sizeof(void *));

typedef ddiffint diffint;
dSASSERT(sizeof(diffint) == sizeof(void *)); // So far, we choose to not support systems that have accessible memory segment size smaller than the pointer size

typedef dsizeint sizeint;
dSASSERT(sizeof(sizeint) == sizeof(void *)); // So far, we choose to not support systems that have accessible memory segment size smaller than the pointer size


#endif
