/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * Threading base wrapper class header file.                             *
 * Copyright (C) 2011-2019 Oleh Derevenko. All rights reserved.          *
 * e-mail: odar@eleks.com (change all "a" to "e")                        *
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
 * A simple cooperative class definition
 * Copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")
 */


#ifndef _ODE__PRIVATE_SIMPLE_COOPERATIVE_H_
#define _ODE__PRIVATE_SIMPLE_COOPERATIVE_H_


#include "objects.h"
#include "threading_base.h"


typedef dxThreadingBase dxSimpleCooperative_ThreadingParent;
class dxSimpleCooperative:
    public dBase,
    public dxSimpleCooperative_ThreadingParent,
    private dxIThreadingDefaultImplProvider
{
public:
    dxSimpleCooperative(const dxThreadingFunctionsInfo *functionInfo, dThreadingImplementationID threadingImpl):
        dBase(),
        dxSimpleCooperative_ThreadingParent()
    {
        dxSimpleCooperative_ThreadingParent::setThreadingDefaultImplProvider(this);
        dxSimpleCooperative_ThreadingParent::assignThreadingImpl(functionInfo, threadingImpl);
    }

    virtual ~dxSimpleCooperative();

public:
    dxThreadingBase *getRelatedThreading() const { return const_cast<dxSimpleCooperative *>(this); }

private: // dxIThreadingDefaultImplProvider
    virtual const dxThreadingFunctionsInfo *retrieveThreadingDefaultImpl(dThreadingImplementationID &out_defaultImpl);
};


static inline 
dxSimpleCooperative *decodeCooperativeID(dCooperativeID cooperative)
{
    return (dxSimpleCooperative *)cooperative;
}


#endif // #ifndef _ODE__PRIVATE_SIMPLE_COOPERATIVE_H_
