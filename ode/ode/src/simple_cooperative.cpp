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
 * The simple cooperative class implementation
 * Copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")
 */


#include <ode/common.h>
#include <ode/cooperative.h>
#include "config.h"
#include "simple_cooperative.h"
#include "default_threading.h"


/*virtual */
dxSimpleCooperative::~dxSimpleCooperative()
{
    // The virtual destructor
}


/*virtual */
const dxThreadingFunctionsInfo *dxSimpleCooperative::retrieveThreadingDefaultImpl(dThreadingImplementationID &out_defaultImpl)
{
    out_defaultImpl = DefaultThreadingHolder::getDefaultThreadingImpl();
    return DefaultThreadingHolder::getDefaultThreadingFunctions();
}


//////////////////////////////////////////////////////////////////////////
// Public interface functions

static inline 
dCooperativeID encodeCooperativeID(dxSimpleCooperative *cooperativeInstance)
{
    return (dCooperativeID)cooperativeInstance;
}


/*extern ODE_API */
dCooperativeID dCooperativeCreate(const dThreadingFunctionsInfo *functionInfo/*=NULL*/, dThreadingImplementationID threadingImpl/*=NULL*/)
{
    dxSimpleCooperative *cooperativeInstance = new dxSimpleCooperative(functionInfo, threadingImpl);
    
    dCooperativeID result = encodeCooperativeID(cooperativeInstance);
    return result;
}

/*extern ODE_API */
void dCooperativeDestroy(dCooperativeID cooperative)
{
    dxSimpleCooperative *cooperativeInstance = decodeCooperativeID(cooperative);

    if (cooperativeInstance != NULL)
    {
        delete cooperativeInstance;
    }
}

