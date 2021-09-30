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
 * The default threading instance holder class implementation
 * Copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")
 */


#include <ode/common.h>
#include <ode/threading_impl.h>
#include "config.h"
#include "default_threading.h"
#include "error.h"


/*static */dThreadingImplementationID DefaultThreadingHolder::m_defaultThreadingImpl = NULL;
/*static */const dThreadingFunctionsInfo *DefaultThreadingHolder::m_defaultThreadingFunctions = NULL;


/*static */
bool DefaultThreadingHolder::initializeDefaultThreading()
{
    dIASSERT(m_defaultThreadingImpl == NULL);

    bool initResult = false;

    dThreadingImplementationID threadingImpl = dThreadingAllocateSelfThreadedImplementation();

    if (threadingImpl != NULL)
    {
        m_defaultThreadingFunctions = dThreadingImplementationGetFunctions(threadingImpl);
        m_defaultThreadingImpl = threadingImpl;

        initResult = true;
    }

    return initResult;
}

/*static */
void DefaultThreadingHolder::finalizeDefaultThreading()
{
    dThreadingImplementationID threadingImpl = m_defaultThreadingImpl;

    if (threadingImpl != NULL)
    {
        dThreadingFreeImplementation(threadingImpl);

        m_defaultThreadingFunctions = NULL;
        m_defaultThreadingImpl = NULL;
    }
}

