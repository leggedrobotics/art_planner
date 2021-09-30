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
 * Resource accounting/preallocation class implementations
 * Copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")
 */


#include <ode/common.h>
#include <ode/cooperative.h>
#include "config.h"
#include "resource_control.h"
#include "simple_cooperative.h"


//////////////////////////////////////////////////////////////////////////
// dxResourceRequirementDescriptor();

dxResourceRequirementDescriptor::~dxResourceRequirementDescriptor()
{
    // Do nothing
}


//////////////////////////////////////////////////////////////////////////
// dxRequiredResourceContainer

dxRequiredResourceContainer::~dxRequiredResourceContainer()
{
    freeResources();
}


bool dxRequiredResourceContainer::allocateResources(const dxResourceRequirementDescriptor &requirementDescriptor)
{
    bool result = false;
    
    bool bufferAllocated = false;

    do
    {
        sizeint memorySizeRequirement = requirementDescriptor.getMemorySizeRequirement();
        
        if (memorySizeRequirement != 0)
        {
            unsigned memoryAlignmentRequirement = requirementDescriptor.getMemoryAlignmentRequirement();
            void *bufferAllocated = m_memoryAllocation.allocAligned(memorySizeRequirement, memoryAlignmentRequirement);
            if (bufferAllocated == NULL)
            {
                break;
            }
        }
        bufferAllocated = true;

        dxThreadingBase *relatedThreading = requirementDescriptor.getrelatedThreading();
        dIASSERT(relatedThreading != NULL);

        unsigned simultaneousCallRequirement = requirementDescriptor.getSimultaneousCallRequirement();
        if (simultaneousCallRequirement != 0)
        {
            if (!relatedThreading->PreallocateResourcesForThreadedCalls(simultaneousCallRequirement))
            {
                break;
            }
        }

        dCallWaitID stockCallWait = NULL;

        if (requirementDescriptor.getIsStockCallWaitRequired())
        {
             stockCallWait = relatedThreading->AllocateOrRetrieveStockCallWaitID();
             if (stockCallWait == NULL)
             {
                 break;
             }
        }

        m_relatedThreading = relatedThreading;
    	m_stockCallWait = stockCallWait;
    
    	result = true;
    }
    while (false);

    if (!result)
    {
        if (bufferAllocated)
        {
            m_memoryAllocation.freeAllocation();
        }
    }
    
    return result;

}

void dxRequiredResourceContainer::freeResources()
{
    if (m_relatedThreading != NULL)
    {
        m_relatedThreading = NULL;
        m_stockCallWait = NULL;
        m_memoryAllocation.freeAllocation();
    }
    else
    {
        dIASSERT(m_stockCallWait == NULL);
        dIASSERT(m_memoryAllocation.getUserAreaPointer() == NULL);
    }
}


//////////////////////////////////////////////////////////////////////////
// Public interface functions

static inline 
dResourceRequirementsID encodeResourceRequirementsID(dxResourceRequirementDescriptor *requirementsDescriptor)
{
    return (dResourceRequirementsID)requirementsDescriptor;
}


/*extern ODE_API */
dResourceRequirementsID dResourceRequirementsCreate(dCooperativeID cooperative)
{
    dAASSERT(cooperative != NULL);

    dxSimpleCooperative *cooperativeInstance = decodeCooperativeID(cooperative);
    dxThreadingBase *threading = cooperativeInstance->getRelatedThreading();

    dxResourceRequirementDescriptor *requirementsDescriptor = new dxResourceRequirementDescriptor(threading);
    
    dResourceRequirementsID result = encodeResourceRequirementsID(requirementsDescriptor);
    return result;
}

/*extern ODE_API */
void dResourceRequirementsDestroy(dResourceRequirementsID requirements)
{
    dxResourceRequirementDescriptor *requirementsDescriptor = decodeResourceRequirementsID(requirements);

    if (requirementsDescriptor != NULL)
    {
        delete requirementsDescriptor;
    }
}


/*extern ODE_API */
dResourceRequirementsID dResourceRequirementsClone(/*const */dResourceRequirementsID requirements)
{
    dAASSERT(requirements != NULL);

    dxResourceRequirementDescriptor *requirementsDescriptor = decodeResourceRequirementsID(requirements);

    dxResourceRequirementDescriptor *descriptorClone = new dxResourceRequirementDescriptor(*requirementsDescriptor);

    dResourceRequirementsID result = encodeResourceRequirementsID(descriptorClone);
    return result;
}

/*extern ODE_API */
void dResourceRequirementsMergeIn(dResourceRequirementsID summaryRequirements, /*const */dResourceRequirementsID extraRequirements)
{
    dAASSERT(summaryRequirements != NULL);
    dAASSERT(extraRequirements != NULL);

    dxResourceRequirementDescriptor *summaryDescriptor = decodeResourceRequirementsID(summaryRequirements);
    dxResourceRequirementDescriptor *extraDescriptor = decodeResourceRequirementsID(extraRequirements);

    summaryDescriptor->mergeAnotherDescriptorIn(*extraDescriptor);
}


//////////////////////////////////////////////////////////////////////////

static inline 
dResourceContainerID encodeResourceContainerID(dxRequiredResourceContainer *containerInstance)
{
    return (dResourceContainerID)containerInstance;
}


/*extern ODE_API */
dResourceContainerID dResourceContainerAcquire(/*const */dResourceRequirementsID requirements)
{
    dAASSERT(requirements != NULL);

    dResourceContainerID result = NULL;
    bool allocationSucceeded = false;
    
    dxRequiredResourceContainer *containerInstance;
    bool containerAllocated = false;

    dxResourceRequirementDescriptor *requirementsInstance = decodeResourceRequirementsID(requirements);

    do
    {
        containerInstance = new dxRequiredResourceContainer();

        if (containerInstance == NULL)
        {
            break;
        }

        containerAllocated = true;
    
        if (!containerInstance->allocateResources(*requirementsInstance))
        {
            break;
        }

    	result = encodeResourceContainerID(containerInstance);
        allocationSucceeded = true;
    }
    while (false);

    if (!allocationSucceeded)
    {
        if (containerAllocated)
        {
            delete containerInstance;
        }
    }
    
    return result;
}

/*extern ODE_API */
void dResourceContainerDestroy(dResourceContainerID resources)
{
    dxRequiredResourceContainer *containerInstance = decodeResourceContainerID(resources);

    if (containerInstance != NULL)
    {
        delete containerInstance;
    }
}

