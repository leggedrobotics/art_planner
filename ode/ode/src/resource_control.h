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
 * Resource accounting/preallocation class declarations
 * Copyright (c) 2017-2019 Oleh Derevenko, odar@eleks.com (change all "a" to "e")
 */

#ifndef _ODE__PRIVATE_RESOURCE_CONTRIOL_H_
#define _ODE__PRIVATE_RESOURCE_CONTRIOL_H_


#include "objects.h"
#include "threading_base.h"
#include "odeou.h"
#include "common.h"
#include "error.h"


using _OU_NAMESPACE::CSimpleFlags;


class dxResourceRequirementDescriptor:
    public dBase
{
public:
    explicit dxResourceRequirementDescriptor(dxThreadingBase *relatedThreading):
        dBase(),
        m_relatedThreading(relatedThreading),
        m_memorySizeRequirement(0),
        m_memoryAlignmentRequirement(0),
        m_simultaneousCallRequirement(0),
        m_featureRequirements()
    {
    }

    ~dxResourceRequirementDescriptor();

    enum
    {
        STOCK_CALLWAIT_REQUIRED = 0x00000001,
    };

    void mergeAnotherDescriptorIn(const dxResourceRequirementDescriptor &anotherDescriptor)
    {
        dIASSERT(getrelatedThreading() == anotherDescriptor.getrelatedThreading()); // m_simultaneousCallRequirement typically depends on threading used

        CSimpleFlags::value_type allOtherFeatureFlags = anotherDescriptor.queryAllFeatureFlags();
        mergeAnotherDescriptorIn(anotherDescriptor.m_memorySizeRequirement, anotherDescriptor.m_memoryAlignmentRequirement, anotherDescriptor.m_simultaneousCallRequirement, allOtherFeatureFlags);
    }

    void mergeAnotherDescriptorIn(sizeint memorySizeRequirement/*=0*/, unsigned memoryAlignmentRequirement,
        unsigned simultaneousCallRequirement/*=0*/, unsigned featureRequirement/*=0*/)
    {
        m_memorySizeRequirement = dMACRO_MAX(m_memorySizeRequirement, memorySizeRequirement);
        m_memoryAlignmentRequirement = dMACRO_MAX(m_memoryAlignmentRequirement, memoryAlignmentRequirement);
        m_simultaneousCallRequirement = dMACRO_MAX(m_simultaneousCallRequirement, simultaneousCallRequirement);
        mergeFeatureFlags(featureRequirement);
    }

public:
    dxThreadingBase *getrelatedThreading() const { return m_relatedThreading; }
    sizeint getMemorySizeRequirement() const { return m_memorySizeRequirement; }
    unsigned getMemoryAlignmentRequirement() const { return m_memoryAlignmentRequirement; }

    unsigned getSimultaneousCallRequirement() const { return m_simultaneousCallRequirement; }

    bool getIsStockCallWaitRequired() const { return getStockCallWaitRequiredFlag(); }

private:
    enum
    {
        FL_STOCK_CALLWAIT_REQUIRED  = STOCK_CALLWAIT_REQUIRED,
    };

    bool getStockCallWaitRequiredFlag() const { return m_featureRequirements.GetFlagsMaskValue(FL_STOCK_CALLWAIT_REQUIRED); }

    CSimpleFlags::value_type queryAllFeatureFlags() const { return m_featureRequirements.QueryFlagsAllValues(); }
    void mergeFeatureFlags(CSimpleFlags::value_type flagValues) { m_featureRequirements.SignalFlagsMaskValue(flagValues); }

private:
    dxThreadingBase     *m_relatedThreading;
    sizeint              m_memorySizeRequirement;
    unsigned            m_memoryAlignmentRequirement;
    unsigned            m_simultaneousCallRequirement;
    CSimpleFlags        m_featureRequirements;
};

static inline 
dxResourceRequirementDescriptor *decodeResourceRequirementsID(dResourceRequirementsID requirements)
{
    return (dxResourceRequirementDescriptor *)requirements;
}


class dxRequiredResourceContainer:
    public dBase
{
public:
    dxRequiredResourceContainer():
        dBase(),
        m_relatedThreading(NULL),
        m_stockCallWait(NULL),
        m_memoryAllocation()
    {
    }

    ~dxRequiredResourceContainer();

    bool allocateResources(const dxResourceRequirementDescriptor &requirementDescriptor);
    void freeResources();

public:
    dxThreadingBase *getThreadingInstance() const { return m_relatedThreading; }
    dCallWaitID getStockCallWait() const { return m_stockCallWait; }
    void *getMemoryBufferPointer() const { return m_memoryAllocation.getUserAreaPointer(); }
    sizeint getMemoryBufferSize() const { return m_memoryAllocation.getUserAreaSize(); }

private:
    dxThreadingBase     *m_relatedThreading;
    dCallWaitID         m_stockCallWait;
    dxAlignedAllocation m_memoryAllocation;
};

static inline 
dxRequiredResourceContainer *decodeResourceContainerID(dResourceContainerID resources)
{
    return (dxRequiredResourceContainer *)resources;
}


#endif // #ifndef _ODE__PRIVATE_RESOURCE_CONTRIOL_H_
