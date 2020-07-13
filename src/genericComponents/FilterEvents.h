/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2020 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include "initOptimusPlugin.h"
#include <sofa/core/objectmodel/Event.h>


namespace sofa
{

namespace component
{

namespace stochastic
{



class  SOFA_STOCHASTIC_API PredictionEndEvent : public sofa::core::objectmodel::Event
{
public:

    SOFA_EVENT_H( PredictionEndEvent )

    PredictionEndEvent( SReal dt );

    ~PredictionEndEvent();

    SReal getDt() const { return dt; }

    inline static const char* GetClassName() { return "AssimilationEndEvent"; }

protected:
    SReal dt;
};


class SOFA_STOCHASTIC_API CorrectionEndEvent : public sofa::core::objectmodel::Event
{
public:

    SOFA_EVENT_H( CorrectionEndEvent )

    CorrectionEndEvent( SReal dt );

    ~CorrectionEndEvent();

    SReal getDt() const { return dt; }

    inline static const char* GetClassName() { return "AssimilationEndEvent"; }

protected:
    SReal dt;
};



}   /// stochastic

}   /// components

}   /// sofa

