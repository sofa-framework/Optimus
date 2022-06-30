/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include "../initOptimusPlugin.h"
#include <sofa/core/objectmodel/Event.h>



namespace sofa
{

namespace component
{

namespace stochastic
{



class SOFA_STOCHASTIC_API PredictionEndEvent : public sofa::core::objectmodel::Event
{
protected:
    SReal dt;

public:
    SOFA_EVENT_H( PredictionEndEvent )

    PredictionEndEvent( SReal dt );

    ~PredictionEndEvent();

    SReal getDt() const { return dt; }

    inline static const char* GetClassName() { return "AssimilationEndEvent"; }
};



class SOFA_STOCHASTIC_API CorrectionEndEvent : public sofa::core::objectmodel::Event
{
protected:
    SReal dt;

public:
    SOFA_EVENT_H( CorrectionEndEvent )

    CorrectionEndEvent( SReal dt );

    ~CorrectionEndEvent();

    SReal getDt() const { return dt; }

    inline static const char* GetClassName() { return "AssimilationEndEvent"; }
};



} // namespace stochastic

} // namespace components

} // namespace sofa

