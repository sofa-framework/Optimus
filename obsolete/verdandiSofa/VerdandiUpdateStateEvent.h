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
#ifndef SOFA_SIMULATION_VERDANDI_UPDATE_STATE_EVENT_H
#define SOFA_SIMULATION_VERDANDI_UPDATE_STATE_EVENT_H

#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/common/common.h>
#include <string>

namespace sofa
{

namespace simulation
{

/**
  Event fired when needed to stop the animation.
*/
template <class DataTypes>
class SOFA_SIMULATION_COMMON_API VerdandiUpdateStateEvent : public sofa::core::objectmodel::Event
{
public:
    VerdandiUpdateStateEvent() {}
    //sofa::component::constraintset::HayBaseNeedleConstraint<DataTypes>* needleConstraints;

    //int type;
    //std::string objectID;

    ~VerdandiUpdateStateEvent() {}

    virtual const char* getClassName() const { return "VerdandiUpdateStateEvent"; }

};

} // namespace simulation

} // namespace sofa

#endif
