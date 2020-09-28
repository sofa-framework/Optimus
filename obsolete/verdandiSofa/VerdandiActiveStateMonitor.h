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
#ifndef SOFA_COMPONENT_MISC_VERDANDI_ACTIVE_STATE_MONITOR_H
#define SOFA_COMPONENT_MISC_VERDANDI_ACTIVE_STATE_MONITOR_H

#include <SofaValidation/Monitor.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Vec.h>

#include "VerdandiAnimationLoop.h"

//#include "../SteerableNeedlesModeling/SteerableNeedleTissueParameters.h"

namespace sofa
{

namespace component
{

namespace misc
{

template<class DataTypes>
class VerdandiActiveStateMonitor : public virtual Monitor<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(VerdandiActiveStateMonitor,DataTypes), SOFA_TEMPLATE(Monitor,DataTypes));

    //typedef sofa::component::controller::SimulationPlanningController<defaulttype::Rigid3dTypes> TPlanningController;
    typedef sofa::simulation::VerdandiAnimationLoop TVerdandiAnimationLoop;

    SingleLink<VerdandiActiveStateMonitor<DataTypes>, TVerdandiAnimationLoop, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> verdandiLoopLink;
    //Data<string> m_objectID;

    typedef Monitor<DataTypes> Inherit;
    typedef typename DataTypes::VecReal VecReal;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;

    //typedef typename sofa::component::container::SteerableNeedleTissueParameters<Real> TTissueParameters;
    //typedef typename Coord::Pos Pos;
protected:
    VerdandiActiveStateMonitor();
    //~VerdandiActiveStateMonitor();*/
    //size_t controllerIndex;
public:
    virtual void init();
    virtual void bwdInit() {}
    //virtual void handleEvent(core::objectmodel::Event *event) {}


};

}

}

}

#endif
