/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
//
// C++ Implementation: SimulationPlanningController
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef SOFA_COMPONENT_CONTROLLER_SIMULATION_PLANNING_CONTROLLER_H
#define SOFA_COMPONENT_CONTROLLER_SIMULATION_PLANNING_CONTROLLER_H

#include <SofaUserInteraction/Controller.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/helper/vector.h>
#include "NeedleEvent.h"
#include <sofa/core/ExecParams.h>

#include <sofa/simulation/common/PauseEvent.h>
#include <sofa/simulation/common/PropagateEventVisitor.h>

#include <sofa/simulation/common/Simulation.h>

#include "../NeedleConstraints/NeedleConstraint.h"


namespace sofa
{

namespace component
{

namespace controller
{

template <class DataTypes>
class SOFA_USER_INTERACTION_API SimulationPlanningController : public Controller
{
public:
    SOFA_CLASS(SimulationPlanningController,Controller);
protected:

    SimulationPlanningController();

    virtual ~SimulationPlanningController() {}
public:

    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename Coord::Pos Pos;
    typedef typename Coord::Rot Rot;

    typedef sofa::core::topology::Topology::Edge Edge;

    Data<VecCoord> entryPoints;
    Data<VecCoord> pilotPoint;
    Data<VecCoord> needleDofs;
    Data<sofa::helper::vector<Edge> > needleEdges;

    Data<Real> needleLength;
    Data<Real> needleOffset;
    Data<int> needleDiscretization;
    Data<int> actualEntryPoint;
    Data<Real> positionIncrement;
    Data<int> numIdleSteps;

    int idleStep, eventType;

    std::vector<std::vector<Pos> > objectPositions;

    sofa::component::constraintset::BaseNeedleConstraint<Real>* needleConstraintToDelete;

    int needleDirection, insertionStep;

    virtual void init()  { reinit(); }

    virtual void reinit();

    void sendSimulationData();

    void updateOnBeginStep();

    void updateOnEndStep();

    virtual void bwdInit();

    virtual void applyController(void);

    virtual void handleEvent(core::objectmodel::Event *event);


protected:

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif //SOFA_COMPONENT_CONTROLLER_SIMULATION_PLANNING_CONTROLLER_H
