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
#ifndef SOFA_COMPONENT_ENGINE_SIMULATIONPLANNINGCONTROLLER_INL
#define SOFA_COMPONENT_ENGINE_SIMULATIONPLANNINGCONTROLLER_INL

#include "SimulationPlanningController.h"
#include <sofa/helper/rmath.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa
{

namespace component
{

namespace controller
{

template<class DataTypes>
SimulationPlanningController<DataTypes>::SimulationPlanningController()
//: fileName( initData (&fileName, "fileName", "filename of a file containing the insertion points, direction and needle angle (7 floats)") )
: entryPoints( initData (&entryPoints, "entryPoints", "points of entry of the needle into the object") )
, pilotPoint( initData (&pilotPoint, "pilotPoint", "moving point (temporary solution)") )
, needleDofs( initData (&needleDofs, "needleDofs", "needle nodes") )
, needleEdges( initData (&needleEdges, "needleEdges", "needle edges") )
, needleLength( initData (&needleLength, Real(1.0), "needleLength", "needle length") )
, needleOffset( initData (&needleOffset, Real(0.001), "needleOffset", "initial distance between the tip and entry point") )
, needleDiscretization( initData (&needleDiscretization, int(10), "needleDiscretization", "initial distance between the tip and entry point") )
, actualEntryPoint( initData (&actualEntryPoint, int(0), "actualEntryPoint", "index of actually processe entry point") )
, positionIncrement( initData (&positionIncrement, Real(0.001), "positionIncrement", "how much the needle advances in one step") )
, numIdleSteps( initData (&numIdleSteps, int(-1), "numberOfIdleSteps", "number of steps the controller is waiting after the event is received") )
{
    this->f_listening.setValue(true);    
}

template<class DataTypes>
void SimulationPlanningController<DataTypes>::reinit() {
    helper::ReadAccessor<Data<VecCoord> > entryP(entryPoints);
    helper::WriteAccessor<Data<VecCoord> > dofs(needleDofs);
    helper::WriteAccessor<Data<VecCoord> > pilotP(pilotPoint);

    helper::WriteAccessor<Data<sofa::helper::vector<Edge> > > edges = needleEdges;

    int actualEntryP = actualEntryPoint.getValue();

    if (actualEntryP >= int(entryPoints.getValue().size()))
        actualEntryP = 0;

    std::cout << "Entry points size: " << entryP.size() << std::endl;
    dofs.clear();
    edges.clear();
    pilotP.clear();

    Rot r = DataTypes::getCRot(entryP[actualEntryP]);
    std::cout << "Rotation: " << r << std::endl;

    Real edgeL = needleLength.getValue()/(Real(needleDiscretization.getValue())-1);
    Pos initP = DataTypes::getCPos(entryP[actualEntryP]);

    std::cout << "Edge length: " << edgeL << std::endl;

    for (int i = 0; i < needleDiscretization.getValue(); i++) {
        Real offX = -needleLength.getValue()-needleOffset.getValue() + i*edgeL;
        Pos p(offX, 0.0, 0.0);
        Pos rotP = initP + r.rotate(p);

        //std::cout << "position " << p << " rotated to " << rotP << std::endl;

        Coord c;
        DataTypes::setCRot(c, r);
        DataTypes::setCPos(c, rotP);

        //std::cout << "Pushing back: " << c << std::endl;
        dofs.push_back(c);

        if (i == 0)
            pilotP.push_back(c);

        if (i > 0) {
            Edge e(i-1, i);
            edges.push_back(e);
        }
    }

    needleDirection = 1;
    insertionStep = 0;

    std::cout << this->getName() << ": Clearing object positions" << std::endl;
    objectPositions.clear();
    idleStep = -1;
}

template<class DataTypes>
void SimulationPlanningController<DataTypes>::bwdInit() {  }

template<class DataTypes>
void SimulationPlanningController<DataTypes>::updateOnBeginStep() {
    int actualEntryP = actualEntryPoint.getValue();

    if (actualEntryP >= int(entryPoints.getValue().size()))
        return;

    if (needleConstraintToDelete && idleStep >= 0) {
        if (idleStep % 10 == 0)
            std::cout << "Idle step: " << idleStep << std::endl;
        idleStep++;
        return;
    }

    if (insertionStep % 10 == 0)
        std::cout << "Step: " << insertionStep << std::endl;
    /*if (insertionStep < 0) {
        actualEntryPoint.setValue(actualEntryPoint.getValue()+1);
        if (actualEntryPoint.getValue() < int(entryPoints.getValue().size())) {
            //reinit();
            std::cout << "Reinitializing the simulation " << std::endl;
            simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
            simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());

            simulation::getSimulation()->init(root);

            //if( currentSimulation() ) this->unloadScene();
            //simulation::Node::SPtr root = simulation::getSimulation()->load ( filename.c_str() );
            //simulation::getSimulation()->init ( root.get() );
        }
        else
            return;
    }*/

    helper::ReadAccessor<Data<VecCoord> > entryP(entryPoints);
    Rot r = DataTypes::getCRot(entryP[actualEntryP]);

    helper::WriteAccessor<Data<VecCoord> > pilotP(pilotPoint);

    Pos actP = DataTypes::getCPos(pilotP[0]);

    Pos incP(Real(needleDirection)*positionIncrement.getValue(), 0.0, 0.0);
    Pos rotIncP = r.rotate(incP) + actP;

    //std::cout << "Shifted position: " << rotIncP << std::endl;

    DataTypes::setCPos(pilotP[0], rotIncP);

    insertionStep += needleDirection;

    needleConstraintToDelete = NULL;
}



template<class DataTypes>
void SimulationPlanningController<DataTypes>::sendSimulationData() {
    switch (eventType) {
        case 0: std::cout << "Simulation accomplished: needle too deep" << std::endl; break;
        case -1: std::cout << "Simulation accomplished: obstacle hit" << std::endl; break;
        case 1: std::cout << "Simulation accomplished: target hit" << std::endl; break;
    }

    std::cout << "The actual deformed coordinates: " << std::endl;

    for (size_t i = 0; i < objectPositions.size(); i++) {
        std::cout << "Object #" << i << std::endl;
        for (typename std::vector<Pos>::iterator it = objectPositions[i].begin(); it != objectPositions[i].end(); it++)
            std::cout << *it << std::endl;
    }
}


template<class DataTypes>
void SimulationPlanningController<DataTypes>::updateOnEndStep() {

    if (actualEntryPoint.getValue() >= int(entryPoints.getValue().size()))
        return;

    if (idleStep > numIdleSteps.getValue()) {

        std::cout << "Removing " << needleConstraintToDelete->getName() << " from the scene" << std::endl;

        sofa::core::objectmodel::BaseContext* needleConstraintContext = needleConstraintToDelete->getContext();

        std::cout << "Deleting " << needleConstraintToDelete->getName() << " from " << needleConstraintContext->getName() << std::endl;
        needleConstraintContext->removeObject(needleConstraintToDelete);
        //needleConstraintToDelete->terminate();

        sendSimulationData();

        std::cout << "Reinitializing the simulation " << std::endl;

        simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
        simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());

        /// find all mechanical states and set position = rest_position = reset_position, zero velocities and applied forces
        actualEntryPoint.setValue(actualEntryPoint.getValue()+1);

        /// reinit
        simulation::getSimulation()->reset(root);
        simulation::getSimulation()->init(root);
    }
}

template<class DataTypes>
void SimulationPlanningController<DataTypes>::handleEvent(core::objectmodel::Event *event)
{
    //std::cout << this->getName() << ": handle event " << std::endl;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        //std::cout << this->getName() << ": Begin animate event" << std::endl;
        updateOnBeginStep();
    }

    if (dynamic_cast<sofa::simulation::AnimateEndEvent *>(event))
    {
        //std::cout << this->getName() << ": End animate event" << std::endl;
        updateOnEndStep();
    }

    if (dynamic_cast<sofa::simulation::NeedleEvent<Real> *>(event))
    {        
        /*std::cout << this->getName() << ": NeedleEvent caught, pausing the simulation" << std::endl;

        std::cout << "Firing needle event, pausing the simulation" << std::endl;
        sofa::core::ExecParams params;
        sofa::simulation::PauseEvent ev;
        sofa::simulation::PropagateEventVisitor act ( &params, &ev );

        simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext());
        simulation::Node *root = dynamic_cast<simulation::Node *>(context->getRootContext());

        root->execute ( act );*/

        if (idleStep < 0) {
            sofa::simulation::NeedleEvent<Real>* ev = dynamic_cast<sofa::simulation::NeedleEvent<Real> *>(event);
            needleConstraintToDelete = ev->needleConstraints;
            eventType = ev->type;
            idleStep=0;
        }
        //std::cout << "Needle constraint " << needleConstraintToDelete->getName() << " should be deleted" << std::endl;
    }


}


template<class DataTypes>
void SimulationPlanningController<DataTypes>::applyController() {
    std::cout << this->getName() << ": applyController " << std::endl;

}


}

}

}


#endif
