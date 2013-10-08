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
#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/common/PrintVisitor.h>
#include <sofa/simulation/common/FindByTypeVisitor.h>
#include <sofa/simulation/common/ExportGnuplotVisitor.h>
#include <sofa/simulation/common/InitVisitor.h>
#include <sofa/simulation/common/AnimateVisitor.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/CollisionVisitor.h>
#include <sofa/simulation/common/CollisionBeginEvent.h>
#include <sofa/simulation/common/CollisionEndEvent.h>
#include <sofa/simulation/common/UpdateContextVisitor.h>
#include <sofa/simulation/common/UpdateMappingVisitor.h>
#include <sofa/simulation/common/ResetVisitor.h>
#include <sofa/simulation/common/VisualVisitor.h>
#include <sofa/simulation/common/ExportOBJVisitor.h>
#include <sofa/simulation/common/WriteStateVisitor.h>
#include <sofa/simulation/common/XMLPrintVisitor.h>
#include <sofa/simulation/common/PropagateEventVisitor.h>
#include <sofa/simulation/common/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/UpdateMappingEndEvent.h>
#include <sofa/simulation/common/CleanupVisitor.h>
#include <sofa/simulation/common/DeleteVisitor.h>
#include <sofa/simulation/common/UpdateBoundingBoxVisitor.h>
#include <sofa/simulation/common/xml/NodeElement.h>

#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/PipeProcess.h>
#include <sofa/helper/AdvancedTimer.h>

#include <sofa/core/visual/VisualParams.h>

#include <stdlib.h>
#include <math.h>
#include <algorithm>

#include "VerdandiAnimationLoop.h"

namespace sofa
{

namespace simulation
{

SOFA_DECL_CLASS(VerdandiAnimationLoop)

int VerdandiAnimationLoopClass = core::RegisterObject("The simplest animation loop, created by default when user do not put on scene")
        .add< VerdandiAnimationLoop >()
        ;

VerdandiAnimationLoop::VerdandiAnimationLoop(simulation::Node* _gnode)
    : Inherit()
    , gnode(_gnode)
    , _configFile( initData(&_configFile, "configFile", "configuration file for the driver") )
    , _positionInState( initData(&_positionInState, true, "positionInState", "position included in Verdandi state") )
    , _velocityInState( initData(&_velocityInState, true, "velocityInState", "velocity included in Verdandi state") )
{    
    assert(gnode);
    driver=new Verdandi::ForwardDriver<SofaModelWrapper<double> >;
    //modelWrapper = &driver.GetModel();
}

VerdandiAnimationLoop::~VerdandiAnimationLoop()
{

}

void VerdandiAnimationLoop::init() {

}

void VerdandiAnimationLoop::bwdInit()
{
    std::cout << "initialize with: " << _configFile.getValue() << std::endl;
    driver->GetModel().initSimuData(gnode, _positionInState.getValue(), _velocityInState.getValue());
    driver->Initialize(_configFile.getValue());

    if (!gnode)
        gnode = dynamic_cast<simulation::Node*>(this->getContext());

    //this->getContext()->get(modelWrapper);


    /*if (modelWrapper == NULL) {
        std::cerr << "No model wrapper found, cannot interface Verdandi-Sofa" << std::endl;
    } else {
        std::cout << "[" << this->getName() << "]: model wrapper found: " << modelWrapper->getName() << std::endl;
        modelWrapper->setNode(gnode);
    }*/
}

void VerdandiAnimationLoop::setNode(simulation::Node* _gnode)
{
    gnode=_gnode;
}

void VerdandiAnimationLoop::step(const core::ExecParams* params, double /*dt*/)
{    
    driver->GetModel().setInitStepData(params, _positionInState.getValue(), _velocityInState.getValue());
    driver->InitializeStep();
    driver->Forward();
    driver->FinalizeStep();
    /*if (dt == 0)
        dt = this->gnode->getDt();

    sofa::helper::AdvancedTimer::stepBegin("AnimationStep");

    sofa::helper::AdvancedTimer::begin("Animate");

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    {
        AnimateBeginEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        gnode->execute ( act );
    }

    double startTime = gnode->getTime();

    BehaviorUpdatePositionVisitor beh(params , dt);
    gnode->execute ( beh );

    AnimateVisitor act(params, dt);
    gnode->execute ( act );

    gnode->setTime ( startTime + dt );
    gnode->execute< UpdateSimulationContextVisitor >(params);

    {
        AnimateEndEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        gnode->execute ( act );
    }

    sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    gnode->execute< UpdateMappingVisitor >(params);
    sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        UpdateMappingEndEvent ev ( dt );
        PropagateEventVisitor act ( params , &ev );
        gnode->execute ( act );
    }
    sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");

#ifndef SOFA_NO_UPDATE_BBOX
    sofa::helper::AdvancedTimer::stepBegin("UpdateBBox");
    gnode->execute< UpdateBoundingBoxVisitor >(params);
    sofa::helper::AdvancedTimer::stepEnd("UpdateBBox");
#endif
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("Step");
#endif

    ///////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////

    sofa::helper::AdvancedTimer::end("Animate");
    sofa::helper::AdvancedTimer::stepEnd("AnimationStep");*/
}


} // namespace simulation

} // namespace sofa



