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

//#define PRESTOCHASTICWRAPPER_CPP

#include <sofa/simulation/PrintVisitor.h>
#include <SofaSimulationCommon/FindByTypeVisitor.h>
#include <sofa/simulation/ExportGnuplotVisitor.h>
#include <sofa/simulation/InitVisitor.h>
#include <sofa/simulation/AnimateVisitor.h>
#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/simulation/CollisionVisitor.h>
#include <sofa/simulation/CollisionBeginEvent.h>
#include <sofa/simulation/CollisionEndEvent.h>
#include <sofa/simulation/UpdateContextVisitor.h>
#include <sofa/simulation/UpdateMappingVisitor.h>
#include <sofa/simulation/ResetVisitor.h>
#include <sofa/simulation/VisualVisitor.h>
#include <sofa/simulation/ExportOBJVisitor.h>
#include <sofa/simulation/WriteStateVisitor.h>
#include <sofa/simulation/XMLPrintVisitor.h>
#include <sofa/simulation/PropagateEventVisitor.h>
#include <sofa/simulation/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/UpdateInternalDataVisitor.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/UpdateMappingEndEvent.h>
#include <sofa/simulation/CleanupVisitor.h>
#include <sofa/simulation/DeleteVisitor.h>
#include <sofa/simulation/UpdateBoundingBoxVisitor.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/PipeProcess.h>
#include <sofa/helper/AdvancedTimer.h>

#include <sofa/core/ObjectFactory.h>
#include "PreStochasticWrapper.h"
//#include <sofa/helper/accessor.h>

namespace sofa
{

namespace component
{

namespace stochastic
{

using namespace defaulttype;

SOFA_DECL_CLASS(PreStochasticWrapper)

// Register in the Factory
int PreStochasticWrapperClass = core::RegisterObject("PreStochasticWrapper")       
        .add< PreStochasticWrapper>()
        ;

PreStochasticWrapper::PreStochasticWrapper()
    : verbose( initData(&verbose, false, "verbose", "print tracing informations") )
{
}

void PreStochasticWrapper::init() {
    gnode = dynamic_cast<sofa::simulation::Node*>(this->getContext());

    if (!gnode) {
        PRNE("Cannot find node!");
    }
}

void PreStochasticWrapper::step(const core::ExecParams* _params, const size_t _step) {

    sofa::helper::AdvancedTimer::stepBegin("PreStochasticWrapper");
    PRNS(" calling step in " << _step);

    sofa::helper::AdvancedTimer::stepBegin("AnimationStep");
    //std::cout<<"step "<<step++<<std::endl;
    //sofa::helper::AdvancedTimer::begin("Animate");
    //std::cout<<"step "<<step++<<std::endl;

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    double    dt = this->gnode->getDt();
    //core::ExecParams* _params = sofa::core::ExecParams::defaultInstance();

    //std::cout << "[" << this->getName() << "]: step default begin at time = " << gnode->getTime() << " update time: " << _update_time << std::endl;

    //sofa::helper::AdvancedTimer::stepBegin("AnimationStep");
    //std::cout<<"step "<<step++<<std::endl;
    //sofa::helper::AdvancedTimer::begin("Animate");
    //std::cout<<"step "<<step++<<std::endl;

//#ifdef SOFA_DUMP_VISITOR_INFO
//    simulation::Visitor::printNode("Step");
//#endif

    {
        //std::cout<<"step "<<step++<<std::endl;
        //std::cout << "[" << this->getName() << "]: animate begin" << std::endl;
        sofa::simulation::AnimateBeginEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act ( _params, &ev );
        this->gnode->execute ( act );
        //std::cout<<"step "<<step++<<std::endl;
    }

    double startTime = this->gnode->getTime();
    //std::cout<<"step "<<step++<<std::endl;
    //std::cout << "[" << this->getName() << "]: behaviour update position" << std::endl;
    sofa::simulation::BehaviorUpdatePositionVisitor beh(_params , dt);
    this->gnode->execute ( beh );
    //std::cout<<"step "<<step++<<std::endl;
    //std::cout << "[" << this->getName() << "]: update internal data" << std::endl;
    sofa::simulation::UpdateInternalDataVisitor uid(_params);
    this->gnode->execute ( uid );
    //std::cout<<"step "<<step++<<std::endl;
    //std::cout << "[" << this->getName() << "]: animate" << std::endl;
    sofa::simulation::AnimateVisitor act(_params, dt);
    this->gnode->execute ( act );

    //if (_updateTime) {
        //std::cout << "[" << this->getName() << "]: update simulation context" << std::endl;
        this->gnode->setTime ( startTime + dt );
        this->gnode->execute<  sofa::simulation::UpdateSimulationContextVisitor >(_params);
    //}
    //std::cout<<"step "<<step++<<std::endl;
    {
        //std::cout << "[" << this->getName() << "]: animate end" << std::endl;
        sofa::simulation::AnimateEndEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act (_params, &ev );
        this->gnode->execute ( act );
    }

    sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    //std::cout << "[" << this->getName() << "]: update mapping" << std::endl;
    this->gnode->execute<  sofa::simulation::UpdateMappingVisitor >(_params);
    //sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        //std::cout << "[" << this->getName() << "]: update mapping end" << std::endl;
        sofa::simulation::UpdateMappingEndEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act ( _params , &ev );
        this->gnode->execute ( act );
    }
    sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");

#ifndef SOFA_NO_UPDATE_BBOX
    sofa::helper::AdvancedTimer::stepBegin("UpdateBBox");
    this->gnode->execute<  sofa::simulation::UpdateBoundingBoxVisitor >(_params);
    sofa::helper::AdvancedTimer::stepEnd("UpdateBBox");
#endif
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("Step");
#endif
    //sofa::helper::AdvancedTimer::end("Animate");
    sofa::helper::AdvancedTimer::stepEnd("AnimationStep");

    sofa::helper::AdvancedTimer::stepEnd("PreStochasticWrapper");
}

} // namespace stochastic

} // namespace component

} // namespace sofa

