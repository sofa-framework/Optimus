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
    , _filterType( initData(&_filterType, "filterType", "type of filter [forward | UKF]", "forward") )
    , _stateErrorVarianceState( initData(&_stateErrorVarianceState, 1.0 , "stateErrorVariance", "state error variance value") )
    , _stateErrorVarianceParams( initData(&_stateErrorVarianceParams, 1.0 , "paramErrorVariance", "parameters error variance value") )
    , _verbose( initData(&_verbose, false, "verbose", "print out traces") )
{
    assert(gnode);
    numStep=0;    
}

VerdandiAnimationLoop::~VerdandiAnimationLoop()
{

}

void VerdandiAnimationLoop::init() {
    std::cout << "Filter type: " << _filterType.getValue() << std::endl;

    if (!gnode)
        gnode = dynamic_cast<simulation::Node*>(this->getContext());

    //fixedConstraints = gnode->get<sofa::component::projectiveconstraintset::FixedConstraint<sofa::defaulttype::Vec3dTypes>*>();

    SofaModelWrapper<double>::ModelData md;
    md.gnode = gnode;
    md.errorVarianceSofaState = _stateErrorVarianceState.getValue();
    md.errorVarianceSofaParams  = _stateErrorVarianceParams.getValue();
    md.positionInState = _positionInState.getValue();
    md.velocityInState = _velocityInState.getValue();

    if (_filterType.getValue() == "forward") {        
        filterType = md.filterType = FORWARD;
        fwdDriver = new Verdandi::ForwardDriver<SofaModelWrapper<double> >;        
        fwdDriver->GetModel().initSimuData(md);
    }
    else if (_filterType.getValue() == "UKF") {        
        filterType = md.filterType = UKF;
        ukfDriver = new Verdandi::UnscentedKalmanFilter<SofaModelWrapper<double>, Verdandi::LinearObservationManager<double> >;        
        ukfDriver->GetModel().initSimuData(md);
    } else if (_filterType.getValue() == "ROUKF") {
        filterType = md.filterType = ROUKF;
        roukfDriver = new Verdandi::ReducedOrderUnscentedKalmanFilter<SofaModelWrapper<double>, Verdandi::LinearObservationManager<double> >;
        roukfDriver->GetModel().initSimuData(md);
    }
    else
        filterType = UNDEF;  

}

void VerdandiAnimationLoop::bwdInit()
{
    //std::cout << "initialize with: " << _configFile.getValue() << std::endl;
    switch (filterType) {
    case FORWARD: fwdDriver->Initialize(_configFile.getValue()); break;
    case UKF: ukfDriver->Initialize(_configFile.getValue()); break;
    case ROUKF: roukfDriver->Initialize(_configFile.getValue()); break;
    case UNDEF: break;
    }
}

void VerdandiAnimationLoop::setNode(simulation::Node* _gnode)
{
    gnode=_gnode;
}

void VerdandiAnimationLoop::step(const core::ExecParams* params, double /*dt*/)
{    
    std::cout << "================== " << filterType << " step: " << ++numStep << " ===================" << std::endl;
    switch (filterType) {
    case FORWARD:
        fwdDriver->GetModel().setInitStepData(params);
        fwdDriver->InitializeStep();
        fwdDriver->Forward();
        fwdDriver->FinalizeStep();
        break;
    case UKF:
        ukfDriver->GetModel().setInitStepData(params);
        ukfDriver->InitializeStep();
        ukfDriver->Forward();
        ukfDriver->Analyze();
        ukfDriver->FinalizeStep();
        break;
    case ROUKF:
        roukfDriver->GetModel().setInitStepData(params);
        roukfDriver->InitializeStep();
        roukfDriver->Forward();
        roukfDriver->Analyze();
        roukfDriver->FinalizeStep();
        break;
    case UNDEF: break;
    }
}


} // namespace simulation

} // namespace sofa



