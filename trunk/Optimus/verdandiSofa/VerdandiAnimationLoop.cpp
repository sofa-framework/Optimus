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

#include <sofa/component/forcefield/TetrahedronFEMForceField.h>

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
    //, _configFile( initData(&_configFile, "configFile", "configuration file for the driver") )
    , _verbose( initData(&_verbose, false, "verbose", "print out traces") )
{
    assert(gnode);
    numStep=0;    
    filterType=UNDEF;
}

VerdandiAnimationLoop::~VerdandiAnimationLoop()
{

}


void VerdandiAnimationLoop::init() {
    //std::cout << "Filter type: " << _filterType.getValue() << std::endl;

    if (!gnode)
        gnode = dynamic_cast<simulation::Node*>(this->getContext());

    /// NEW INTERFACE:
    gnode->get(roukfDriver, core::objectmodel::BaseContext::SearchDown);

    gnode->get(roukfDriverParallel, core::objectmodel::BaseContext::SearchDown);

    if (roukfDriver) {
        std::cout << this->getName() << "ROUKF driver " << roukfDriver->getName() << " found." << std::endl;
        filterType = ROUKF;
    } else if (roukfDriverParallel) {
        std::cout << this->getName() << "ROUKF driver parallel " << roukfDriverParallel->getName() << " found." << std::endl;
        filterType = ROUKF;

    } else {
        std::cerr << this->getName() << "Warning: ROUKF driver not found! "<< std::endl;
    }

    if (filterType == UNDEF) {
        std::cerr << this->getName() << "ERROR: no filter defined, idle animation loop!" << std::endl;
    }

    char cmd[100];
    sprintf(cmd, "mkdir -p output");
    system(cmd);

}


void VerdandiAnimationLoop::bwdInit()
{    
    if (roukfDriver)
        roukfDriver->InitializeFilter();

    if (roukfDriverParallel)
        roukfDriverParallel->InitializeFilter();


    /*switch (filterType) {
    case FORWARD: fwdDriver->Initialize(_configFile.getValue()); break;
    case UKF: ukfDriver->Initialize(_configFile.getValue()); break;
    case ROUKF: roukfDriver->InitializeFilter(roukfParams); break;
    case UNDEF: break;
    }*/
}

void VerdandiAnimationLoop::setNode(simulation::Node* _gnode)
{
    gnode=_gnode;
}

void VerdandiAnimationLoop::step(const core::ExecParams* params, double /*dt*/)
{
    std::cout << "================== " << " step: " << ++numStep << " ===================" << std::endl;
   /* switch (filterType) {
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

        break;
    case UNDEF: break;
    }*/


    if (roukfDriver) {
        roukfDriver->GetModel().setInitStepData(params);
        roukfDriver->InitializeStep();
        roukfDriver->Forward();
        roukfDriver->Analyze();
        roukfDriver->FinalizeStep();
    }

    if (roukfDriverParallel) {
        std::cout<<"XXX:verdandianimationloop:: step 1:\n";
        roukfDriverParallel->GetModel().setInitStepData(params);
        std::cout<<"XXX:verdandianimationloop:: step 2:\n";
        roukfDriverParallel->InitializeStep();
        std::cout<<"XXX:verdandianimationloop:: step 3:\n";
        roukfDriverParallel->Forward();
        std::cout<<"XXX:verdandianimationloop:: step 4:\n";
        roukfDriverParallel->Analyze();
        std::cout<<"XXX:verdandianimationloop:: step 5:\n";
        roukfDriverParallel->FinalizeStep();
    }

}


} // namespace simulation

} // namespace sofa





