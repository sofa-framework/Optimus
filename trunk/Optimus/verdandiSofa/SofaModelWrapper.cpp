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

#include "SofaModelWrapper.h"

namespace sofa
{

namespace simulation
{

template <class Type>
SofaModelWrapper<Type>::SofaModelWrapper()
    //: Inherit()
{}

template <class Type>
SofaModelWrapper<Type>::~SofaModelWrapper()
{}

template <class Type>
void SofaModelWrapper<Type>::Message(string _message) {
    std::cout << "Message: " << _message << std::endl;
    if (_message.find("initial condition") != string::npos || _message.find("forecast") != string::npos) {
        //Save();
    }
}


template <class Type>
void SofaModelWrapper<Type>::initSimuData( simulation::Node* _gnode, bool _posInState, bool _velInState )
{
    std::cout << "sofa init " << std::endl;
    gnode=_gnode;
    positionInState = _posInState;
    velocityInState = _velInState;
    std::cout << "Registering object: " << this->GetName() << std::endl;
    this->setName("sofaModelWrapper");
    gnode->addObject(this);

    /// looking for optimization parameters which will be a part of the state    
    gnode->get(vecParams);

    if (vecParams != NULL) {
        std::cout << "Vectorial parameter container found " << vecParams->getName() << ", size: " << vecParams->size() << std::endl;
    } else
        std::cerr << "Vectorial parameter not found! " << std::endl;

    gnode->get(mechanicalObject);

    if (mechanicalObject != NULL)
        std::cout << "Mechanical object found " << mechanicalObject->getName() << std::endl;
    else
        std::cerr << "Mechanical object not found! " << std::endl;
}

template <class Type>
typename SofaModelWrapper<Type>::state& SofaModelWrapper<Type>::GetState() {
    typename core::behavior::MechanicalState<defaulttype::Vec3dTypes>::ReadVecCoord pos = mechanicalObject->readPositions();
    typename core::behavior::MechanicalState<defaulttype::Vec3dTypes>::ReadVecDeriv vel = mechanicalObject->readVelocities();

    size_t size = 0;
    if (positionInState)
        size += 3*pos.size();

    if (velocityInState)
        size += 3*vel.size();



    if (vecParams)
        size += vecParams->size();

    _state.Nullify();
    _state.Resize(size);

    //std::cout << "State having " << size << " members " << std::endl;

    size_t j = 0;
    if (positionInState) {
        for (size_t i = 0; i < pos.size(); i++)
            for (size_t d = 0; d < 3; d++) {
                _state(j++) = pos[i][d];
            }
    }

    if (velocityInState) {
        for (size_t i = 0; i < vel.size(); i++)
            for (size_t d = 0; d < 3; d++)
                _state(j++) = vel[i][d];
    }

    const helper::vector<double>& vecPar = vecParams->getValue();
    for (size_t i = 0; i < vecParams->size(); i++)
        _state(j++) = vecPar[i];

    return _state;
}


template <class Type>
void SofaModelWrapper<Type>::Forward()
{
    double    dt = this->gnode->getDt();

    sofa::helper::AdvancedTimer::stepBegin("AnimationStep");

    sofa::helper::AdvancedTimer::begin("Animate");

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    {
        AnimateBeginEvent ev ( dt );
        PropagateEventVisitor act ( execParams, &ev );
        gnode->execute ( act );
    }

    double startTime = gnode->getTime();

    BehaviorUpdatePositionVisitor beh(execParams , dt);
    gnode->execute ( beh );

    AnimateVisitor act(execParams, dt);
    gnode->execute ( act );

    gnode->setTime ( startTime + dt );
    gnode->execute< UpdateSimulationContextVisitor >(execParams);

    {
        AnimateEndEvent ev ( dt );
        PropagateEventVisitor act ( execParams, &ev );
        gnode->execute ( act );
    }

    sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    gnode->execute< UpdateMappingVisitor >(execParams);
    sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        UpdateMappingEndEvent ev ( dt );
        PropagateEventVisitor act ( execParams , &ev );
        gnode->execute ( act );
    }
    sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");

#ifndef SOFA_NO_UPDATE_BBOX
    sofa::helper::AdvancedTimer::stepBegin("UpdateBBox");
    gnode->execute< UpdateBoundingBoxVisitor >(execParams);
    sofa::helper::AdvancedTimer::stepEnd("UpdateBBox");
#endif
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("Step");
#endif
    sofa::helper::AdvancedTimer::end("Animate");
    sofa::helper::AdvancedTimer::stepEnd("AnimationStep");
}

SOFA_DECL_CLASS(SofaModelWrapper)

int SofaModelWrapperClass = core::RegisterObject("A class implementing an interface between SOFA and verdandi")
#ifndef SOFA_FLOAT
        .add< SofaModelWrapper<double> >()
#endif
/*#ifndef SOFA_DOUBLE
        .add< SofaModelWrapper<float> >()
#endif*/
        ;

template class SofaModelWrapper<double>;


} // namespace simulation

} // namespace sofa
