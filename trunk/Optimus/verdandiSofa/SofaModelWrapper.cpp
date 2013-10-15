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
    : Inherit()
    , current_row_(-1)
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
void SofaModelWrapper<Type>::initSimuData(simulation::Node* _gnode, bool _posInState, bool _velInState, double _stateErrorVarianceValue , bool _verbose)
{
    /// set variables given by animation loop
    Verb("initSimuData");

    gnode=_gnode;
    positionInState = _posInState;
    velocityInState = _velInState;
    state_error_variance_value_ = _stateErrorVarianceValue;
    verbose = _verbose;

    /// register the object in the scene
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

    numStep = 0;
}

template <class Type>
void SofaModelWrapper<Type>::StateSofa2Verdandi() {
    typename core::behavior::MechanicalState<defaulttype::Vec3dTypes>::ReadVecCoord pos = mechanicalObject->readPositions();
    typename core::behavior::MechanicalState<defaulttype::Vec3dTypes>::ReadVecDeriv vel = mechanicalObject->readVelocities();

    size_t j = 0;
    if (positionInState) {
        for (size_t i = 4; i < pos.size(); i++)
            for (size_t d = 0; d < 3; d++) {
                state_(j++) = pos[i][d];
            }
    }

    if (velocityInState) {
        for (size_t i = 4; i < vel.size(); i++)
            for (size_t d = 0; d < 3; d++)
                state_(j++) = vel[i][d];
    }

    const helper::vector<double>& vecPar = vecParams->getValue();
    for (size_t i = 0; i < vecParams->size(); i++)
        state_(j++) = vecPar[i];
}

template <class Type>
void SofaModelWrapper<Type>::StateVerdandi2Sofa() {
    typename core::behavior::MechanicalState<defaulttype::Vec3dTypes>::WriteVecCoord pos = mechanicalObject->writePositions();
    typename core::behavior::MechanicalState<defaulttype::Vec3dTypes>::WriteVecDeriv vel = mechanicalObject->writeVelocities();

    size_t j = 0;
    if (positionInState) {
        for (size_t i = 4; i < pos.size(); i++)
            for (size_t d = 0; d < 3; d++) {
                pos[i][d] = state_(j++);
            }
    }

    if (velocityInState) {
        for (size_t i = 4; i < vel.size(); i++)
            for (size_t d = 0; d < 3; d++)
                vel[i][d] = state_(j++);
    }

    helper::vector<double> vecPar(vecParams->size());
    for (size_t i = 0; i < vecParams->size(); i++)
        vecPar[i] = state_(j++);
    vecParams->setValue(vecPar);
}


template <class Type>
void SofaModelWrapper<Type>::Initialize(std::string &/*configFile*/)
{
    Verb("initialize");
    /// initialize filter state
    state_size_ = 0;
    if (positionInState)
        state_size_ += 3*(mechanicalObject->getSize()-4);

    if (velocityInState)
        state_size_ += 3*(mechanicalObject->getSize()-4);

    std::cout << "Initializing with size " << state_size_ << std::endl;

    if (vecParams)
        state_size_ += vecParams->size();

    std::cout << "SSS "  << vecParams->size() << std::endl;
    std::cout << "Initializing with size " << state_size_ << std::endl;

    state_.Nullify();
    state_.Resize(state_size_);

    StateSofa2Verdandi();

    /// initialize state error variance
    state_error_variance_.Reallocate(GetNstate(), GetNstate());
    state_error_variance_.SetIdentity();
    Mlt(Type(state_error_variance_value_), state_error_variance_);
    state_error_variance_inverse_.Reallocate(GetNstate(), GetNstate());
    state_error_variance_inverse_.SetIdentity();
    Mlt(Type(state_error_variance_value_), state_error_variance_inverse_);
}

template <class Type>
void SofaModelWrapper<Type>::FinalizeStep() {
    /*if ((numStep%200) == 0) {
        state& temp = GetState();
        int sz = temp.GetSize();
        double x = temp(sz-1);
        temp(sz-1) = temp(sz-2);
        temp(sz-2) = x;
        StateUpdated();
    }*/
}


template <class Type>
typename SofaModelWrapper<Type>::state& SofaModelWrapper<Type>::GetState() {
    /// propagate the SOFA state towards verdandi state
    StateSofa2Verdandi();

    /// return a reference to duplicate state
    duplicated_state_.Reallocate(state_.GetM());
    for (int i = 0; i < state_.GetM(); i++)
        duplicated_state_(i) = state_(i);
    return duplicated_state_;

}

template <class Type>
void SofaModelWrapper<Type>::StateUpdated() {
    if (verbose)
        std::cout << this->getName() << " :state updated " << std::endl;
    for (int i = 0; i < state_.GetM(); i++)
        state_(i) = duplicated_state_(i);
    StateVerdandi2Sofa();
}

template <class Type>
void SofaModelWrapper<Type>::GetStateCopy(state& _copy) {
    Verb("get state copy");
    _copy.Reallocate(state_.GetM());
    for (int i = 0; i < state_.GetM(); i++)
        _copy(i) = state_(i);
}


template <class Type>
double SofaModelWrapper<Type>::ApplyOperator(state& _x, bool _preserve_state, bool _update_force)  {
    Verb("state updated begin");
    /*std::cout << "Apply operator on ";
    for (size_t i = 0; i < _x.GetSize(); i++)
        std::cout << _x(i) << " ";
    std::cout << std::endl;*/

    double saved_time = 0;
    state saved_state;
    saved_time = GetTime();

    if (_preserve_state)
        saved_state.SetData(duplicated_state_);

    duplicated_state_.Nullify();
    duplicated_state_.SetData(_x);
    StateUpdated();

    Forward(_update_force);
    StateSofa2Verdandi();
    double new_time = GetTime();

    GetStateCopy(duplicated_state_);

    duplicated_state_.Nullify();

    SetTime(saved_time);

    /*std::cout << "end _x ";
    for (size_t i = 0; i < _x.GetSize(); i++)
        std::cout << _x(i) << " ";
    std::cout << std::endl;*/

    if (_preserve_state)
    {
        duplicated_state_.SetData(saved_state);
        StateUpdated();
        saved_state.Nullify();
    }

    Verb("state updated begin end");
    return new_time;
}


template <class Type>
void SofaModelWrapper<Type>::Forward(bool _update_force)
{
    Verb("forward begin");
    if (_update_force) {

    }

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

    Verb("forward end");
}


template <class Type>
typename SofaModelWrapper<Type>::state_error_variance& SofaModelWrapper<Type>::GetStateErrorVariance() {
    std::cout << this->getName() << " " << GetTime() << " getStateErrorVariance" << std::endl;
    return state_error_variance_;
}


template <class Type>
typename SofaModelWrapper<Type>::state_error_variance_row& SofaModelWrapper<Type>::GetStateErrorVarianceRow(int row)
{
    if (row == current_row_)
        return state_error_variance_row_;

    GetRow(state_error_variance_, row, state_error_variance_row_);
    current_row_ = row;

    return state_error_variance_row_;
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
