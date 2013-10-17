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
    , dim_(3)
    , state_size_(0)
    , reduced_state_size_(0)
    , reduced_state_index_(0)
    , freeIndices(0)
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
void SofaModelWrapper<Type>::initSimuData(ModelData &_md)
{
    /// set variables given by animation loop
    Verb("initSimuData");

    modelData = _md;
    simulation::Node* gnode = modelData.gnode;

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

    gnode->get(fixedConstraints);
    if (fixedConstraints != NULL)
        std::cout << "FixedConstraints found " << fixedConstraints->getName() << std::endl;
    else
        std::cerr << "Fixed constraints not found! " << std::endl;

    std::cout << "Number of fixed constraints: " << fixedConstraints->f_indices.getValue().size() << std::endl;

    numStep = 0;
}

template <class Type>
void SofaModelWrapper<Type>::StateSofa2Verdandi() {
    typename MechStateVec3d::ReadVecCoord pos = mechanicalObject->readPositions();
    typename MechStateVec3d::ReadVecDeriv vel = mechanicalObject->readVelocities();

    size_t j = 0;
    if (modelData.positionInState) {
        for (size_t i = 0; i < free_nodes_size; i++)
            for (size_t d = 0; d < dim_; d++)
                state_(j++) = pos[freeIndices[i]][d];
    }

    if (modelData.velocityInState) {
        for (size_t i = 0; i < free_nodes_size; i++)
            for (size_t d = 0; d < dim_; d++)
                state_(j++) = vel[freeIndices[i]][d];
    }

    const helper::vector<double>& vecPar = vecParams->getValue();
    for (size_t i = 0; i < vecParams->size(); i++)
        state_(j++) = vecPar[i];
}

template <class Type>
void SofaModelWrapper<Type>::StateVerdandi2Sofa() {
    typename MechStateVec3d::WriteVecCoord pos = mechanicalObject->writePositions();
    typename MechStateVec3d::WriteVecDeriv vel = mechanicalObject->writeVelocities();

    size_t j = 0;
    if (modelData.positionInState) {
        for (size_t i = 0; i < free_nodes_size; i++)
            for (size_t d = 0;  d < dim_; d++)
                pos[freeIndices[i]][d] = state_(j++);
    }

    if (modelData.velocityInState) {
        for (size_t i = 0; i < free_nodes_size; i++)
            for (size_t d = 0;  d < dim_; d++)
                vel[freeIndices[i]][d] = state_(j++);
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
    /// get fixed nodes
    const FixedConstraintVec3d::SetIndexArray& fixedIndices = fixedConstraints->f_indices.getValue();

    freeIndices.clear();
    for (int i = 0; i < mechanicalObject->getSize(); i++) {
        bool found = false;
        for (size_t j = 0; j < fixedIndices.size(); j++) {
            if (int(fixedIndices[j]) == i) {
                found = true;
                break;
            }
        }
        if (!found)
            freeIndices.push_back(i);
    }
    free_nodes_size = freeIndices.size();

    /// initialize filter state                
    state_size_ = 0;
    if (modelData.positionInState)
        state_size_ += dim_* free_nodes_size;

    if (modelData.velocityInState)
        state_size_ += dim_* free_nodes_size;

    reduced_state_index_ = state_size_;

    if (vecParams) {
        reduced_state_size_ = vecParams->size();
        state_size_ += reduced_state_size_;
    }

    std::cout << "Initializing model (filter type " << modelData.filterType << ") with size " << state_size_ << std::endl;
    std::cout << "Reduced state index: " << reduced_state_index_ << " size: " << reduced_state_size_ << std::endl;

    state_.Nullify();
    state_.Resize(state_size_);

    StateSofa2Verdandi();


    if (modelData.filterType == UKF) {
        /// initialize state error variance
        state_error_variance_.Reallocate(GetNstate(), GetNstate());
        state_error_variance_.SetIdentity();
        for (size_t i = 0; i < size_t(reduced_state_index_); i++)
            state_error_variance_(i,i) *= modelData.errorVarianceSofaState;

        for (size_t i = reduced_state_index_; i < size_t(state_size_); i++)
            state_error_variance_(i,i) *= modelData.errorVarianceSofaParams;

        //Mlt(Type(state_error_variance_value_), state_error_variance_);

        state_error_variance_inverse_.Reallocate(GetNstate(), GetNstate());
        state_error_variance_inverse_.SetIdentity();
        for (size_t i = 0; i < size_t(reduced_state_index_); i++)
            state_error_variance_inverse_(i,i) *= modelData.errorVarianceSofaState;
        for (size_t i = size_t(reduced_state_index_); i < size_t(state_size_); i++)
            state_error_variance_inverse_(i,i) *= modelData.errorVarianceSofaParams;
    }

    if (modelData.filterType == ROUKF) {
        variance_projector_allocated_ = false;
        variance_reduced_allocated_ = false;
    }
}

template <class Type>
void SofaModelWrapper<Type>::FinalizeStep() {    
    std::cout << "Actual parameter values:";
    for (size_t i = reduced_state_index_; i < state_size_; i++)
        std::cout << " " << state_(i);
    std::cout << std::endl;
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
    if (modelData.verbose)
        std::cout << this->getName() << " :state updated " << std::endl;
    for (int i = 0; i < state_.GetM(); i++)
        state_(i) = duplicated_state_(i);
    StateVerdandi2Sofa();
}

template <class Type>
void SofaModelWrapper<Type>::SetTime(double _time) {
    time_ = _time;
    simulation::Node* gnode = modelData.gnode;
    gnode->setTime ( _time );
    gnode->execute< UpdateSimulationContextVisitor >(execParams);
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

    Forward(_update_force, false);
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
void SofaModelWrapper<Type>::Forward(bool _update_force, bool _update_time)
{
    Verb("forward begin");
    if (_update_force) {

    }
    simulation::Node* gnode = modelData.gnode;

    double    dt = gnode->getDt();

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

    if (_update_time) {
        gnode->setTime ( startTime + dt );
        gnode->execute< UpdateSimulationContextVisitor >(execParams);
    }

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

template <class Type>
typename SofaModelWrapper<Type>::state_error_variance& SofaModelWrapper<Type>::GetStateErrorVarianceProjector() {
    //std::cout << this->getName() << " " << GetTime() << " getStateErrorVarianceProjector" << std::endl;
    std::cout << "GetSEV_PROJECTOR" << std::endl;
    if (!variance_projector_allocated_)
    {
        /*int Nreduced = 0;
        for (unsigned int i = 0; i < reduced_.size(); i++)
            Nreduced += x_.GetVector(reduced_[i]).GetSize();
        std::cout << "  PP-Nreduced: " << Nreduced << std::endl;
        // Initializes L.
        std::cout << "  P-reducedSize: " << reduced_.size() << std::endl;*/
        state_error_variance_projector_.Reallocate(state_size_, reduced_state_size_);
        state_error_variance_projector_.Fill(Type(0.0));
        //for (size_t i = 0, l = 0; i < reduced_state_size_; i++) {
        for (size_t i = 0, l = 0; i < 1; i++) {   /// only one reduced state
            for(size_t k = reduced_state_index_; k < reduced_state_index_ + reduced_state_size_; k++) {
                std::cout << "k,l = " << k << " " << l << std::endl;
                state_error_variance_projector_(k, l++) = 1;
            }
        }
        std::cout << "  Initialize L: " << std::endl;
        variance_projector_allocated_ = true;
    }
    //std::cout << "  L = " << state_error_variance_projector_ << std::endl;
    return state_error_variance_projector_;
}

template <class Type>
typename SofaModelWrapper<Type>::state_error_variance& SofaModelWrapper<Type>::GetStateErrorVarianceReduced() {
    //std::cout << this->getName() << " " << GetTime() << " getStateErrorVarianceReduced" << std::endl;
    //std::cout << "GetSEV_REDUCED" << std::endl;
    if (!variance_reduced_allocated_)
    {
        /*int Nreduced = 0;
        for (unsigned int i = 0; i < reduced_.size(); i++)
            Nreduced += x_.GetVector(reduced_[i]).GetSize();
        std::cout << "  R-Nreduced: " << Nreduced << std::endl;
        // Initializes U.*/
        state_error_variance_reduced_.Reallocate(reduced_state_size_,  reduced_state_size_);
        state_error_variance_reduced_.Fill(Type(0.0));
        for (size_t i = 0; i < reduced_state_size_; i++)
            state_error_variance_reduced_(i, i) = Type(Type(1.0) / modelData.errorVarianceSofaParams);
        std::cout << "  Initialize U: " << std::endl;
        variance_reduced_allocated_ = true;
    }
    //std::cout << "U = " << state_error_variance_reduced_ << std::endl;
    return state_error_variance_reduced_;
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
