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

#include <sofa/core/VecId.h>
#include <sofa/simulation/common/MechanicalOperations.h>
#include <sofa/simulation/common/SolveVisitor.h>
#include <sofa/simulation/common/VectorOperations.h>

#include <sofa/simulation/common/IntegrateBeginEvent.h>
#include <sofa/simulation/common/IntegrateEndEvent.h>

#include "SofaModelWrapper.h"
#include "VerdandiClasses.h"

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
{
    displayTime.setValue(false);
    m_solveVelocityConstraintFirst.setValue(false);
    constraintSolver = NULL;
}

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
    gnode->get(constraintSolver, core::objectmodel::BaseContext::SearchDown);
    if (constraintSolver == NULL)
        std::cout << "No ConstraintSolver found, considering the version with no contacts" << std::endl;
    else
        std::cout << "Constraint solver " << constraintSolver->getName() << " found, modeling contacts" << std::endl;

    sofaObjects.clear();
    std::cout << "Searching optim params: " << std::endl;
    helper::vector<OptimParams*> listOP;
    gnode->get<OptimParams>(&listOP, BaseContext::SearchRoot );

    for (size_t iop = 0; iop < listOP.size(); iop++) {
        OptimParams* oparam = listOP[iop];

        if (!oparam->optimize())
            continue;

        simulation::Node* opnode = dynamic_cast<simulation::Node*>(oparam->getContext());

        oparam->mappingIndices.clear();

        bool existingObject = false;
        for (size_t i = 0; i < sofaObjects.size(); i++) {
            if (sofaObjects[i].node == opnode) {
                sofaObjects[i].oparams.push_back(oparam);
                existingObject = true;
                break;
            }
        }

        if (!existingObject) {
            SofaObject obj;
            obj.node = opnode;
            opnode->get(obj.vecMS);
            opnode->get(obj.rigidMS);

            opnode->get(obj.vecFC);
            opnode->get(obj.rigidFC);

            obj.oparams.push_back(oparam);

            if (obj.vecMS == NULL && obj.rigidMS == NULL)
                std::cerr << "PROBLEM: NO MECHANICAL STATE FOUND! " << std::endl;

            sofaObjects.push_back(obj);
        }
    }

    std::cout << this->getName() << " NUMBER of SOFA objects: " << sofaObjects.size() << std::endl;

    numStep = 0;
}

/*template <class Type>
void SofaModelWrapper<Type>::VectorSofaToVerdandi() {

}*/

template<class Type>
typename SofaModelWrapper<Type>::SofaObject* SofaModelWrapper<Type>::getObject(typename SofaModelWrapper<Type>::MechStateVec3d *_state) {
    for (size_t iop = 0; iop < sofaObjects.size(); iop++) {
        if (sofaObjects[iop].vecMS == _state) {
            std::cout << this->getName() << " : found SOFA object " << iop << std::endl;
            return &sofaObjects[iop];
        }
    }
    return(NULL);
}

template<class Type>
void SofaModelWrapper<Type>::SetSofaVectorFromVerdandiState(defaulttype::Vec3dTypes::VecCoord & vec, const state &_state, SofaObject* obj) {    
    vec.clear();
    typename MechStateVec3d::ReadVecCoord pos = obj->vecMS->readPositions();
    vec.resize(pos.size());
    for (size_t i = 0; i < vec.size(); i++) {
        //std::cout << "[" << i << "]: " << pos[i] << std::endl;
        vec[i] = pos[i];
    }

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj->positionPairs.begin(); it != obj->positionPairs.end(); it++)
        for (size_t d = 0; d < dim_; d++) {
            //std::cout << "x[" << it->first << "]: " << vec[it->first] << std::endl;
            vec[it->first][d] = _state(dim_*it->second + d);
        }

}

/// copy the actual SOFA mechanical object and parameter vector to Verdandi state vector using already created index maps
template <class Type>
void SofaModelWrapper<Type>::StateSofa2Verdandi() {
    /// for all SofaObjects (associated with nodes having OptimParams component)
    for (size_t iop = 0; iop < sofaObjects.size(); iop++) {
        SofaObject& obj = sofaObjects[iop];

        if (obj.vecMS != NULL) {
            typename MechStateVec3d::ReadVecCoord pos = obj.vecMS->readPositions();
            typename MechStateVec3d::ReadVecDeriv vel = obj.vecMS->readVelocities();

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    state_(dim_*it->second + d) = pos[it->first][d];

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    state_(dim_*it->second + d) = vel[it->first][d];
        }

        if (obj.rigidMS != NULL) {
            typename MechStateRigid3d::ReadVecCoord pos = obj.rigidMS->readPositions();
            typename MechStateRigid3d::ReadVecDeriv vel = obj.rigidMS->readVelocities();

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                defaulttype::Rigid3dTypes::CPos rpos = defaulttype::Rigid3dTypes::getCPos(pos[it->first]);
                //std::cout << rpos << std::endl;
                for (size_t d = 0; d < dim_; d++)
                    state_(dim_*it->second + d ) = rpos[d];
            }

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++) {
                defaulttype::Rigid3dTypes::DPos rvel = Rigid3dTypes::getDPos(vel[it->first]);
                for (size_t d = 0; d < dim_; d++)
                    state_(dim_*it->second + d ) = rvel[d];

            }
        }

        for (size_t opi = 0; opi < obj.oparams.size(); opi++) {
            OptimParams* oparam = obj.oparams[opi];
            const helper::vector<Type> vecPar = oparam->getValue();

            switch (modelData.transformParams) {
            case 1:
                for (size_t i = 0; i < oparam->mappingIndices.size(); i++)
                    state_(reduced_state_index_ + oparam->mappingIndices[i]) = fabs(vecPar[i]);
                break;
            default:
                for (size_t i = 0; i < oparam->mappingIndices.size(); i++)
                    state_(reduced_state_index_ + oparam->mappingIndices[i]) = vecPar[i];
            }
        }

    }

}

/// copy a Verdandi state to SOFA mechanical object and parameter vector in corresponding OptimParams
template <class Type>
void SofaModelWrapper<Type>::StateVerdandi2Sofa() {
    for (size_t iop = 0; iop < sofaObjects.size(); iop++) {
        SofaObject& obj = sofaObjects[iop];
        std::cout << "Verdandi => Sofa on " << obj.vecMS->getName() <<  std::endl;

        if (obj.vecMS != NULL) {
            typename MechStateVec3d::WriteVecCoord pos = obj.vecMS->writePositions();
            typename MechStateVec3d::WriteVecDeriv vel = obj.vecMS->writeVelocities();

            //size_t ii = 0;
            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                for (size_t d = 0; d < dim_; d++) {
                    pos[it->first][d] = state_(dim_*it->second + d);                    
                }
                /*if (ii < 2)
                    std::cout << pos[it->first] << " ";
                ii++;*/
            }
            //std::cout << std::endl;

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    vel[it->first][d] = state_(dim_*it->second + d);
        }

        if (obj.rigidMS != NULL) {
            typename MechStateRigid3d::WriteVecCoord pos = obj.rigidMS->writePositions();
            typename MechStateRigid3d::WriteVecDeriv vel = obj.rigidMS->writeVelocities();

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++) {
                defaulttype::Rigid3dTypes::CPos rpos(state_(dim_*it->second), state_(dim_*it->second + 1), state_(dim_*it->second + 2));
                defaulttype::Rigid3dTypes::setCPos(pos[it->first], rpos);
                //std::cout << rpos << std::endl;
            }

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.velocityPairs.begin(); it != obj.velocityPairs.end(); it++) {
                defaulttype::Rigid3dTypes::DPos rvel(state_(dim_*it->second), state_(dim_*it->second + 1), state_(dim_*it->second + 2));
                defaulttype::Rigid3dTypes::setDPos(vel[it->first], rvel);
            }
        }

        MechanicalParams mp;
        MechanicalPropagatePositionAndVelocityVisitor(&mp).execute( obj.node );

        std::cout << "Params: " << std::endl;
        for (size_t opi = 0; opi < obj.oparams.size(); opi++) {
            OptimParams* oparam = obj.oparams[opi];
            helper::vector<Type> vecPar;

            switch (modelData.transformParams) {
            case 1:
                for (size_t i = 0; i < oparam->mappingIndices.size(); i++) {
                    vecPar.push_back(fabs(state_(reduced_state_index_ + oparam->mappingIndices[i])));
                    std::cout << vecPar.back() << " ";
                }
                break;
            default:
                for (size_t i = 0; i < oparam->mappingIndices.size(); i++) {
                    vecPar.push_back(state_(reduced_state_index_ + oparam->mappingIndices[i]));
                    std::cout << vecPar.back() << " ";
                }
            }


            oparam->setValue(vecPar);
        }
        std::cout << std::endl;

    }

}


template <class Type>
void SofaModelWrapper<Type>::Initialize()
{
    Verb("initialize sofaModelWrapper");
    /// get fixed nodes
    size_t vsi = 0;
    size_t vpi = 0;

    for (size_t iop = 0; iop < sofaObjects.size(); iop++) {
        SofaObject& obj = sofaObjects[iop];

        helper::vector<size_t> freeNodes;

        if (obj.vecMS != NULL) {
            for (int msi = 0; msi < obj.vecMS->getSize(); msi++) {
                bool found = false;
                if (obj.vecFC != NULL) {
                    const FixedConstraintVec3d::SetIndexArray& fix = obj.vecFC->f_indices.getValue();
                    for (size_t j = 0; j < fix.size(); j++) {
                        if (int(fix[j]) == msi) {
                            found = true;
                            break;
                        }
                    }
                }
                if (!found)
                    freeNodes.push_back(msi);
            }
        }

        if (obj.rigidMS != NULL) {
            for (int msi = 0; msi < obj.rigidMS->getSize(); msi++) {
                bool found = false;
                if (obj.vecFC != NULL) {
                    const FixedConstraintVec3d::SetIndexArray& fix = obj.rigidFC->f_indices.getValue();
                    for (size_t j = 0; j < fix.size(); j++) {
                        if (int(fix[j]) == msi) {
                            found = true;
                            break;
                        }
                    }
                }
                if (!found)
                    freeNodes.push_back(msi);
            }
        }

        obj.positionPairs.clear();
        obj.velocityPairs.clear();

        if (modelData.positionInState) {
            for (size_t i = 0; i < freeNodes.size(); i++) {
                std::pair<size_t, size_t> pr(freeNodes[i], vsi++);
                obj.positionPairs.push_back(pr);
            }
        }

        if (modelData.velocityInState) {
            for (size_t i = 0; i < freeNodes.size(); i++) {
                std::pair<size_t, size_t> pr(freeNodes[i], vsi++);
                obj.velocityPairs.push_back(pr);
            }
        }

        for (size_t pi = 0; pi < obj.oparams.size(); pi++) {
            helper::vector<size_t>& opv = obj.oparams[pi]->mappingIndices;
            opv.clear();

            for (size_t i = 0; i < obj.oparams[pi]->size(); i++)
                opv.push_back(vpi++);
        }

        reduced_state_index_ = dim_ * vsi;
        state_size_ = dim_* vsi + vpi;
        reduced_state_size_ = vpi;

        /// print out the structure:
        for (size_t iop = 0; iop < sofaObjects.size(); iop++) {
            SofaObject& obj = sofaObjects[iop];

            std::cout << "Object in node: " << obj.node->getName() << " " << obj.oparams.size() << std::endl;

            for (size_t i = 0; i < obj.oparams.size(); i++) {
                std::cout << "  Params: " << obj.oparams[i]->getName();
                std::cout << "     inx:";
                for (size_t j = 0; j < obj.oparams[i]->mappingIndices.size(); j++)
                    std::cout << " " << obj.oparams[i]->mappingIndices[j];
                std::cout << std::endl;
            }

            if (obj.vecMS && obj.vecFC)
                std::cout << "VecMO " << obj.vecMS->getName() << " size: " << obj.vecMS->getSize() << "  FC size: " << obj.vecFC->f_indices.getValue().size() << std::endl;

            if (obj.rigidMS && obj.rigidFC)
                std::cout << "RigidMO " << obj.rigidMS->getName() << " size: " << obj.rigidMS->getSize() << "  FC size: " << obj.rigidFC->f_indices.getValue().size() << std::endl;

            /*std::cout << "Num of position pairs: " << obj.positionPairs.size() << " inx: ";
            for (size_t i = 0; i < obj.positionPairs.size(); i++)
                std::cout << "(" << obj.positionPairs[i].first << "," << obj.positionPairs[i].second << ") ";
            std::cout << std::endl;

            std::cout << "Num of velocity pairs: " << obj.velocityPairs.size() << " inx: ";
            for (size_t i = 0; i < obj.velocityPairs.size(); i++)
                std::cout << "(" << obj.velocityPairs[i].first << "," << obj.velocityPairs[i].second << ") ";
            std::cout << std::endl;*/
        }
        /// end of print out
    }

    std::cout << "Initializing model (filter type " << modelData.filterType << ") with size " << state_size_ << std::endl;
    std::cout << "Reduced state index: " << reduced_state_index_ << " size: " << reduced_state_size_ << std::endl;

    state_.Nullify();
    state_.Resize(state_size_);

    StateSofa2Verdandi();

    if (modelData.filterType == ROUKF) {
        variance_projector_allocated_ = false;
        variance_reduced_allocated_ = false;
    }
}

template <class Type>
void SofaModelWrapper<Type>::FinalizeStep() {
    std::cout << "Actual parameter values:";
    switch (modelData.transformParams) {
    case 1:
        for (size_t i = reduced_state_index_; i < state_size_; i++)
            std::cout << " " << fabs(state_(i));
        break;
    default:
        for (size_t i = reduced_state_index_; i < state_size_; i++)
            std::cout << " " << state_(i);
    }

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
        std::cout << "[" << this->getName() << "]: state updated " << std::endl;
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
    Verb("apply operator begin");
    std::cout << "Apply operator on ";
    //for (size_t i = 0; i < _x.GetSize(); i++)
    //      std::cout << _x(i) << " ";
    //std::cout << std::endl;
    //char nm[100];
    //sprintf(nm, "aoState_%04d_%02u.dat", numStep, applyOpNum);
    //std::ofstream of(nm);
    //printVector(_x, of);
    //of.close();

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

    applyOpNum++;
    Verb("apply operator end");
    return new_time;
}

template <class Type>
void SofaModelWrapper<Type>::Forward(bool _update_force, bool _update_time)
{
    Verb("forward begin");

    if (constraintSolver)
        StepFreeMotion(_update_force, _update_time);
    else
        StepDefault(_update_force, _update_time);

    Verb("forward end");
}


template <class Type>
void SofaModelWrapper<Type>::StepDefault(bool _update_force, bool _update_time) {
    if (_update_force) {

    }
    simulation::Node* gnode = modelData.gnode;

    double    dt = gnode->getDt();

    //std::cout << "[" << this->getName() << "]: step default begin" << std::endl;

    sofa::helper::AdvancedTimer::stepBegin("AnimationStep");

    sofa::helper::AdvancedTimer::begin("Animate");

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    {
        //std::cout << "[" << this->getName() << "]: animate begin" << std::endl;
        AnimateBeginEvent ev ( dt );
        PropagateEventVisitor act ( execParams, &ev );
        gnode->execute ( act );
    }

    double startTime = gnode->getTime();

    //std::cout << "[" << this->getName() << "]: behaviour update position" << std::endl;
    BehaviorUpdatePositionVisitor beh(execParams , dt);
    gnode->execute ( beh );

    //std::cout << "[" << this->getName() << "]: animate" << std::endl;
    AnimateVisitor act(execParams, dt);
    gnode->execute ( act );

    if (_update_time) {
        //std::cout << "[" << this->getName() << "]: update simulation context" << std::endl;
        gnode->setTime ( startTime + dt );
        gnode->execute< UpdateSimulationContextVisitor >(execParams);
    }

    {
        //std::cout << "[" << this->getName() << "]: animate end" << std::endl;
        AnimateEndEvent ev ( dt );
        PropagateEventVisitor act ( execParams, &ev );
        gnode->execute ( act );
    }

    sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    //std::cout << "[" << this->getName() << "]: update mapping" << std::endl;
    gnode->execute< UpdateMappingVisitor >(execParams);
    sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        //std::cout << "[" << this->getName() << "]: update mapping end" << std::endl;
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


template <class Type>
void SofaModelWrapper<Type>::StepFreeMotion(bool _update_force, bool _update_time) {
    if (_update_force) {

    }

    simulation::Node* gnode = modelData.gnode;
    double dt = gnode->getDt();

    sofa::helper::AdvancedTimer::begin("Animate");

    sofa::helper::AdvancedTimer::stepBegin("AnimationStep");
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    {
        sofa::helper::AdvancedTimer::stepBegin("AnimateBeginEvent");
        AnimateBeginEvent ev ( dt );
        PropagateEventVisitor act ( execParams, &ev );
        gnode->execute ( act );
        sofa::helper::AdvancedTimer::stepEnd("AnimateBeginEvent");
    }

    double startTime = gnode->getTime();

    simulation::common::VectorOperations vop(execParams, this->getContext());
    simulation::common::MechanicalOperations mop(execParams, this->getContext());

    MultiVecCoord pos(&vop, core::VecCoordId::position() );
    MultiVecDeriv vel(&vop, core::VecDerivId::velocity() );
    MultiVecCoord freePos(&vop, core::VecCoordId::freePosition() );
    MultiVecDeriv freeVel(&vop, core::VecDerivId::freeVelocity() );

    // This solver will work in freePosition and freeVelocity vectors.
    // We need to initialize them if it's not already done.
    sofa::helper::AdvancedTimer::stepBegin("MechanicalVInitVisitor");
    simulation::MechanicalVInitVisitor< core::V_COORD >(execParams, core::VecCoordId::freePosition(), core::ConstVecCoordId::position(), true).execute(gnode);
    simulation::MechanicalVInitVisitor< core::V_DERIV >(execParams, core::VecDerivId::freeVelocity(), core::ConstVecDerivId::velocity(), true).execute(gnode);

    sofa::helper::AdvancedTimer::stepEnd("MechanicalVInitVisitor");

    BehaviorUpdatePositionVisitor beh(execParams , dt);

    using helper::system::thread::CTime;
    using sofa::helper::AdvancedTimer;

    double time = 0.0;
    //double timeTotal = 0.0;
    double timeScale = 1000.0 / (double)CTime::getTicksPerSec();

    if (displayTime.getValue())
    {
        time = (double) CTime::getTime();
        //timeTotal = (double) CTime::getTime();
    }

    // Update the BehaviorModels
    // Required to allow the RayPickInteractor interaction
    if (f_printLog.getValue())
        serr << "updatePos called" << sendl;

    AdvancedTimer::stepBegin("UpdatePosition");
    gnode->execute(&beh);
    AdvancedTimer::stepEnd("UpdatePosition");

    if (f_printLog.getValue())
        serr << "updatePos performed - beginVisitor called" << sendl;

    simulation::MechanicalBeginIntegrationVisitor beginVisitor(execParams, dt);
    gnode->execute(&beginVisitor);

    if (f_printLog.getValue())
        serr << "beginVisitor performed - SolveVisitor for freeMotion is called" << sendl;

    // Free Motion
    AdvancedTimer::stepBegin("FreeMotion");
    simulation::SolveVisitor freeMotion(execParams, dt, true);
    gnode->execute(&freeMotion);
    AdvancedTimer::stepEnd("FreeMotion");

    mop.propagateXAndV(freePos, freeVel, true);

    if (f_printLog.getValue())
        serr << " SolveVisitor for freeMotion performed" << sendl;

    if (displayTime.getValue())
    {
        sout << " >>>>> Begin display FreeMotionAnimationLoop time" << sendl;
        sout <<" Free Motion " << ((double)CTime::getTime() - time) * timeScale << " ms" << sendl;

        time = (double)CTime::getTime();
    }

    // Collision detection and response creation
    AdvancedTimer::stepBegin("Collision");
    computeCollision();
    AdvancedTimer::stepEnd  ("Collision");

    mop.propagateX(pos, false);

    if (displayTime.getValue())
    {
        sout << " computeCollision " << ((double) CTime::getTime() - time) * timeScale << " ms" << sendl;
        time = (double)CTime::getTime();
    }

    // Solve constraints
    if (constraintSolver)
    {
        AdvancedTimer::stepBegin("ConstraintSolver");

        if (m_solveVelocityConstraintFirst.getValue())
        {
            core::ConstraintParams cparams(*execParams);
            cparams.setX(freePos);
            cparams.setV(freeVel);

            cparams.setOrder(core::ConstraintParams::VEL);
            constraintSolver->solveConstraint(&cparams, vel);

            MultiVecDeriv dv(&vop, constraintSolver->getDx());
            mop.projectResponse(dv);
            mop.propagateDx(dv);

            // xfree += dv * dt
            freePos.eq(freePos, dv, dt);
            mop.propagateX(freePos, false);

            cparams.setOrder(core::ConstraintParams::POS);
            constraintSolver->solveConstraint(&cparams, pos);

            MultiVecDeriv dx(&vop, constraintSolver->getDx());

            //mop.projectResponse(vel);
            mop.propagateV(vel, true);
            mop.projectResponse(dx);
            mop.propagateDx(dx, true);

            // "mapped" x = xfree + dx
            simulation::MechanicalVOpVisitor(execParams, pos, freePos, dx, 1.0 ).setOnlyMapped(true).execute(gnode);
        }
        else
        {
            core::ConstraintParams cparams(*execParams);
            cparams.setX(freePos);
            cparams.setV(freeVel);

            constraintSolver->solveConstraint(&cparams, pos, vel);
            //mop.projectResponse(vel);
            mop.propagateV(vel, true);

            MultiVecDeriv dx(&vop, constraintSolver->getDx());
            mop.projectResponse(dx);
            mop.propagateDx(dx, true);

            // "mapped" x = xfree + dx
            simulation::MechanicalVOpVisitor(execParams, pos, freePos, dx, 1.0 ).setOnlyMapped(true).execute(gnode);
        }
        AdvancedTimer::stepEnd("ConstraintSolver");

    }

    if ( displayTime.getValue() )
    {
        sout << " contactCorrections " << ((double)CTime::getTime() - time) * timeScale << " ms" <<sendl;
        sout << "<<<<<< End display FreeMotionAnimationLoop time." << sendl;
    }

    simulation::MechanicalEndIntegrationVisitor endVisitor(execParams /* PARAMS FIRST */, dt);
    gnode->execute(&endVisitor);

    if (_update_time) {
        gnode->setTime ( startTime + dt );
        gnode->execute<UpdateSimulationContextVisitor>(execParams);  // propagate time
    }

    {
        AnimateEndEvent ev ( dt );
        PropagateEventVisitor act ( execParams, &ev );
        gnode->execute ( act );
    }


    sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    gnode->execute<UpdateMappingVisitor>(execParams);
    //	sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        UpdateMappingEndEvent ev ( dt );
        PropagateEventVisitor act ( execParams , &ev );
        gnode->execute ( act );
    }
    sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");

#ifndef SOFA_NO_UPDATE_BBOX
    sofa::helper::AdvancedTimer::stepBegin("UpdateBBox");
    gnode->execute<UpdateBoundingBoxVisitor>(execParams);
    sofa::helper::AdvancedTimer::stepEnd("UpdateBBox");
#endif
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("Step");
#endif

    sofa::helper::AdvancedTimer::stepEnd("AnimationStep");
    sofa::helper::AdvancedTimer::end("Animate");

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
    //std::cout << "GetSEV_PROJECTOR" << std::endl;
    if (!variance_projector_allocated_)
    {
        //int Nreduced = 0;
        //for (unsigned int i = 0; i < reduced_.size(); i++)
        //    Nreduced += x_.GetVector(reduced_[i]).GetSize();
        //std::cout << "  PP-Nreduced: " << Nreduced << std::endl;
        // Initializes L.
        //std::cout << "  P-reducedSize: " << reduced_.size() << std::endl;
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
    std::cout << "GetSEV_REDUCED" << std::endl;
    if (!variance_reduced_allocated_)
    {
        //int Nreduced = 0;
        //for (unsigned int i = 0; i < reduced_.size(); i++)
        //    Nreduced += x_.GetVector(reduced_[i]).GetSize();
        //std::cout << "  R-Nreduced: " << Nreduced << std::endl;
        // Initializes U.
        state_error_variance_reduced_.Reallocate(reduced_state_size_,  reduced_state_size_);
        state_error_variance_reduced_.Fill(Type(0.0));
        //for (size_t i = 0; i < reduced_state_size_; i++)
        //    state_error_variance_reduced_(i, i) = Type(Type(1.0) / modelData.errorVarianceSofaParams);
        for (size_t soi = 0, vpi = 0; soi < sofaObjects.size(); soi++) {
            SofaObject& obj = sofaObjects[soi];

            for (size_t opi = 0; opi < obj.oparams.size(); opi++) {
                OptimParams* oparam = obj.oparams[opi];
                //const helper::vector<Type>& stdev = op->getStdev();
                helper::vector<double>& stdev = oparam->getStDev();

                for (size_t pi = 0; pi < oparam->size(); pi++, vpi++)
                    state_error_variance_reduced_(vpi, vpi) = Type(Type(1.0) / (stdev[pi] * stdev[pi]));
            }
        }

        std::cout << "  Initialize U: " << std::endl;
        printMatrix(state_error_variance_reduced_, std::cout);
        variance_reduced_allocated_ = true;
        std::fstream f;
        f.open("U.mat", std::fstream::out);
        printMatrixInRow(state_error_variance_reduced_, f);
        f.close();
    } else {
        std::cout << "!!!!!!!!!!! U = " << std::endl;
        printMatrix(state_error_variance_reduced_, std::cout);
        std::fstream f;
        f.open("U.mat", std::fstream::out | std::fstream::app);
        printMatrixInRow(state_error_variance_reduced_, f);
        f.close();
    }
    //std::cout << "U = " << state_error_variance_reduced_ << std::endl;

    return state_error_variance_reduced_;
}

template<class Type>
void SofaModelWrapper<Type>::computeCollision()
{
    if (this->f_printLog.getValue()) std::cerr<<"CollisionAnimationLoop::computeCollision()"<<endl;

    {
        CollisionBeginEvent evBegin;
        PropagateEventVisitor eventPropagation( execParams /* PARAMS FIRST */, &evBegin);
        eventPropagation.execute(getContext());
    }

    CollisionVisitor act(execParams);
    act.setTags(this->getTags());
    act.execute( getContext() );

    {
        CollisionEndEvent evEnd;
        PropagateEventVisitor eventPropagation( execParams /* PARAMS FIRST */, &evEnd);
        eventPropagation.execute(getContext());
    }
}

/// ROUKF:
template <class Model, class ObservationManager>
SofaReducedOrderUKF<Model, ObservationManager>::SofaReducedOrderUKF()
    //: Inherit1()
    : Inherit2()
    , m_outputDirectory( initData(&m_outputDirectory, "outputDirectory", "working directory of the filter") )
    //, m_configFile( initData(&m_configFile, "configFile", "lua configuration file (temporary)") )
    , m_sigmaPointType( initData(&m_sigmaPointType, std::string("star"), "sigmaPointType", "type of sigma points (canonical|star|simplex)") )
    , m_observationErrorVariance( initData(&m_observationErrorVariance, std::string("matrix_inverse"), "observationErrorVariance", "observationErrorVariance") )
    , m_saveVQ( initData(&m_saveVQ, true, "saveVQ", "m_saveVQ") )
    , m_showIteration( initData(&m_showIteration, false, "showIteration", "showIteration") )
    , m_showTime( initData(&m_showTime, true, "showTime", "showTime") )
    , m_analyzeFirstStep( initData(&m_analyzeFirstStep, false, "analyzeFirstStep", "analyzeFirstStep") )
    , m_withResampling( initData(&m_withResampling, false, "withResampling", "withResampling") )
    , m_positionInState( initData(&m_positionInState, true, "positionInState", "include position in the non-reduced state") )
    , m_velocityInState( initData(&m_velocityInState, false, "velocityInState", "include position in the non-reduced state") )
    , m_transformParams( initData(&m_transformParams, 0, "transformParams", "transform estimated params: 0: do nothing, 1: absolute value, 2: quadratic (not implemented)") )

{
}

template <class Model, class ObservationManager>
void SofaReducedOrderUKF<Model, ObservationManager>::init() {
    SofaModelWrapper<double>::ModelData md;
    md.positionInState = m_positionInState.getValue();
    md.velocityInState = m_velocityInState.getValue();
    md.filterType = ROUKF;
    md.gnode = dynamic_cast<simulation::Node*>(this->getContext());
    md.transformParams = m_transformParams.getValue();

    this->model_.initSimuData(md);
}


template <class Model, class ObservationManager>
void SofaReducedOrderUKF<Model, ObservationManager>::InitializeFilter() { //VerdandiROUKFParams* _roukfParams) {
    simulation::Node* gnode = dynamic_cast<simulation::Node*>(this->getContext());

    gnode->get(this->observation_manager_, core::objectmodel::BaseContext::SearchDown);
    if (this->observation_manager_) {
        std::cout << this->getName() << " observation manager found: " << this->observation_manager_->getName() << std::endl;
    } else {
        std::cerr << this->getName() << " ERROR: observation manager not found." << std::endl;
        return;
    }

    InitializeParams();

    this->model_.Initialize();
    this->observation_manager_->Initialize(this->model_, this->configuration_file_);
    this->observation_manager_->DiscardObservation(false);

    InitializeStructures();
}


template <class Model, class ObservationManager>
void SofaReducedOrderUKF<Model, ObservationManager>
::InitializeParams() {
    this->output_directory_ = m_outputDirectory.getValue();
    this->configuration_file_ = m_configFile.getValue();

    this->saveVQ_ = m_saveVQ.getValue();
    this->analyze_first_step_ = m_analyzeFirstStep.getValue();
    this->with_resampling_ = m_withResampling.getValue();

    this->positionInState = m_positionInState.getValue();
    this->velocityInState = m_velocityInState.getValue();

    this->observation_error_variance_ = m_observationErrorVariance.getValue();
    this->sigma_point_type_ = m_sigmaPointType.getValue();

    this->option_display_["show_iteration"] = m_showIteration.getValue();
    this->option_display_["show_time"] = m_showTime.getValue();


    char comm[100];
    sprintf(comm, "rm %s/roukf*dat", this->output_directory_.c_str());
    std::cout << "Executing: " << comm << std::endl;
    system(comm);
}


/*template <class Model, class ObservationManager>
void SofaReducedOrderUKF<Model, ObservationManager>
::Initialize(std::string configuration_file,
             bool initialize_model, bool initialize_observation_manager)
{
    Verdandi::VerdandiOps configuration(configuration_file);
    Initialize(configuration, initialize_model,
               initialize_observation_manager);
}

template <class Model, class ObservationManager>
void SofaReducedOrderUKF<Model, ObservationManager>::Initialize(Verdandi::VerdandiOps& configuration, bool initialize_model, bool initialize_observation_manager)
{
    Verdandi::MessageHandler::Send(*this, "all", "::Initialize begin");

    this->configuration_file_ = configuration.GetFilePath();

    configuration.Set("output_directory", "", this->configuration_file_, this->output_directory_);

    char comm[100];
    sprintf(comm, "rm %s/roukf*dat", this->output_directory_.c_str());
    std::cout << "Executing: " << comm << std::endl;
    system(comm);

    configuration.SetPrefix("reduced_order_unscented_kalman_filter.");

    configuration.Set("output.saveVQ", this->saveVQ_);

    configuration.Set("model.configuration_file", "", this->configuration_file_,
                      this->model_configuration_file_);

    configuration.Set("observation_manager.configuration_file", "",
                      this->configuration_file_,
                      this->observation_configuration_file_);

    // Should iterations be displayed on screen?
    configuration.Set("display.show_iteration",
                      this->option_display_["show_iteration"]);
    // Should current time be displayed on screen?
    configuration.Set("display.show_time", this->option_display_["show_time"]);

    configuration.Set("data_assimilation.analyze_first_step",
                      this->analyze_first_step_);
    configuration.Set("data_assimilation.with_resampling",
                      this->with_resampling_);
    configuration.Set("data_assimilation.observation_error_variance",
                      "ops_in(v, {'matrix', 'matrix_inverse'})",
                      this->observation_error_variance_);

    configuration.Set("sigma_point.type",
                      "ops_in(v, {'canonical', 'star', 'simplex'})",
                      this->sigma_point_type_);

        configuration.
            SetPrefix("reduced_order_unscented_kalman_filter"
                      ".output_saver.");
        this->output_saver_.Initialize(configuration);
        this->output_saver_.Empty("forecast_time");
        this->output_saver_.Empty("forecast_state");
        this->output_saver_.Empty("analysis_time");
        this->output_saver_.Empty("analysis_state");

        configuration.SetPrefix("reduced_order_unscented_kalman_filter.");

        if (configuration.Exists("output.configuration"))
        {
            std::string output_configuration;
            configuration.Set("output.configuration",
                              output_configuration);
            configuration.WriteLuaDefinition(output_configuration);
        }


    if (configuration.Exists("output.log"))
        Verdandi::Logger::SetFileName(configuration.Get<std::string>("output.log"));

    if (initialize_model)
    {
        this->model_.Initialize(this->model_configuration_file_);
    }
    if (initialize_observation_manager)
    {
        this->observation_manager_.Initialize(this->model_,
                                        this->observation_configuration_file_);
        this->observation_manager_.DiscardObservation(false);
    }
}*/

 template <class Model, class ObservationManager>
 void SofaReducedOrderUKF<Model, ObservationManager>::InitializeStructures() {
    this->Nstate_ = this->model_.GetNstate();
    this->Nobservation_ = this->observation_manager_->GetNobservation();

    Copy(this->model_.GetStateErrorVarianceReduced(), this->U_);
    this->U_inv_.Copy(this->U_);

    GetInverse(this->U_inv_);

    this->Nreduced_ = this->U_.GetN();

    std::cout << "NSTATE: " << this->Nstate_ << " Nreduced: " << this->Nreduced_ << std::endl;

    /*** Sigma-points ***/

    typename Inherit1::sigma_point_matrix V_trans;
    if (this->sigma_point_type_ == "canonical")
        Verdandi::ComputeCanonicalSigmaPoint(this->Nreduced_, V_trans, this->D_alpha_,
                                   this->alpha_constant_);
    else if (this->sigma_point_type_ == "star")
        Verdandi::ComputeStarSigmaPoint(this->Nreduced_, V_trans, this->D_alpha_,
                              this->alpha_constant_);
    else if (this->sigma_point_type_ == "simplex")
        Verdandi::ComputeSimplexSigmaPoint(this->Nreduced_, V_trans, this->D_alpha_,
                                 this->alpha_constant_);
    if (this->alpha_constant_)
        this->alpha_ = this->D_alpha_(0);

    this->Nsigma_point_ = V_trans.GetM();

    std::cout << "NSigma: " << this->Nsigma_point_ << std::endl;

    // Initializes transpose of I.
    typename Inherit1::sigma_point_matrix P_alpha_v(this->Nreduced_, this->Nreduced_);
    this->I_trans_.Reallocate(this->Nsigma_point_, this->Nreduced_);

    if (this->alpha_constant_)
    {
        MltAdd(typename Inherit1::Ts(this->alpha_), Seldon::SeldonTrans, V_trans, Seldon::SeldonNoTrans, V_trans,
               typename Inherit1::Ts(0), P_alpha_v);
        GetInverse(P_alpha_v);
        Verdandi::GetCholesky(P_alpha_v);
        MltAdd(typename Inherit1::Ts(1), Seldon::SeldonNoTrans, V_trans, Seldon::SeldonTrans, P_alpha_v,
               typename Inherit1::Ts(0), this->I_trans_);
    }
    else
        throw Verdandi::ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                             "Initialize()", "Calculation not "
                             "implemented for no constant alpha_i.");
    this->I_.Copy(this->I_trans_);
    Transpose(this->I_);

    // Initializes D_v.
    this->D_v_.Reallocate(this->Nsigma_point_, this->Nsigma_point_);
    if (this->alpha_constant_)
        MltAdd(typename Inherit1::Ts(this->alpha_ * this->alpha_), Seldon::SeldonNoTrans, this->I_trans_, Seldon::SeldonTrans,
               this->I_trans_, typename Inherit1::Ts(0), this->D_v_);
    else
        throw Verdandi::ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                             "Initialize()", "Calculation not "
                             "implemented for no constant alpha_i.");

    /*** Assimilation ***/

    if (this->analyze_first_step_)
        this->Analyze();

    //if (initialize_model)
    {
        Verdandi::MessageHandler::Send(*this, "model", "initial condition");
        Verdandi::MessageHandler::Send(*this, "driver", "initial condition");
    }
    Verdandi::MessageHandler::Send(*this, "all", "::Initialize end");
}


 template <class DataTypes1, class DataTypes2>
 void MappedPointsObservationManager<DataTypes1, DataTypes2>::init() {
     simulation::Node* gnode = dynamic_cast<simulation::Node*>(this->getContext());

     gnode->get(mapping);
     if (mapping) {
         std::cout << "[" << this->getName() << "]: " << "found mapping: " << mapping->getName() << std::endl;
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no mapping found " << std::endl;

     gnode->get(observationSource);
     if (observationSource) {
         std::cout << "[" << this->getName() << "]: " << "found observation source: " << observationSource->getName() << std::endl;
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no observation source found " << std::endl;

     gnode->get(sofaModel, core::objectmodel::BaseContext::SearchUp);
     if (sofaModel) {
         std::cout << "[" << this->getName() << "]: " << "found SOFA model: " << sofaModel->getName() << std::endl;
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no SOFA model found " << std::endl;

     gnode->get(mappedState);
     if (mappedState) {
         std::cout << "[" << this->getName() << "]: " << "found mapped mechanical state: " << mappedState->getName() << std::endl;
         typename MappedState::ReadVecCoord mappedPos = mappedState->readPositions();
         mappedStateSize = mappedPos.size();
     } else
         std::cerr << "[" << this->getName() << "]: ERROR no mapped state found " << std::endl;


     sofaObject = NULL;
     masterState = dynamic_cast<MasterState*>(mapping->getFromModel());
     if (masterState != NULL) {
         sofaObject = sofaModel->getObject(masterState);
         typename MasterState::ReadVecCoord masterPos = masterState->readPositions();
         masterStateSize = masterPos.size();
     }
     else
         std::cerr << this->getName() << "ERROR SOFA MO not found!" << std::endl;

     if (!sofaObject)
         std::cerr << this->getName() << "ERROR SOFA object not found " << std::endl;



 }

 template <class DataTypes1, class DataTypes2>
 void MappedPointsObservationManager<DataTypes1, DataTypes2>::bwdInit() {
     typename DataTypes1::VecCoord& inputObservation = *inputObservationData.beginEdit();

     inputObservation = observationSource->getObservation(0.0);

     //std::cout << "Apply mapping on observations" << std::endl;

     MechanicalParams mp;
     mapping->apply(&mp, mappedObservationData, inputObservationData);

     noise.clear();
     noise.resize(3*mappedObservationData.getValue().size());
 }



 template <class DataTypes1, class DataTypes2>
 MappedPointsObservationManager<DataTypes1,DataTypes2>
 ::Inherit::observation& MappedPointsObservationManager<DataTypes1, DataTypes2>::GetInnovation(const typename SofaModelWrapper<double>::state& x) {
     std::cout << "[" << this->getName() << "]: new get innovation " << std::endl;


     //Data<typename DataTypes1::VecCoord> inputObservationData;
     //Data<typename DataTypes2::VecCoord> mappedObservationData;

     typename DataTypes1::VecCoord& inputObservation = *inputObservationData.beginEdit();

     inputObservation = observationSource->getObservation(this->time_);

     //std::cout << "Apply mapping on observations" << std::endl;

     MechanicalParams mp;
     mapping->apply(&mp, mappedObservationData, inputObservationData);

     //typename DataTypes2::VecCoord mappedObservation = *mappedObservationData.beginEdit();

     sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObservation = mappedObservationData;

     //std::cout << this->getName() << ": size of mapped observation: " << mappedObservation.size() << std::endl;
     Inherit::observation actualObs(mappedObservation.size()*3);
     for (size_t i = 0; i < mappedObservation.size(); i++)
         for (size_t d = 0; d < 3; d++) {
             mappedObservation[i][d] += noise[3*i+d];
             actualObs(3*i+d) = mappedObservation[i][d];
         }

     Data<typename DataTypes1::VecCoord> actualStateData;
     Data<typename DataTypes2::VecCoord> mappedStateData;

     typename DataTypes1::VecCoord& actualState = *actualStateData.beginEdit();
     typename DataTypes2::VecCoord& mappedState = *mappedStateData.beginEdit();

     mappedState.resize(mappedStateSize);
     sofaModel->SetSofaVectorFromVerdandiState(actualState, x, sofaObject);

     mapping->apply(&mp, mappedStateData, actualStateData);

     std::cout << this->getName() << ": size of mapped state: " << mappedState.size() << std::endl;
     this->innovation_.Reallocate(mappedState.size()*3);
     for (size_t i = 0; i < mappedState.size(); i++)
         for (size_t d = 0; d < 3; d++)
             this->innovation_(3*i+d) = mappedState[i][d];

     //this->innovation_.Reallocate(this->Nobservation_);
     //this->ApplyOperator(x, this->innovation_);     
     //Inherit::observation predObs  = this->innovation_;
     Mlt(double(-1.0), this->innovation_);     
     //Add(double(1.0), this->GetObservation(), this->innovation_);     
     Add(double(1.0), actualObs, this->innovation_);     
     //std::cout << this->getName() << ": innovation updated" << std::endl;

     //for (size_t i = 0; i < this->innovation_.GetM(); i++)
     //    std::cout << actualObs(i) << " " << predObs(i) << " " << this->innovation_(i) << std::endl;

     //std::cout << "ERROR VARIANCE: " << this->error_variance_ << std::endl;

     inputObservationData.endEdit();
     //mappedObservationData.endEdit();
     actualStateData.endEdit();
     mappedStateData.endEdit();

     //std::cout << "MPP: " << mappedObservationData.getValue() << std::endl;

     return this->innovation_;
 }

 template <class DataTypes1, class DataTypes2>
 void MappedPointsObservationManager<DataTypes1, DataTypes2>::Initialize(SofaModelWrapper<double>& /*model*/, std::string /*confFile*/) {
     Verb("initialize mappedPointsObsManager");
     //Inherit1::Initialize(model, confFile);

     this->Delta_t_ = 0.001;
     this->Nskip_= 1;
     this->initial_time_ = 0.0;
     this->final_time_ = 1000.0;


     if (int(masterStateSize) != observationSource->getNParticles()) {
         std::cerr << this->getName() << " ERROR: number of nodes in master state " << masterStateSize << " and observation source " << observationSource->getNParticles() << " differ!" << std::endl;
         return;
     }

     this->error_variance_value_ = m_errorVariance.getValue();
     this->Nobservation_ = 3*mappedStateSize;
     this->error_variance_.Reallocate(this->Nobservation_, this->Nobservation_);
     this->error_variance_.SetIdentity();
     Mlt(this->error_variance_value_, this->error_variance_);
     this->error_variance_inverse_.Reallocate(this->Nobservation_, this->Nobservation_);
     this->error_variance_inverse_.SetIdentity();
     Mlt(double(double(1.0)/ this->error_variance_value_), this->error_variance_inverse_);

     std::cout << this->getName() << " size of observed state: " << this->Nobservation_ << std::endl;

     return;
 }


/// LINEAR MANAGER

/*
template <class T>
template <class Model>
void SofaObservationManager<T>::Initialize(Model& model, std::string configuration_file)
{
    observation_aggregator_.Initialize(configuration_file);

    Verdandi::VerdandiOps configuration(configuration_file);
    InitializeOperator(model, configuration_file);
    configuration.SetPrefix("observation.");

    bool with_observation;
    configuration.Set("option.with_observation", with_observation);
    if (!with_observation)
        return;

    configuration.Set("file", observation_file_);
    configuration.Set("file_type", "", "binary", observation_file_type_);
    configuration.Set("observation_dataset_path","", "",
                      observation_dataset_path_);
    configuration.Set("type", "", "state", observation_type_);
    configuration.Set("Delta_t_constant", is_delta_t_constant_);
    if (!is_delta_t_constant_)
    {
        configuration.Set("observation_time_file",
                          observation_time_file_);
        observation_time_.Read(observation_time_file_);
    }
    else
        configuration.Set("Delta_t", "v > 0", Delta_t_);

    configuration.Set("Nskip", "v > 0", Nskip_);
    configuration.Set("initial_time", "", 0., initial_time_);
    configuration.Set("final_time", "", std::numeric_limits<double>::max(),
                      final_time_);

    time_ = std::numeric_limits<double>::min();

    configuration.Set("width_file", "", "", width_file_);

    if (observation_type_ == "state")
        Nbyte_observation_ = Nstate_model_ * sizeof(T) + sizeof(int);

    if (observation_type_ == "observation")
        Nbyte_observation_ = Nobservation_ * sizeof(T) + sizeof(int);

    if (is_delta_t_constant_)
    {

        int expected_file_size;
        expected_file_size = Nbyte_observation_
            * int((final_time_ - initial_time_)
                  / (Delta_t_ * double(Nskip_)) + 1.);

        int file_size;
        std::ifstream file_stream;
        file_stream.open(observation_file_.c_str());

#ifdef VERDANDI_CHECK_IO
        // Checks if the file was opened.
        if (!file_stream.is_open())
            throw Verdandi::ErrorIO("LinearObservationManager"
                          "::Initialize(model, configuration_file)",
                          "Unable to open file \""
                          + observation_file_ + "\".");
#endif

        file_stream.seekg(0, std::ios_base::end);
        file_size = file_stream.tellg() ;
        file_stream.close();

        if (expected_file_size > file_size)
            throw Verdandi::IOError("LinearObservationManager"
                          "::Initialize(model, configuration_file)",
                          "Too few available observations, the size of \""
                          + observation_file_ + "\" must be greater than "
                          + Seldon::to_str(expected_file_size) + " B.");
    }
    else
    {
        //int file_size;
        std::ifstream file_stream;
        file_stream.open(observation_file_.c_str());

#ifdef VERDANDI_CHECK_IO
        // Checks if the file was opened.
        if (!file_stream.is_open())
            throw Verdandi::ErrorIO("LinearObservationManager"
                          "::Initialize(model, configuration_file)",
                          "Unable to open file \""
                          + observation_file_ + "\".");
#endif
        file_stream.close();
    }
}


template <class T>
template <class Model>
void SofaObservationManager<T>::InitializeOperator(Model& model, std::string configuration_file)
{
    Verdandi::VerdandiOps configuration(configuration_file);
    Nstate_model_ = model.GetNstate();
    configuration.SetPrefix("observation.");

    configuration.Set("operator.scaled_identity",
                      operator_scaled_identity_);
    configuration.Set("error.variance", "v > 0",
                      error_variance_value_);

    if (operator_scaled_identity_)
    {
        configuration.Set("operator.diagonal_value",
                          operator_diagonal_value_);

        Nobservation_ = Nstate_model_;
#ifdef VERDANDI_TANGENT_LINEAR_OPERATOR_SPARSE
        build_diagonal_sparse_matrix(Nstate_model_,
                                     operator_diagonal_value_,
                                     tangent_operator_matrix_);
#else
        tangent_operator_matrix_.Reallocate(Nobservation_, Nstate_model_);
        tangent_operator_matrix_.SetIdentity();
        Mlt(operator_diagonal_value_, tangent_operator_matrix_);
#endif
    }
    else // Operator given in a Lua table or in a file.
    {
#ifdef VERDANDI_TANGENT_LINEAR_OPERATOR_SPARSE
        if (configuration.Is<std::string>("operator.value"))
        {
            Matrix<T, General, ArrayRowSparse> tmp;
            tmp.Read(configuration.Get<string>("operator.value"));
            Copy(tmp, tangent_operator_matrix_);
        }
        else
        {
            Matrix<T> tmp;
            configuration.Set("operator.value", tmp);
            if (tmp.GetN() % model.GetNstate() != 0)
                throw ErrorArgument("LinearObservationManager"
                                    "::Initialize()",
                                    "The total number of elements in the "
                                    "tangent operator matrix ("
                                    + to_str(tangent_operator_matrix_
                                             .GetN())
                                    + ") is not a multiple of the"
                                    " dimension of the model state ("
                                    + to_str(model.GetNstate()) + ").");
            tmp.Resize(tmp.GetN() / model.GetNstate(), model.GetNstate());
            Matrix<T, General, ArrayRowSparse> tmp_array;
            ConvertDenseToArrayRowSparse(tmp, tmp_array);
            Copy(tmp_array, tangent_operator_matrix_);
        }
        if (tangent_operator_matrix_.GetN() != model.GetNstate())
            throw ErrorArgument("LinearObservationManager::Initialize()",
                                "The number of columns of the tangent "
                                "operator matrix ("
                                + to_str(tangent_operator_matrix_.GetN())
                                + ") is inconsistent with the"
                                " dimension of the model state ("
                                + to_str(model.GetNstate()) + ").");
        Nobservation_ = tangent_operator_matrix_.GetM();
#else
        if (configuration.Is<string>("operator.value"))
            tangent_operator_matrix_
                .Read(configuration.Get<string>("operator.value"));
        else
        {
            tangent_operator_matrix_.Clear();
            configuration.Set("operator.value", tangent_operator_matrix_);
            if (tangent_operator_matrix_.GetN() % model.GetNstate() != 0)
                throw ErrorArgument("LinearObservationManager"
                                    "::Initialize()",
                                    "The total number of elements in the "
                                    "tangent operator matrix ("
                                    + to_str(tangent_operator_matrix_
                                             .GetN())
                                    + ") is not a multiple of the"
                                    " dimension of the model state ("
                                    + to_str(model.GetNstate()) + ").");
            tangent_operator_matrix_
                .Resize(tangent_operator_matrix_.GetN()
                        / model.GetNstate(),
                        model.GetNstate());
        }
        if (tangent_operator_matrix_.GetN() != model.GetNstate())
            throw ErrorArgument("LinearObservationManager::Initialize()",
                                "The number of columns of the tangent "
                                "operator matrix ("
                                + to_str(tangent_operator_matrix_.GetN())
                                + ") is inconsistent with the"
                                " dimension of the model state ("
                                + to_str(model.GetNstate()) + ").");
        Nobservation_ = tangent_operator_matrix_.GetM();
#endif
    }

#ifdef VERDANDI_OBSERVATION_ERROR_SPARSE
    build_diagonal_sparse_matrix(Nobservation_, error_variance_value_,
                                 error_variance_);
    build_diagonal_sparse_matrix(Nobservation_,
                                 T(T(1) / error_variance_value_),
                                 error_variance_inverse_);
#else
    error_variance_.Reallocate(Nobservation_, Nobservation_);
    error_variance_.SetIdentity();
    Mlt(error_variance_value_, error_variance_);
    error_variance_inverse_.Reallocate(Nobservation_, Nobservation_);
    error_variance_inverse_.SetIdentity();
    Mlt(T(T(1)/ error_variance_value_), error_variance_inverse_);
#endif

}

template <class T>
template <class Model>
void SofaObservationManager<T>::SetTime(Model& model, double time)
{
    SetTime(time);
}

template <class T>
void SofaObservationManager<T>::SetTime(double time)
{
    if (time_ == time)
        return;

    time_ = time;
    SetAvailableTime(time_, available_time_);
}



template <class T>
void SofaObservationManager<T>::SetAvailableTime(double time, time_vector& available_time)
{
    double time_inf, time_sup;
    int selection_policy;
    observation_aggregator_.GetContributionInterval(time, time_inf,
                                                    time_sup,
                                                    selection_policy);
    SetAvailableTime(time, time_inf, time_sup, selection_policy,
                     available_time);

    Verdandi::Logger::Log<3>(*this, Seldon::to_str(time) + ", [" + Seldon::to_str(time_inf) + " " +
                   Seldon::to_str(time_sup) + "], {" + to_str(available_time) +
                   "}\n");
}


template <class T>
void SofaObservationManager<T>::SetAvailableTime(double time_inf, double time_sup, time_vector& available_time)
{
    available_time.Clear();

    if (is_delta_t_constant_)
    {
        double period = Delta_t_ * Nskip_;
        double available_time_0
            = initial_time_
            + floor((time_inf - initial_time_) / period) * period;
        if (available_time_0 == time_inf)
            available_time.PushBack(available_time_0);
        available_time_0 += period;
        for (double t = available_time_0; t < time_sup; t += period)
            available_time.PushBack(t);
    }
    else
        for (int i = 0; i < observation_time_.GetM(); i++)
            if (observation_time_(i) >= time_inf &&
                observation_time_(i) <= time_sup)
                available_time.PushBack(observation_time_(i));
    return;
}


template <class T>
void SofaObservationManager<T>::SetAvailableTime(double time, double time_inf, double time_sup, int selection_policy, time_vector& available_time)
{
    available_time.Clear();
    time_inf = time_inf > initial_time_ ? time_inf : initial_time_;
    time_sup = time_sup < final_time_ ? time_sup : final_time_;


    if (is_delta_t_constant_)
    {
        double period = Delta_t_ * Nskip_;

        // All observations available in the given interval are
        // considered.
        if (selection_policy == 0)
        {
            double available_time_0
                = initial_time_
                + floor((time_inf - initial_time_) / period) * period;
            if (available_time_0 == time_inf)
                available_time.PushBack(available_time_0);
            available_time_0 += period;
            for (double t = available_time_0; t < time_sup; t += period)
                available_time.PushBack(t);
            observation_aggregator_.Contribution(time_, available_time_,
                                                 contribution_);
            return;
        }

        // Only the closest left observation and the closest right
        // observation are requested.
        if (selection_policy == 2)
        {
            double t1, t2;

            if (Verdandi::is_multiple(time - initial_time_, period))
                t1 = time;
            else
                t1 = initial_time_
                    + floor((time - initial_time_) / period) * period;

            t2 = t1 + period;

            if (t1 <= final_time_)
                available_time.PushBack(t1);
            if (t2 <= final_time_)
                available_time.PushBack(t2);
            observation_aggregator_.Contribution(time_, available_time_,
                                                 contribution_);
            return;
        }

        // All observations available in the given interval are considered
        // taking into account non constant triangle widths associated
        // with observations.
        if (selection_policy == 3)
        {
            double available_time_0, available_time_1;
            int Nobservation;

            Seldon::Vector<double> width_left, width_right, available_width_left,
                available_width_right;
            ReadObservationTriangleWidth(time_inf, time_sup, width_left,
                                         width_right);

            available_time_0 = initial_time_
                + floor((time_inf - initial_time_) / period) * period;
            if (!Verdandi::is_equal(available_time_0, time_inf))
                available_time_0 += period;

            available_time_1 = initial_time_
                + floor((time_sup - initial_time_) / period) * period;
            if (Verdandi::is_equal(available_time_1, time_sup))
                available_time_1 -= period;

            Nobservation = floor((available_time_1 - available_time_0)
                                 / period) + 1;

            double t = available_time_0;
            for (int i = 0; i < Nobservation; i++, t += period)
            {
                if (t < time)
                {
                    if (t + width_right(i) > time)
                    {
                        available_time.PushBack(t);
                        available_width_left.PushBack(width_left(i));
                        available_width_right.PushBack(width_right(i));
                    }
                }
                else if (t > time)
                {
                    if (t - width_left(i) < time)
                    {
                        available_time.PushBack(t);
                        available_width_left.PushBack(width_left(i));
                        available_width_right.PushBack(width_right(i));
                    }
                }
                else
                {
                    available_time.PushBack(t);
                    available_width_left.PushBack(width_left(i));
                    available_width_right.PushBack(width_right(i));
                }

            }
            observation_aggregator_.
                Contribution(time_, available_time_,
                             available_width_left, available_width_right,
                             contribution_);
            return;
        }
    }
    else
    {
        // All observations available in the given interval are
        // considered.
        if (selection_policy == 0)
        {
            for (int i = 0; i < observation_time_.GetM(); i++)
                if (observation_time_(i) >= time_inf &&
                    observation_time_(i) <= time_sup)
                    available_time.PushBack(observation_time_(i));
            observation_aggregator_.Contribution(time_, available_time_,
                                                 contribution_);
            return;
        }
        // Only the closest left observation and the closest right
        // observation are requested.
        if (selection_policy == 2)
        {
            if (observation_time_(0) > time)
                return;
            for (int i = 1; i < observation_time_.GetM(); i++)
                if (observation_time_(i) >= time)
                {
                    available_time.PushBack(observation_time_(i - 1));
                    available_time.PushBack(observation_time_(i));
                    observation_aggregator_.
                        Contribution(time_, available_time_,
                                     contribution_);
                    return;
                }
            return;
        }
    }

    throw Verdandi::ErrorArgument("void LinearObservationManager<T>"
                        "::SetAvailableTime(double time,"
                        " double time_inf, double time_sup,"
                        " int selection_policy,"
                        "LinearObservationManager<T>"
                        "::time_vector&"
                        "available_time) const");
}


template <class T>
void SofaObservationManager<T>::ReadObservationTriangleWidth(double time_inf, double time_sup,
                               Seldon::Vector<double>& width_left,
                               Seldon::Vector<double>& width_right) const
{
    double period, available_time_0, available_time_1;
    int Nwidth, Nbyte_width;
    Seldon::Vector<double> input_data;

    std::ifstream file_stream;
    file_stream.open(width_file_.c_str());
    std::streampos position;
#ifdef VERDANDI_CHECK_IO
    // Checks if the file was opened.
    if (!file_stream.is_open())
        throw Verdandi::ErrorIO("LinearObservationManager"
                      "::ReadObservationTriangleWidth()",
                      "Unable to open file \""
                      + width_file_ + "\".");
#endif
    period = Delta_t_ * Nskip_;
    available_time_0 = initial_time_
        + floor((time_inf - initial_time_) / period) * period;
    if (!Verdandi::is_equal(available_time_0, time_inf))
        available_time_0 += period;
    available_time_1 = initial_time_
        + floor((time_sup - initial_time_) / period) * period;
    if (Verdandi::is_equal(available_time_1, time_sup))
        available_time_1 -= period;

    Nwidth = floor((available_time_1 - available_time_0) / period) + 1;
    Nbyte_width = 2 * sizeof(double) + sizeof(int);
    width_left.Reallocate(Nwidth);
    width_right.Reallocate(Nwidth);

    position = floor((available_time_0 - initial_time_) / Delta_t_)
        * Nbyte_width;
    for (int i = 0; i < Nwidth; i++, position += Nskip_ * Nbyte_width)
    {
        file_stream.seekg(position);
        input_data.Read(file_stream);
        width_left(i) = input_data(0);
        width_right(i) = input_data(1);
        input_data.Clear();
    }

    file_stream.close();
}*/




} // namespace simulation

} // namespace sofa
