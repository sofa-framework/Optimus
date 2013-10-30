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

    /// looking for optimization parameters which will be a part of the state
    /*std::cout << "Searching optim params: " << std::endl;
    helper::vector<OPVector*> listOP;
    gnode->get<OPVector>(&listOP, BaseContext::SearchRoot );

    listOP3d.clear();
    listMS3d.clear();
    listFC3d.clear();

    /// TODO!!!:  we suppose there is only one OptimParam in a node
    for (size_t i = 0; i < listOP.size(); i++) {
        listOP3d.push_back(listOP[i]);

        simulation::Node* lnode = dynamic_cast<simulation::Node*>(listOP[i]->getContext());
        std::cout << "  Found optim params " << listOP[i]->getName() << " in node " << lnode->getName() << std::endl;

        MechStateVec3d* ms;
        lnode->get(ms);
        if (ms != NULL)
            std::cout << "Mechanical state found: " << ms->getName() << std::endl;
        else
            std::cerr << "PROBLEM: NO MECHANICAL STATE FOUND! " << std::endl;
        listMS3d.push_back(ms);

        FixedConstraintVec3d* fc;
        lnode->get(fc);
        if (fc != NULL) {
            std::cout << "Fixed constraints found: " << fc->getName() << std::endl;
        }
        else
            std::cerr << "PROBLEM: NO FIXED CONSTRAINTS FOUND! " << std::endl;
        listFC3d.push_back(fc);
    }*/

    gnode->get(constraintSolver, core::objectmodel::BaseContext::SearchDown);
    if (constraintSolver == NULL)
        std::cout << "No ConstraintSolver found, considering the version with no contacts" << std::endl;
    else
        std::cout << "Constraint solver " << constraintSolver->getName() << " found, modeling contacts" << std::endl;

    /// original implementation
    /*gnode->get(vecParams);
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

    std::cout << "Number of fixed constraints: " << fixedConstraints->f_indices.getValue().size() << std::endl;*/


    sofaObjects.clear();
    std::cout << "Searching optim params: " << std::endl;
    helper::vector<OPVector*> listOP;
    gnode->get<OPVector>(&listOP, BaseContext::SearchRoot );

    for (size_t iop = 0; iop < listOP.size(); iop++) {
        OPVector* op = listOP[iop];

        if (!op->optimize())
            continue;

        simulation::Node* opnode = dynamic_cast<simulation::Node*>(op->getContext());

        OPVecInd oparam;
        oparam.first = op;
        oparam.second.clear();

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

    numStep = 0;
}

template <class Type>
void SofaModelWrapper<Type>::StateSofa2Verdandi() {
    /*for (size_t si = 0; si < listMS3d.size(); si++) {
        typename MechStateVec3d::ReadVecCoord pos = listMS3d[si]->readPositions();
        typename MechStateVec3d::ReadVecDeriv vel = listMS3d[si]->readVelocities();

        size_t ii = 0;
        for (size_t i = listStateBegin[si]; i < listStateMiddle[si]; i++) {
            for (size_t d = 0; d < dim_; d++)
                state_(3*i+d) = pos[listFreeIndices[si][ii]][d];
            ii++;
        }

        ii = 0;
        for (size_t i = listStateMiddle[si]; i < listStateEnd[si]; i++) {
            for (size_t d = 0; d < dim_; d++)
                state_(3*i+d) = vel[listFreeIndices[si][ii]][d];
            ii++;
        }

        ii = 0;
        const helper::vector<double>& vecPar = listOP3d[si]->getValue();
        for (size_t i = listParamBegin[si]; i < listParamEnd[si]; i++)
            state_(i) = vecPar[ii++];
    }*/

    /*typename MechStateVec3d::ReadVecCoord pos = mechanicalObject->readPositions();
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
        state_(j++) = vecPar[i];*/

    //std::cout << "S2V: " << std::endl;

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
            OPVecInd& op = obj.oparams[opi];
            const helper::vector<Type> vecPar = op.first->getValue();
            for (size_t i = 0; i < op.second.size(); i++)
                state_(reduced_state_index_ + op.second[i]) = vecPar[i];
        }

    }

}

template <class Type>
void SofaModelWrapper<Type>::StateVerdandi2Sofa() {
    /*for (size_t si = 0; si < listMS3d.size(); si++) {
        typename MechStateVec3d::WriteVecCoord pos = listMS3d[si]->writePositions();
        typename MechStateVec3d::WriteVecDeriv vel = listMS3d[si]->writeVelocities();

        size_t ii = 0;
        for (size_t i = listStateBegin[si]; i < listStateMiddle[si]; i++) {
            for (size_t d = 0; d < dim_; d++)
                pos[listFreeIndices[si][ii]][d] = state_(3*i+d);
            ii++;
        }

        ii = 0;
        for (size_t i = listStateMiddle[si]; i < listStateEnd[si]; i++) {
            for (size_t d = 0; d < dim_; d++)
                vel[listFreeIndices[si][ii]][d] = state_(3*i+d);
            ii++;
        }

        ii = 0;
        helper::vector<Type> vecPar(listOP3d[si]->size());
        for (size_t i = listParamBegin[si]; i < listParamEnd[si]; i++)
            vecPar[ii++] = state_(i);
        listOP3d[si]->setValue(vecPar);
    }*/

    /*typename MechStateVec3d::WriteVecCoord pos = mechanicalObject->writePositions();
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
    vecParams->setValue(vecPar);*/

    /// THE LATEST VERSION:
    //std::cout << "V2S: " << std::endl;
    for (size_t iop = 0; iop < sofaObjects.size(); iop++) {
        SofaObject& obj = sofaObjects[iop];

        if (obj.vecMS != NULL) {
            typename MechStateVec3d::WriteVecCoord pos = obj.vecMS->writePositions();
            typename MechStateVec3d::WriteVecDeriv vel = obj.vecMS->writeVelocities();

            for (helper::vector<std::pair<size_t, size_t> >::iterator it = obj.positionPairs.begin(); it != obj.positionPairs.end(); it++)
                for (size_t d = 0; d < dim_; d++)
                    pos[it->first][d] = state_(dim_*it->second + d);

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

        for (size_t opi = 0; opi < obj.oparams.size(); opi++) {
            OPVecInd& op = obj.oparams[opi];
            helper::vector<Type> vecPar;
            for (size_t i = 0; i < op.second.size(); i++)
                vecPar.push_back(state_(reduced_state_index_ + op.second[i]));
            op.first->setValue(vecPar);
        }

    }

}


template <class Type>
void SofaModelWrapper<Type>::Initialize(std::string &/*configFile*/)
{
    Verb("initialize");
    /// get fixed nodes

    /*listStateBegin.resize(listMS3d.size(),size_t(0));
    listStateMiddle.resize(listMS3d.size(),0);
    listStateEnd.resize(listMS3d.size(),0);

    listParamBegin.resize(listMS3d.size(),0);
    listParamEnd.resize(listMS3d.size(),0);

    listFreeIndices.clear();

    size_t lastS = 0;
    size_t lastP = 0;
    for (size_t si = 0; si < listMS3d.size(); si++) {
        const FixedConstraintVec3d::SetIndexArray& fixedIndices = listFC3d[si]->f_indices.getValue();
        helper::vector<size_t> freeIndices;

        for (int i = 0; i < listMS3d[si]->getSize(); i++) {
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

        std::cout <<" FREE INDICES: " << freeIndices.size() << std::endl;

        listFreeIndices.push_back(freeIndices);
        size_t ns = freeIndices.size();
        size_t np = listOP3d[si]->size();

        listStateBegin[si] = lastS;
        if (modelData.positionInState)
            lastS += ns;
        listStateMiddle[si] = lastS;
        if (modelData.velocityInState)
            lastS += ns;
        listStateEnd[si] = lastS;

        listParamBegin[si] = lastP;
        lastP += np;
        listParamEnd[si] = lastP;

        reduced_state_index_ = state_size_;
    }

    for (size_t si = 0; si < listMS3d.size(); si++) {
        listParamBegin[si] += dim_ * lastS;
        listParamEnd[si] += dim_ * lastS;
    }

    reduced_state_index_ = dim_ * lastS;
    state_size_ = dim_* lastS + lastP;
    reduced_state_size_ = lastP;

    std::cout << "Initializing model (filter type " << modelData.filterType << ") with size " << state_size_ << std::endl;
    std::cout << "Reduced state index: " << reduced_state_index_ << " size: " << reduced_state_size_ << std::endl;*/

    /*const FixedConstraintVec3d::SetIndexArray& fixedIndices = fixedConstraints->f_indices.getValue();

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
    }*/

    /// THE LATEST VERSION
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
            helper::vector<size_t>& opv = obj.oparams[pi].second;
            opv.clear();

            for (size_t i = 0; i < obj.oparams[pi].first->size(); i++)
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
                std::cout << "  Params: " << obj.oparams[i].first->getName();
                std::cout << "     inx:";
                for (size_t j = 0; j < obj.oparams[i].second.size(); j++)
                    std::cout << " " << obj.oparams[i].second[j];
                std::cout << std::endl;
            }

            if (obj.vecMS && obj.vecFC)
                std::cout << "VecMO " << obj.vecMS->getName() << " size: " << obj.vecMS->getSize() << "  FC size: " << obj.vecFC->f_indices.getValue().size() << std::endl;

            if (obj.rigidMS && obj.rigidFC)
                std::cout << "RigidMO " << obj.rigidMS->getName() << " size: " << obj.rigidMS->getSize() << "  FC size: " << obj.rigidFC->f_indices.getValue().size() << std::endl;

            std::cout << "Num of position pairs: " << obj.positionPairs.size() << " inx: ";
            for (size_t i = 0; i < obj.positionPairs.size(); i++)
                std::cout << "(" << obj.positionPairs[i].first << "," << obj.positionPairs[i].second << ") ";
            std::cout << std::endl;

            std::cout << "Num of velocity pairs: " << obj.velocityPairs.size() << " inx: ";
            for (size_t i = 0; i < obj.velocityPairs.size(); i++)
                std::cout << "(" << obj.velocityPairs[i].first << "," << obj.velocityPairs[i].second << ") ";
            std::cout << std::endl;
        }
        /// end of print out
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

        //for (size_t i = reduced_state_index_; i < size_t(state_size_); i++)
        //    state_error_variance_(i,i) *= modelData.errorVarianceSofaParams;

        for (size_t soi = 0, vpi = 0; soi < sofaObjects.size(); soi++) {
            SofaObject& obj = sofaObjects[soi];

            for (size_t opi = 0; opi < obj.oparams.size(); opi++) {
                OPVector* op = obj.oparams[opi].first;
                const helper::vector<Type>& stdev = op->getStdev();
                for (size_t pi = 0; pi < op->size(); pi++, vpi++)
                    state_error_variance_(vpi, vpi) *=  (stdev[pi] * stdev[pi]);

            }
        }

        //Mlt(Type(state_error_variance_value_), state_error_variance_);

        state_error_variance_inverse_.Reallocate(GetNstate(), GetNstate());
        state_error_variance_inverse_.SetIdentity();
        for (size_t i = 0; i < size_t(reduced_state_index_); i++)
            state_error_variance_inverse_(i,i) *= modelData.errorVarianceSofaState;
        /*for (size_t i = size_t(reduced_state_index_); i < size_t(state_size_); i++)
            state_error_variance_inverse_(i,i) *= modelData.errorVarianceSofaParams;*/

        for (size_t soi = 0, vpi = 0; soi < sofaObjects.size(); soi++) {
            SofaObject& obj = sofaObjects[soi];

            for (size_t opi = 0; opi < obj.oparams.size(); opi++) {
                OPVector* op = obj.oparams[opi].first;
                const helper::vector<Type>& stdev = op->getStdev();
                for (size_t pi = 0; pi < op->size(); pi++, vpi++)
                    state_error_variance_inverse_(vpi, vpi) *=  (stdev[pi] * stdev[pi]);

            }
        }

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
    /*char nm[100];
    sprintf(nm, "aoState_%04d_%02u.dat", numStep, applyOpNum);
    std::ofstream of(nm);
    printVector(_x, of);
    of.close();*/

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
    Verb("state updated begin end");
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

    mop.propagateXAndV(freePos, freeVel);

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

    mop.propagateX(pos);

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
            mop.propagateX(freePos);

            cparams.setOrder(core::ConstraintParams::POS);
            constraintSolver->solveConstraint(&cparams, pos);

            MultiVecDeriv dx(&vop, constraintSolver->getDx());

            mop.projectResponse(vel);
            mop.propagateV(vel);
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
            mop.projectResponse(vel);
            mop.propagateV(vel);

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
    std::cout << "GetSEV_REDUCED" << std::endl;
    if (!variance_reduced_allocated_)
    {
        /*int Nreduced = 0;
        for (unsigned int i = 0; i < reduced_.size(); i++)
            Nreduced += x_.GetVector(reduced_[i]).GetSize();
        std::cout << "  R-Nreduced: " << Nreduced << std::endl;
        // Initializes U.*/
        state_error_variance_reduced_.Reallocate(reduced_state_size_,  reduced_state_size_);
        state_error_variance_reduced_.Fill(Type(0.0));
        //for (size_t i = 0; i < reduced_state_size_; i++)
        //    state_error_variance_reduced_(i, i) = Type(Type(1.0) / modelData.errorVarianceSofaParams);
        for (size_t soi = 0, vpi = 0; soi < sofaObjects.size(); soi++) {
            SofaObject& obj = sofaObjects[soi];

            for (size_t opi = 0; opi < obj.oparams.size(); opi++) {
                OPVector* op = obj.oparams[opi].first;
                const helper::vector<Type>& stdev = op->getStdev();

                for (size_t pi = 0; pi < op->size(); pi++, vpi++)
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
