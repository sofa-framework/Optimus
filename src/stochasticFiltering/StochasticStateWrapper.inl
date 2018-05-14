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
#ifndef SOFASTATEWRAPPER_INL
#define SOFASTATEWRAPPER_INL

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
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/UpdateMappingEndEvent.h>
#include <sofa/simulation/CleanupVisitor.h>
#include <sofa/simulation/DeleteVisitor.h>
#include <sofa/simulation/UpdateBoundingBoxVisitor.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/PipeProcess.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/IntegrateBeginEvent.h>
#include <sofa/simulation/IntegrateEndEvent.h>



#include "StochasticStateWrapper.h"


namespace sofa
{
namespace component
{
namespace stochastic
{
template<>
class InternalCopy<Rigid3dTypes> {
public :

    typedef typename Rigid3dTypes::VecCoord VecCoord;

    void copyStateToFilter(helper::vector<std::pair<size_t, size_t> > & positionPairs, const VecCoord & pos) {
        m_ori.clear();

        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++){
            //for (unsigned i=0;i<pos.size();i++) {
            m_ori.push_back(pos[it->first].getOrientation());
        }
        std::cout << "[internal stockedQUAT] " << m_ori <<std::endl;
    }

    void copyFilterToSofa(helper::vector<std::pair<size_t, size_t> > & positionPairs, VecCoord & pos) {
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++){
            //for (unsigned i=0;i<pos.size();i++) {
            pos[it->first].getOrientation() = m_ori[it->first];
            std::cout << "[internal injectedQUAT] " << pos[it->first].getOrientation() << std::endl;
        }
    }

    helper::vector<defaulttype::Quat> m_ori;

};

using namespace sofa::simulation;

template <class DataTypes, class FilterType>
StochasticStateWrapper<DataTypes, FilterType>::StochasticStateWrapper()
    :Inherit()
    , d_langrangeMultipliers( initData(&d_langrangeMultipliers, false, "langrangeMultipliers", "perform collision detection and response with Lagrange multipliers (requires constraint solver)") )
    , estimatePosition( initData(&estimatePosition, false, "estimatePosition", "estimate the position (e.g., if initial conditions with uncertainty") )
    , estimateVelocity( initData(&estimateVelocity, false, "estimateVelocity", "estimate the velocity (e.g., if initial conditions with uncertainty") )
    , posModelStdev( initData(&posModelStdev, helper::vector<FilterType>(0.0), "posModelStdev", "standard deviation in observations") )
    , velModelStdev( initData(&velModelStdev, helper::vector<FilterType>(0.0), "velModelStdev", "standard deviation in observations") )
    , paramModelStdev( initData(&paramModelStdev, helper::vector<FilterType>(0.0), "paramModelStdev", "standard deviation in observations") )
    , d_positionStdev( initData(&d_positionStdev, helper::vector<FilterType>(1, 0.0), "positionStdev", "estimate standard deviation for positions"))
    , d_velocityStdev( initData(&d_velocityStdev, helper::vector<FilterType>(1, 0.0), "velocityStdev", "estimate standard deviation for velocities"))
    //    , d_mappedStatePath(initData(&d_mappedStatePath, "mappedState", "Link to Virtual Mapped Catheter "))
    , d_draw(initData(&d_draw, false, "draw","Activation of draw"))
    , d_radius_draw( initData(&d_radius_draw, "radiusDraw", "radius of the spheres") )
    , d_fullMatrix( initData(&d_fullMatrix, "fullMatrix", "testing the full matrix" ) )
{
}

template <class DataTypes, class FilterType>
StochasticStateWrapper<DataTypes, FilterType>::~StochasticStateWrapper()
{
}

template <>
void StochasticStateWrapper<Rigid3dTypes, double>::stateDim(){
    posDim = 6;
    velDim = 6;
}

template <>
void StochasticStateWrapper<Vec3dTypes, double>::stateDim(){
    posDim = 3;
    velDim = 3;
}
template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::init()
{
    Inherit::init();
    valid = true;
    /// get a mechanical state
    this->gnode->get(mechanicalState,  core::objectmodel::BaseContext::SearchDown);
    if (!mechanicalState) {
        PRNE("No mechanical state found");
        valid=false;
    } else {
        PRNS("Mechanical state found: " << mechanicalState->getName());
    }

    /// initialize Dimensions
    stateDim();

    /// get optim params (multiple per node, at least one)
    vecOptimParams.clear();
    this->gnode->template get<OptimParamsBase>(&vecOptimParams, core::objectmodel::BaseContext::SearchDown );
    if (vecOptimParams.empty()) {
        PRNW("No OptimParams found");
        //valid=false;
    } else {
        PRNSC("OptimParams found " << vecOptimParams.size() << "x: ");
        if (this->verbose.getValue()) {
            for (size_t i = 0; i < vecOptimParams.size(); i++)
                std::cout << vecOptimParams[i]->getName() << " ";
            std::cout << std::endl;
        }
    }

    /// get fixed constraints (optional)
    this->gnode->get(fixedConstraint,  core::objectmodel::BaseContext::SearchDown);
    if (fixedConstraint) {
        PRNS("Fixed constraint found: " << fixedConstraint->getName());
    }

    /// search for constraint solver
    if (this->d_langrangeMultipliers.getValue()) {
        PRNS("Looking for constraint solver");

        {
            simulation::common::VectorOperations vop(core::ExecParams::defaultInstance(), this->getContext());
            core::behavior::MultiVecDeriv dx(&vop, core::VecDerivId::dx() ); dx.realloc( &vop, true, true );
            core::behavior::MultiVecDeriv df(&vop, core::VecDerivId::dforce() ); df.realloc( &vop, true, true );
        }


        this->getContext()->get(constraintSolver, core::objectmodel::BaseContext::SearchDown);
        if (constraintSolver == NULL && defaultSolver != NULL)
        {
            serr << "No ConstraintSolver found, using default LCPConstraintSolver" << sendl;
            this->getContext()->addObject(defaultSolver);
            constraintSolver = defaultSolver.get();
            defaultSolver = NULL;
        }
        else
        {
            defaultSolver.reset();
        }
    }
    //        this->getContext()->get(mappedState, d_mappedStatePath.getValue());
    //        if (mappedState == NULL) std::cout << "Error: Cannot find the Mapped State Component" <<std::endl;

    this->EstimatePOSITION = estimatePosition.getValue();
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::bwdInit() {
    if (!valid)
        return;
    this->mStateSize = mechanicalState->getSize();
    //        this->mappedMStateSize = mappedState->getSize();


    /// extract free and fixed nodes (fixed nodes cannot be included in the stochastic state)
    fixedNodes.clear();
    freeNodes.clear();
    if (fixedConstraint) {
        const typename FixedConstraint::SetIndexArray& fix = fixedConstraint-> d_indices.getValue();
        for (size_t i = 0; i < fix.size(); i++)
            fixedNodes.push_back(size_t(fix[i]));

        for (size_t i = 0; i < mechanicalState->getSize(); i++) {
            helper::vector<size_t>::iterator it = find(fixedNodes.begin(), fixedNodes.end(), i);

            if (it == fixedNodes.end())
                freeNodes.push_back(i);
        }
    } else {
        for (size_t i = 0; i < mechanicalState->getSize(); i++)
            freeNodes.push_back(i);
    }


    positionPairs.clear();
    velocityPairs.clear();

    size_t vsi = 0;
    size_t vpi = 0;

    posStdev.resize(posDim);
    velStdev.resize(velDim);


    if (d_positionStdev.getValue().size() != posDim) {
        PRNS("[WARNING] Bad initial value of Position Initial Covariance. Resized to 3 values for Vec3 mstate, or 6 values for Rigid mstate");
        for (size_t i=0 ;i<posDim;i++)
           posStdev[i]=d_positionStdev.getValue()[0];

       }else{
        for (size_t i=0 ;i<posDim;i++)
           posStdev[i]=d_positionStdev.getValue()[i];
       }

    if( d_velocityStdev.getValue().size() != velDim ){
        PRNS("[WARNING] Bad initial value of Velocity Initial Covariance. Resized 3 values for Vec3 mstate, or 6 values for Rigid mstate");
        for (size_t i=0 ;i<velDim;i++)
           velStdev[i]=d_velocityStdev.getValue()[0];

       }else{
        for (size_t i=0 ;i<posDim;i++)
           velStdev[i]=d_velocityStdev.getValue()[i];
       }
    if (estimatePosition.getValue()) {
        for (size_t i = 0; i < freeNodes.size(); i++) {
            std::pair<size_t, size_t> pr(freeNodes[i], vsi);
            vsi+=posDim;
            positionPairs.push_back(pr);
        }
        /// add standart deviation for positions
        this->positionVariance.resize(posDim * positionPairs.size());
        for (size_t i = 0; i < positionPairs.size(); i++){
            for (size_t j = 0; j < posDim; j ++)
                this->positionVariance[i*posDim+j] = posStdev[j%posDim] * posStdev[j%posDim];
        }
    }


    /*PRNS("Mapping stochastic -> mechanical");
    for (size_t i = 0; i < positionPairs.size(); i++) {
        PRNS(positionPairs[i].first << " --> " << positionPairs[i].second);
    }*/

    if (estimateVelocity.getValue()) {
        for (size_t i = 0; i < freeNodes.size(); i++) {
            std::pair<size_t, size_t> pr(freeNodes[i], vsi);
            vsi+=velDim;
            velocityPairs.push_back(pr);
        }
        /// add standart deviation for velocities
        this->velocityVariance.resize(velDim * velocityPairs.size());
        for (size_t i = 0; i < velocityPairs.size(); i++ ){
            for (size_t j = 0; j < velDim; j ++)
            this->velocityVariance[i*velDim+j] = velStdev[j%velDim] * velStdev[j%velDim];
        }
    }



    this->reducedStateIndex = vsi;
    for (size_t pi = 0; pi < vecOptimParams.size(); pi++) {
        helper::vector<size_t> opv;
        for (size_t i = 0; i < vecOptimParams[pi]->size(); i++, vpi++) {
            opv.push_back(this->reducedStateIndex+vpi);
        }
        vecOptimParams[pi]->setVStateParamIndices(opv);
    }

    this->stateSize = vsi + vpi;

    this->reducedStateSize = vpi;

    PRNS("Initializing stochastic state with size " << this->stateSize);
    PRNS("Reduced state index: " << this->reducedStateIndex << " size: " << this->reducedStateSize);

    this->state.resize(this->stateSize);
    copyStateSofa2Filter();
}


/// function called from observation manager when it needs to perform mapping (a more elegant solution could/should be found)
template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::setSofaVectorFromFilterVector(EVectorX& _state, typename DataTypes::VecCoord& _vec) {
    if (_vec.size() != mechanicalState->getSize()) {
        PRNE("Input vector not compatible with the actual Sofa state size");
        return;
    }

    typename MechanicalState::ReadVecCoord pos = mechanicalState->readPositions();
    for (size_t fni = 0; fni < fixedNodes.size(); fni++) {
        size_t fn = fixedNodes[fni];
        _vec[fn] = pos[fn];
        //PRNS("Setting fixed[" << fn << "] = " << _vec[fn]);
    }

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
        if (it->first >= _vec.size()) {
            PRNE("Accessing Sofa vector out of bounds: " << it->first <<  " vs. " << _vec.size());
            return;
        }

        if ((it->second + posDim) >= (size_t)_state.rows()) {
            PRNE("Accessing DA vector out of bounds: " << it->second + posDim <<  " vs. " << _state.rows());
            return;
        }

        for (size_t d = 0; d < posDim; d++) {
            _vec[it->first][d] = _state(it->second + d);
        }
        //PRNS("Setting free[" << it->first << "] = " << _vec[it->first]);
    }
}


/// function that sets SOFA state (position, velocity, parameters) from this->state (stochastic state, Eigen vector)
template <>
void StochasticStateWrapper<Rigid3dTypes, double>::copyStateFilter2Sofa(const core::MechanicalParams* _mechParams, bool _setVelocityFromPosition) {
    typename MechanicalState::WriteVecCoord pos = mechanicalState->writePositions();
    typename MechanicalState::WriteVecDeriv vel = mechanicalState->writeVelocities();

    helper::vector<Vector6> EulerPos;
    EulerPos.resize(pos.size());
    defaulttype::Quat q_ori;
    q_ori.normalize();
    Vector3 euler_ori;
    Vector3 eu_pos;

    if(estimatePosition.getValue()){
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
            for (size_t d = 0; d < posDim; d++) {
                EulerPos[it->first][d] = this->state(it->second + d);
                //    m_internalCopy.copyFilterToSofa(positionPairs,pos.wref());
            }
        }

        for(size_t i= 0; i < pos.size(); i++)    {
            for(size_t j=0; j <3; j++){
                eu_pos[j]=EulerPos[i][j];
            }
            unsigned k=0;
            for(size_t j=3; j <6; j++, k++){
                euler_ori[k]=EulerPos[i][j];
            }

            q_ori=defaulttype::Quat::createQuaterFromEuler(euler_ori*(M_PI/180.0));
            q_ori.normalize();
//            std::cout<< " HERE look at quat: "<< q_ori <<std::endl;

            pos[i].getOrientation()= q_ori;
            pos[i].getCenter() =eu_pos;
            q_ori.clear();
        }
    }
    if(estimateVelocity.getValue()){
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = velocityPairs.begin(); it != velocityPairs.end(); it++) {
            for (size_t d = 0; d < velDim; d++) {
                vel[it->first][d] = this->state(it->second + d);
            }
        }
    }
    /// if velocity is not estimated, it must be computed from the positions at the beginning and at the end of the time step
    if (_setVelocityFromPosition) {
        if (velocityPairs.empty() && beginTimeStepPos.size() == pos.size()) {
            for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
                for (size_t d = 0; d < velDim; d++) {
                    vel[it->first][d] = (pos[it->first][d] - beginTimeStepPos[it->first][d])/this->getContext()->getDt();
                }
            }
        }
    }

    sofa::simulation::MechanicalPropagateOnlyPositionAndVelocityVisitor(_mechParams).execute( this->gnode );

    /// let the OptimParams to extract the actual values of parameters from the verdandi state
    for (size_t opi = 0; opi < vecOptimParams.size(); opi++)
        vecOptimParams[opi]->vectorToParams(this->state);
}

/// function that compies SOFA state (position, velocity, parameters) to this->state (stochastic state, Eigen vector)
template <>
void StochasticStateWrapper<Rigid3dTypes, double>::copyStateSofa2Filter() {
    typename MechanicalState::ReadVecCoord pos = mechanicalState->readPositions();
    typename MechanicalState::ReadVecDeriv vel = mechanicalState->readVelocities();

    helper::vector<Vector6> EulerPos;
    EulerPos.resize(pos.size());
    defaulttype::Quat q_ori;
    q_ori.normalize();
    Vector3 euler_ori;
    Vector3 eu_pos;

    for(size_t i= 0; i < pos.size(); i++)    {
        q_ori.clear();
        q_ori=pos[i].getOrientation();
        eu_pos=pos[i].getCenter();
        euler_ori=q_ori.quatToRotationVector();

        for(size_t j=0; j <3; j++){
            EulerPos[i][j]=eu_pos[j];
        }
        unsigned k=0;
        for(size_t j=3; j <6; j++, k++){
            EulerPos[i][j]=euler_ori[k];
        }

    }

    if(estimatePosition.getValue()){
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++)
            for (size_t d = 0; d < posDim; d++) {
                this->state(it->second + d) = EulerPos[it->first][d];
            }
        //        m_internalCopy.copyStateToFilter(positionPairs,pos.ref());

    }

    if(estimateVelocity.getValue()){
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = velocityPairs.begin(); it != velocityPairs.end(); it++)
            for (size_t d = 0; d < velDim; d++)
                this->state(it->second + d) = vel[it->first][d];
    }
    for (size_t opi = 0; opi < vecOptimParams.size(); opi++)
        vecOptimParams[opi]->paramsToVector(this->state);

}


/// function that sets SOFA state (position, velocity, parameters) from this->state (stochastic state, Eigen vector)
template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::copyStateFilter2Sofa(const core::MechanicalParams* _mechParams, bool _setVelocityFromPosition) {
    typename MechanicalState::WriteVecCoord pos = mechanicalState->writePositions();
    typename MechanicalState::WriteVecDeriv vel = mechanicalState->writeVelocities();

    if(estimatePosition.getValue()){
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
            for (size_t d = 0; d < posDim; d++) {
                pos[it->first][d] = this->state(it->second + d);
                //    m_internalCopy.copyFilterToSofa(positionPairs,pos.wref());
            }
        }
    }

    if(estimateVelocity.getValue()){
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = velocityPairs.begin(); it != velocityPairs.end(); it++) {
            for (size_t d = 0; d < velDim; d++) {
                vel[it->first][d] = this->state(it->second + d);
            }
        }
    }
    /// if velocity is not estimated, it must be computed from the positions at the beginning and at the end of the time step
    if (_setVelocityFromPosition) {
        if (velocityPairs.empty() && beginTimeStepPos.size() == pos.size()) {
            for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
                for (size_t d = 0; d < velDim; d++) {
                    vel[it->first][d] = (pos[it->first][d] - beginTimeStepPos[it->first][d])/this->getContext()->getDt();
                }
            }
        }
    }

    sofa::simulation::MechanicalPropagateOnlyPositionAndVelocityVisitor(_mechParams).execute( this->gnode );

    /// let the OptimParams to extract the actual values of parameters from the verdandi state
    for (size_t opi = 0; opi < vecOptimParams.size(); opi++)
        vecOptimParams[opi]->vectorToParams(this->state);
}

/// function that compies SOFA state (position, velocity, parameters) to this->state (stochastic state, Eigen vector)
template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::copyStateSofa2Filter() {
    typename MechanicalState::ReadVecCoord pos = mechanicalState->readPositions();
    typename MechanicalState::ReadVecDeriv vel = mechanicalState->readVelocities();

    if(estimatePosition.getValue()){
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++)
            for (size_t d = 0; d < posDim; d++) {
                this->state(it->second + d) = pos[it->first][d];
            }
        //        m_internalCopy.copyStateToFilter(positionPairs,pos.ref());

    }

    if(estimateVelocity.getValue()){
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = velocityPairs.begin(); it != velocityPairs.end(); it++)
            for (size_t d = 0; d < velDim; d++)
                this->state(it->second + d) = vel[it->first][d];
    }
    for (size_t opi = 0; opi < vecOptimParams.size(); opi++)
        vecOptimParams[opi]->paramsToVector(this->state);

}

/// create an internal copy of SOFA state (positions and velocities); needed to reinitialize before simulation of each sigma point
template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::storeMState() {
    /// store the actual mechanical state (position, velocity)
    beginTimeStepPos.resize(this->mStateSize);
    beginTimeStepVel.resize(this->mStateSize);
    //    beginTimeStepMappedPos.resize(this->mappedMStateSize);


    typename MechanicalState::ReadVecCoord pos = mechanicalState->readPositions();
    typename MechanicalState::ReadVecDeriv vel = mechanicalState->readVelocities();
    //    typename MechanicalState::ReadVecCoord mapPos = mappedState->readPositions();

    for (size_t i = 0; i < this->mStateSize; i++) {
        beginTimeStepPos[i] = pos[i];
        beginTimeStepVel[i] = vel[i];
    }
    //    for (size_t i = 0; i < this->mappedMStateSize; i++) {
    //        beginTimeStepMappedPos[i] = mapPos[i];
    //    }
}

/// re-initialize SOFA state (positions and velocities) from back-up create by storeMState
template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::reinitMState(const core::MechanicalParams* _mechParams) {
    typename MechanicalState::WriteVecCoord pos = mechanicalState->writePositions();
    typename MechanicalState::WriteVecDeriv vel = mechanicalState->writeVelocities();
    //    typename MechanicalState::WriteVecCoord mapPos = mappedState->writePositions();

    for (size_t i = 0; i < this->mStateSize; i++) {
        pos[i] = beginTimeStepPos[i];
        vel[i] = beginTimeStepVel[i];
    }
    //    for (size_t i = 0; i < this->mappedMStateSize; i++)
    //        mapPos[i] = beginTimeStepMappedPos[i] ;

    sofa::simulation::MechanicalPropagateOnlyPositionAndVelocityVisitor(_mechParams).execute( this->gnode );

}


template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::getActualPosition(int _id, VecCoord& _pos) {
    _pos = sigmaStatePos[_id];
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::getActualMappedPosition(/*int _id, VecCoord& _mapPos*/) {
    //    _mapPos = sigmaMappedStatePos[_id];
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::initializeStep(size_t _stepNumber) {
    PRNS("Initialize time step" << _stepNumber);
    Inherit::initializeStep(_stepNumber);
    /// storing the initial state does not hurt for any type of the filter
    PRNS("Store mstate");
    storeMState();

    /// reinitialize the computed states
    sigmaStatePos.clear();
    sigmaStateVel.clear();
    sigmaMappedStatePos.clear();

}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::computeSimulationStep(EVectorX &_state, const core::MechanicalParams *_mparams, int& _stateID) {
    if (this->filterKind == SIMCORR) {
        typename MechanicalState::ReadVecCoord posT = mechanicalState->readPositions();

        reinitMState(_mparams);

        this->state = _state;
        copyStateFilter2Sofa(_mparams);

        if (this->d_langrangeMultipliers.getValue())
            computeSofaStepWithLM(_mparams);
        else
            computeSofaStep(_mparams, false);

        /// store the result of the simulation as a vector
        VecCoord actualPos(this->mStateSize);
        VecDeriv actualVel(this->mStateSize);

        typename MechanicalState::ReadVecCoord pos = mechanicalState->readPositions();
        typename MechanicalState::ReadVecDeriv vel = mechanicalState->readVelocities();

        for (size_t i = 0; i < this->mStateSize; i++) {
            actualPos[i] = pos[i];
            actualVel[i] = vel[i];
        }
        sigmaStatePos.push_back(actualPos);
        sigmaStateVel.push_back(actualVel);

        _stateID = sigmaStatePos.size() - 1;
    }
    return;
}


template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::transformState(EVectorX &_vecX, const core::MechanicalParams *_mparams, int* _stateID) {
    if (! (this->filterKind == CLASSIC || this->filterKind == REDORD) )
        return;

    EVectorX savedState;
    if ((_stateID == nullptr) || (*_stateID >= 0))
        savedState = this->state;


    reinitMState(_mparams);
    this->state = _vecX;

    copyStateFilter2Sofa(_mparams);

    if (this->d_langrangeMultipliers.getValue())
        computeSofaStepWithLM(_mparams);
    else
        computeSofaStep(_mparams, false);

    copyStateSofa2Filter();
    _vecX = this->state;


    if (this->filterKind == CLASSIC) {
        /// store the result of the simulation as a vector
        VecCoord actualPos(this->mStateSize);
        VecDeriv actualVel(this->mStateSize);
        //        VecCoord actualMappedPos(this->mappedMStateSize);


        typename MechanicalState::ReadVecCoord pos = mechanicalState->readPositions();
        typename MechanicalState::ReadVecDeriv vel = mechanicalState->readVelocities();
        //        typename MechanicalState::ReadVecCoord mapPos = mappedState->readPositions();

        for (size_t i = 0; i < this->mStateSize; i++) {
            actualPos[i] = pos[i];
            actualVel[i] = vel[i];
        }
        sigmaStatePos.push_back(actualPos);
        sigmaStateVel.push_back(actualVel);

        //        for (size_t i = 0; i < this->mappedMStateSize; i++) {
        //            actualMappedPos[i] = mapPos[i];
        //        }
        //        sigmaMappedStatePos.push_back(actualMappedPos);

        if (_stateID != nullptr) {
            *_stateID = sigmaStatePos.size() - 1;
        }
    }

    if ((_stateID == nullptr) || (*_stateID >= 0)) {
        this->state = savedState;
        copyStateFilter2Sofa(_mparams);
    }
}
template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::lastApplyOperator(EVectorX &_vecX, const core::MechanicalParams *_mparams) {
    if (!this->filterKind == CLASSIC )
        return;

    this->state = _vecX;
    reinitMState(_mparams);

    copyStateFilter2Sofa(_mparams);

    if (this->d_langrangeMultipliers.getValue())
        computeSofaStepWithLM(_mparams);
    else
        computeSofaStep(_mparams, false);
    copyStateSofa2Filter();

}


template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::draw(const core::visual::VisualParams* vparams ) {
    if (d_draw.getValue()){
        if (vparams->displayFlags().getShowVisualModels()) {
            std::vector<sofa::defaulttype::Vec3d> points;

            for (unsigned i =0;  i < sigmaStatePos.size(); i++){
                VecCoord &pts = sigmaStatePos[i];
                points.resize(pts.size());

                for(unsigned j =0;  j < pts.size(); j++){
                    points[j][0]=pts[j][0];
                    points[j][1]=pts[j][1];
                    points[j][2]=pts[j][2];
                }
                vparams->drawTool()->drawSpheres(points, d_radius_draw.getValue(), sofa::defaulttype::Vec<4, float>(0.0f,0.5f,0.3f,1.0f));
            }
        }
    }

}
/// perform a simulation step, code taken from DefaultAnimationLoop

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::computeSofaStep(const core::ExecParams* execParams, bool _updateTime) {
    double    dt = this->gnode->getDt();
    //core::ExecParams* execParams = sofa::core::ExecParams::defaultInstance();

    //std::cout << "[" << this->getName()   << "]: step default begin at time = " << gnode->getTime() << " update time: " << _update_time << std::endl;

    sofa::helper::AdvancedTimer::stepBegin("AnimationStep");
    //std::cout<<"step "<<step++<<std::endl;
    //sofa::helper::AdvancedTimer::begin("Animate");
    //std::cout<<"step "<<step++<<std::endl;

#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printNode("Step");
#endif

    {
        //std::cout<<"step "<<step++<<std::endl;
        //std::cout << "[" << this->getName() << "]: animate begin" << std::endl;
        sofa::simulation::AnimateBeginEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act ( execParams, &ev );
        this->gnode->execute ( act );
        //std::cout<<"step "<<step++<<std::endl;
    }

    double startTime = this->gnode->getTime();
    //std::cout<<"step "<<step++<<std::endl;
    //std::cout << "[" << this->getName() << "]: behaviour update position" << std::endl;
    sofa::simulation::BehaviorUpdatePositionVisitor beh(execParams , dt);
    this->gnode->execute ( beh );
    //std::cout<<"step "<<step++<<std::endl;
    //std::cout << "[" << this->getName() << "]: animate" << std::endl;
    sofa::simulation::AnimateVisitor act(execParams, dt);
    this->gnode->execute ( act );

    if (_updateTime) {
        //std::cout << "[" << this->getName() << "]: update simulation context" << std::endl;
        this->gnode->setTime ( startTime + dt );
        this->gnode->template execute<  sofa::simulation::UpdateSimulationContextVisitor >(execParams);
    }
    //std::cout<<"step "<<step++<<std::endl;
    {
        //std::cout << "[" << this->getName() << "]: animate end" << std::endl;
        sofa::simulation::AnimateEndEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act (execParams, &ev );
        this->gnode->execute ( act );
    }

    sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    //std::cout << "[" << this->getName() << "]: update mapping" << std::endl;
    this->gnode->template execute<  sofa::simulation::UpdateMappingVisitor >(execParams);
    sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        //std::cout << "[" << this->getName() << "]: update mapping end" << std::endl;
        sofa::simulation::UpdateMappingEndEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act ( execParams , &ev );
        this->gnode->execute ( act );
    }
    sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");

#ifndef SOFA_NO_UPDATE_BBOX
    sofa::helper::AdvancedTimer::stepBegin("UpdateBBox");
    this->gnode->template execute<  sofa::simulation::UpdateBoundingBoxVisitor >(execParams);
    sofa::helper::AdvancedTimer::stepEnd("UpdateBBox");
#endif
#ifdef SOFA_DUMP_VISITOR_INFO
    simulation::Visitor::printCloseNode("Step");
#endif
    //sofa::helper::AdvancedTimer::end("Animate");
    sofa::helper::AdvancedTimer::stepEnd("AnimationStep");

}

/// perform a simulation step with collisions, code taken from FreeMotionAnimationLoop

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::computeSofaStepWithLM(const core::ExecParams* params) {
    //if (dt == 0)
    SReal dt = this->gnode->getDt();

    {
        sofa::simulation::AnimateBeginEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act ( params, &ev );
        this->gnode->execute ( act );
    }

    double startTime = this->gnode->getTime();

    simulation::common::VectorOperations vop(params, this->getContext());
    simulation::common::MechanicalOperations mop(params, this->getContext());

    core::behavior::MultiVecCoord pos(&vop, core::VecCoordId::position() );
    core::behavior::MultiVecDeriv vel(&vop, core::VecDerivId::velocity() );
    core::behavior::MultiVecCoord freePos(&vop, core::VecCoordId::freePosition() );
    core::behavior::MultiVecDeriv freeVel(&vop, core::VecDerivId::freeVelocity() );

    {
        core::behavior::MultiVecDeriv dx(&vop, core::VecDerivId::dx() ); dx.realloc( &vop, true, true );
        core::behavior::MultiVecDeriv df(&vop, core::VecDerivId::dforce() ); df.realloc( &vop, true, true );
    }


    // This solver will work in freePosition and freeVelocity vectors.
    // We need to initialize them if it's not already done.
    sofa::helper::AdvancedTimer::stepBegin("MechanicalVInitVisitor");
    simulation::MechanicalVInitVisitor< core::V_COORD >(params, core::VecCoordId::freePosition(), core::ConstVecCoordId::position(), true).execute(this->gnode);
    simulation::MechanicalVInitVisitor< core::V_DERIV >(params, core::VecDerivId::freeVelocity(), core::ConstVecDerivId::velocity(), true).execute(this->gnode);

    sofa::simulation::BehaviorUpdatePositionVisitor beh(params , dt);

    // Update the BehaviorModels
    // Required to allow the RayPickInteractor interaction

    this->gnode->execute(&beh);

    simulation::MechanicalBeginIntegrationVisitor beginVisitor(params, dt);
    this->gnode->execute(&beginVisitor);

    // Free Motion
    simulation::SolveVisitor freeMotion(params, dt, true);
    this->gnode->execute(&freeMotion);

    mop.projectPositionAndVelocity(freePos, freeVel); // apply projective constraints
    mop.propagateXAndV(freePos, freeVel);

    // Collision detection and response creation
    this->computeCollision(params);
    mop.propagateX(pos); // Why is this done at that point ???

    // Solve constraints
    if (constraintSolver)
    {
        if (m_solveVelocityConstraintFirst.getValue())
        {
            core::ConstraintParams cparams(*params);
            cparams.setX(freePos);
            cparams.setV(freeVel);

            cparams.setOrder(core::ConstraintParams::VEL);
            constraintSolver->solveConstraint(&cparams, vel);

            core::behavior::MultiVecDeriv dv(&vop, constraintSolver->getDx());
            mop.projectResponse(dv);
            mop.propagateDx(dv);

            // xfree += dv * dt
            freePos.eq(freePos, dv, dt);
            mop.propagateX(freePos);

            cparams.setOrder(core::ConstraintParams::POS);
            constraintSolver->solveConstraint(&cparams, pos);

            core::behavior::MultiVecDeriv dx(&vop, constraintSolver->getDx());

            mop.projectVelocity(vel); // apply projective constraints
            mop.propagateV(vel);
            mop.projectResponse(dx);
            mop.propagateDx(dx, true);

            // "mapped" x = xfree + dx
            simulation::MechanicalVOpVisitor(params, pos, freePos, dx, 1.0 ).setOnlyMapped(true).execute(this->gnode);
        }
        else
        {
            core::ConstraintParams cparams(*params);
            cparams.setX(freePos);
            cparams.setV(freeVel);

            constraintSolver->solveConstraint(&cparams, pos, vel);
            mop.projectVelocity(vel); // apply projective constraints
            mop.propagateV(vel);

            core::behavior::MultiVecDeriv dx(&vop, constraintSolver->getDx());
            mop.projectResponse(dx);
            mop.propagateDx(dx, true);

            // "mapped" x = xfree + dx
            simulation::MechanicalVOpVisitor(params, pos, freePos, dx, 1.0 ).setOnlyMapped(true).execute(this->gnode);
        }
    }

    simulation::MechanicalEndIntegrationVisitor endVisitor(params, dt);
    this->gnode->execute(&endVisitor);

    this->gnode->setTime ( startTime + dt );
    this->gnode->template execute<UpdateSimulationContextVisitor>(params);  // propagate time

    {
        sofa::simulation::AnimateEndEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act ( params, &ev );
        this->gnode->execute ( act );
    }


    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    this->gnode->template execute<UpdateMappingVisitor>(params);
    {
        sofa::simulation::UpdateMappingEndEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act ( params , &ev );
        this->gnode->execute ( act );
    }

#ifndef SOFA_NO_UPDATE_BBOX
    this->gnode->template execute<UpdateBoundingBoxVisitor>(params);
#endif
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::computeCollision(const core::ExecParams* params)
{
    {
        CollisionBeginEvent evBegin;
        PropagateEventVisitor eventPropagation( params, &evBegin);
        eventPropagation.execute(this->getContext());
    }

    CollisionVisitor act(params);
    act.setTags(this->getTags());
    act.execute( this->getContext() );

    {
        CollisionEndEvent evEnd;
        PropagateEventVisitor eventPropagation( params, &evEnd);
        eventPropagation.execute(this->getContext());
    }
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::integrate(const core::ExecParams* params, SReal dt)
{

    {
        IntegrateBeginEvent evBegin;
        PropagateEventVisitor eventPropagation( params, &evBegin);
        eventPropagation.execute(this->getContext());
    }

    MechanicalIntegrationVisitor act( params, dt );
    act.setTags(this->getTags());
    act.execute( this->getContext() );

    {
        IntegrateEndEvent evBegin;
        PropagateEventVisitor eventPropagation( params, &evBegin);
        eventPropagation.execute(this->getContext());
    }
}

/*template <class DataTypes, class FilterType>
const StochasticStateWrapper::Solvers& StochasticStateWrapper<DataTypes, FilterType>::getSolverSequence()
{
    simulation::Node* gnode = dynamic_cast<simulation::Node*>( getContext() );
    assert( gnode );
    return gnode->solver;
}*/



} // simulation
} // component
} // sofa

#endif // SOFASTATEWRAPPER_INL
