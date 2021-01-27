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
#pragma once

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
public:

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
    : Inherit()
    , d_langrangeMultipliers( initData(&d_langrangeMultipliers, false, "langrangeMultipliers", "perform collision detection and response with Lagrange multipliers (requires constraint solver)") )
    , estimatePosition( initData(&estimatePosition, false, "estimatePosition", "estimate the position (e.g., if initial conditions with uncertainty") )
    , estimateOnlyXYZ( initData(&estimateOnlyXYZ, false, "estimateOnlyXYZ", "estimate only the X Y Z for a Rigid Body") )
    , estimateVelocity( initData(&estimateVelocity, false, "estimateVelocity", "estimate the velocity (e.g., if initial conditions with uncertainty") )
    , posModelStdev( initData(&posModelStdev, helper::vector<FilterType>(0.0), "posModelStdev", "standard deviation in observations") )
    , velModelStdev( initData(&velModelStdev, helper::vector<FilterType>(0.0), "velModelStdev", "standard deviation in observations") )
    , paramModelStdev( initData(&paramModelStdev, helper::vector<FilterType>(0.0), "paramModelStdev", "standard deviation in observations") )
    , d_positionStdev( initData(&d_positionStdev, helper::vector<FilterType>(1, 0.0), "positionStdev", "estimate standard deviation for positions"))
    , d_velocityStdev( initData(&d_velocityStdev, helper::vector<FilterType>(1, 0.0), "velocityStdev", "estimate standard deviation for velocities"))
    , d_mappedStatePath(initData(&d_mappedStatePath, "mappedState", "Link to Virtual Mapped Catheter "))
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
    if(!estimateOnlyXYZ.getValue()){
        std::cout<<"[StochasticStateWrapper] Estimate 6DoF Position"<<std::endl;
        posDim = 6;
    }
    else {
        std::cout<<"[StochasticStateWrapper] Estimate 3Dof Position"<<std::endl;
        posDim = 3;
    }
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
//        PRNW("No OptimParams found");
        //valid=false;
    } else {
        PRNSC("OptimParams found " << vecOptimParams.size() << "x: ");
        if (this->verbose.getValue()) {
            for (size_t i = 0; i < vecOptimParams.size(); i++)
                std::cout << "[OptimParams] " << vecOptimParams[i]->getName() << " ";
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
    this->gnode->get(mappedState, d_mappedStatePath.getValue());
    if ( mappedState != NULL)  {
        std::cout<<"Found mapped mechanical state: " << mappedState->getName()<<std::endl;
        this->declaredMappedState=1;
    }
    else {
        std::cout<<"[WARNING] No mapped state state found! Necessary for BindedSimpleObservationManager"<<std::endl;
        this->declaredMappedState=0;
    }
    this->EstimatePOSITION = estimatePosition.getValue();
    this->EstimateONLYXYZ=estimateOnlyXYZ.getValue();
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::bwdInit() {
    if (!valid)
        return;
    this->mStateSize = mechanicalState->getSize();
    if ( mappedState != NULL)  {
        this->mappedMStateSize = mappedState->getSize();
    }

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
    if (estimatePosition.getValue()) {
        helper::vector<double> posStdev;
        posStdev.resize(posDim);
        if (d_positionStdev.getValue().size() != posDim) {
            PRNW("Bad initial value of Position Initial Covariance. Resize according to Position DoFs");
            for (size_t i=0 ;i<posDim;i++)
                posStdev[i]=d_positionStdev.getValue()[0];

        } else{
            for (size_t i=0 ;i<posDim;i++)
                posStdev[i]=d_positionStdev.getValue()[i];
        }

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
        helper::vector<double> velStdev;
        velStdev.resize(velDim);

        if( d_velocityStdev.getValue().size() != velDim ){
            PRNW("Bad initial value of Velocity Initial Covariance.  Resize according to Velocity DoFs");
            for (size_t i=0 ;i<velDim;i++)
                velStdev[i]=d_velocityStdev.getValue()[0];

        } else{
            for (size_t i=0 ;i<posDim;i++)
                velStdev[i]=d_velocityStdev.getValue()[i];
        }

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


    size_t vpi = 0;
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

    color.resize(this->stateSize+1); //TRUE IF USING SIMPLEX SIGMA PTS
    colorB.resize(color.size());
    for(size_t i =0; i < color.size(); i++){
        color[i]= ((double) rand() / (RAND_MAX)) ;
        colorB[i]= ((double) rand() / (RAND_MAX)) ;
    }
}



template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::updateState(bool addData) {
    if (addData) {
        size_t vsi = freeNodes.size() * posDim;
        if (estimateVelocity.getValue()) {
            vsi += freeNodes.size() * velDim;
        }

        size_t vpi = 0;
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
        updateStateErrorVariance();
        updateModelErrorVariance();
        copyStateSofa2Filter();

        color.resize(this->stateSize+1); //TRUE IF USING SIMPLEX SIGMA PTS
        colorB.resize(color.size());
        for(size_t i =0; i < color.size(); i++){
            color[i]= ((double) rand() / (RAND_MAX)) ;
            colorB[i]= ((double) rand() / (RAND_MAX)) ;
        }
    } else {
        PRNS("Update stochastic state with size " << this->stateSize);
        std::cout << "Update stochastic state with size " << this->stateSize << std::endl;
        updateStateErrorVariance();
        copyStateSofa2Filter();
    }
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
template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::setSofaVelocityFromFilterVector(EVectorX& _state, typename DataTypes::VecDeriv& _vel) {
    if (_vel.size() != mechanicalState->getSize()) {
        PRNE("Input velocity vector not compatible with the actual Sofa state size");
        return;
    }

    typename MechanicalState::ReadVecDeriv vel = mechanicalState->readVelocities();
    for (size_t fni = 0; fni < fixedNodes.size(); fni++) {
        size_t fn = fixedNodes[fni];
        _vel[fn] = vel[fn];
        //PRNS("Setting fixed[" << fn << "] = " << _vec[fn]);
    }

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = velocityPairs.begin(); it != velocityPairs.end(); it++) {
        if (it->first >= _vel.size()) {
            PRNE("Accessing Sofa vector out of bounds: " << it->first <<  " vs. " << _vel.size());
            return;
        }

        if ((it->second + velDim) >= (size_t)_state.rows()) {
            PRNE("Accessing DA vector out of bounds: " << it->second + velDim <<  " vs. " << _state.rows());
            return;
        }

        for (size_t d = 0; d < velDim; d++) {
            _vel[it->first][d] = _state(it->second + d);
        }
        //PRNS("Setting free[" << it->first << "] = " << _vec[it->first]);
    }
}


/// function that sets SOFA state (position, velocity, parameters) from this->state (stochastic state, Eigen vector)
template <>
void StochasticStateWrapper<Rigid3dTypes, double>::copyStateFilter2Sofa(const core::MechanicalParams* _mechParams, bool /* _setVelocityFromPosition */) {
    typename MechanicalState::WriteVecCoord pos = mechanicalState->writePositions();
    typename MechanicalState::WriteVecDeriv vel = mechanicalState->writeVelocities();

    /// Position and Orientation
    if(estimatePosition.getValue() && !estimateOnlyXYZ.getValue()){
        helper::vector<Vector6> EulerPos;
        EulerPos.resize(pos.size());
        defaulttype::Quat q_ori;
        q_ori.normalize();
        Vector3 euler_ori;
        Vector3 eu_pos;
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

            q_ori=defaulttype::Quat::createQuaterFromEuler(euler_ori*M_PI/180);
            q_ori.normalize();
            pos[i].getOrientation()= q_ori;
            pos[i].getCenter() =eu_pos;
            q_ori.clear();
        }
    }

    if(!estimatePosition.getValue()&& estimateOnlyXYZ.getValue()){
       std::cout<<"ERROR check boolean estimatePosition " <<std::endl;
    }

    /// Only Position in State Vector
    if(estimatePosition.getValue() && estimateOnlyXYZ.getValue()){

        helper::vector<Vector3> EulerPos;
        EulerPos.resize(pos.size());
        Vector3 eu_pos;

        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
            for (size_t d = 0; d < 3; d++) {
                EulerPos[it->first][d] = this->state(it->second + d);
            }
        }

        for(size_t i= 0; i < pos.size(); i++)    {
            for(size_t j=0; j <3; j++){
                eu_pos[j]=EulerPos[i][j];
            }
            pos[i].getCenter() =eu_pos;
        }


        //Compute Quaternion for First Node
        {
            Vector3 P0(EulerPos[0][0],EulerPos[0][1],EulerPos[0][2]);
            Vector3 P1(EulerPos[1][0],EulerPos[1][1],EulerPos[1][2]);

            Vector3 X0 = P1-P0;
            X0.normalize();
            Vector3 Y0,Z0;

            if (fabs(dot(X0,Vector3(1,0,0))) >= 0.999999999999999) {
                Y0 = cross(X0,Vector3(0,1,0));
            }else {
                Y0 = cross(X0,Vector3(1,0,0));
            }
            Y0.normalize();
            Z0 = cross(X0,Y0);
            Z0.normalize();
            pos[0].getOrientation().fromFrame(X0,Y0,Z0);
        }

        //Compute Quaternion for Beam Shaft
        for(size_t i= 1; i < pos.size()-1; i++)    {
            Vector3 P0(EulerPos[i][0],EulerPos[i][1],EulerPos[i][2]);
            Vector3 P1(EulerPos[i+1][0],EulerPos[i+1][1],EulerPos[i+1][2]);

            Vector3 X = P1-P0;
            X.normalize();
            Vector3 Yprec = pos[i-1].getOrientation().rotate(Vector3(0,1,0));
            Vector3 Z = cross(X,Yprec);
            Z.normalize();

            Vector3 Y = cross(Z,X);
            Y.normalize();

            pos[i].getOrientation().fromFrame(X,Y,Z);
        }


        // Compute quaternion for last node
        int end =pos.size()-1;{

            Vector3 P0(EulerPos[end-1][0],EulerPos[end-1][1],EulerPos[end-1][2]);
            Vector3 P1(EulerPos[end][0],EulerPos[end][1],EulerPos[end][2]);

            Vector3 X = P1-P0;
            X.normalize();
            Vector3 Yprec = pos[end-1].getOrientation().rotate(Vector3(0,1,0));

            Vector3 Z = cross(X,Yprec);
            Z.normalize();

            Vector3 Y = cross(Z,X);
            Y.normalize();

            pos[end].getOrientation().fromFrame(X,Y,Z);
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
    if (!estimateVelocity.getValue()) {
        if (velocityPairs.empty() && beginTimeStepPos.size() == pos.size()) {


            helper::vector<Vector6> EulerPos;            EulerPos.resize(pos.size());
            defaulttype::Quat q_ori;            q_ori.normalize();
            Vector3 euler_ori;           Vector3 eu_pos;


            helper::vector<Vector6> begEulerPos;            begEulerPos.resize(pos.size());
            defaulttype::Quat begq_ori;            begq_ori.normalize();
            Vector3 begeuler_ori;           Vector3 begeu_pos;

            for(size_t i= 0; i < pos.size(); i++)    {
                q_ori.clear();
                q_ori=pos[i].getOrientation();
                q_ori.normalize();
                eu_pos=pos[i].getCenter();
                euler_ori=q_ori.toEulerVector();

                begq_ori.clear();
                begq_ori=beginTimeStepPos[i].getOrientation();
                begq_ori.normalize();
                begeu_pos=beginTimeStepPos[i].getCenter();
                begeuler_ori=begq_ori.toEulerVector();

                for(size_t j=0; j <3; j++){
                    EulerPos[i][j]=eu_pos[j];
                    begEulerPos[i][j]=begeu_pos[j];

                }
                unsigned k=0;
                for(size_t j=3; j <6; j++, k++){
                    EulerPos[i][j]=euler_ori[k]*180/M_PI;
                    begEulerPos[i][j]=begeuler_ori[k]*180/M_PI;

                }

            }


            for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
                for (size_t d = 0; d < 6; d++) {
                    vel[it->first][d] = (EulerPos[it->first][d] - begEulerPos[it->first][d])/this->getContext()->getDt();
                }
//                std::cout<<"pos: " <<pos <<std::endl;
//                std::cout<<"beginTimeStepPos: " <<beginTimeStepPos <<std::endl;
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


    if(!estimatePosition.getValue()&& estimateOnlyXYZ.getValue()){
       std::cout<<"ERROR check boolean estimatePosition " <<std::endl;
    }
    if(estimatePosition.getValue()&& !estimateOnlyXYZ.getValue()){

        helper::vector<Vector6> EulerPos;
        EulerPos.resize(pos.size());
        defaulttype::Quat q_ori;
        q_ori.normalize();
        Vector3 euler_ori;
        Vector3 eu_pos;

        for(size_t i= 0; i < pos.size(); i++)    {
            q_ori.clear();
            q_ori=pos[i].getOrientation();
            q_ori.normalize();
            eu_pos=pos[i].getCenter();
            euler_ori=q_ori.toEulerVector();
            //        euler_ori.normalize();

            for(size_t j=0; j <3; j++){
                EulerPos[i][j]=eu_pos[j];
            }
            unsigned k=0;
            for(size_t j=3; j <6; j++, k++){
                EulerPos[i][j]=euler_ori[k]*180/M_PI;
            }

        }
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++)
            for (size_t d = 0; d < posDim; d++) {
                this->state(it->second + d) = EulerPos[it->first][d];
            }
    }


    if(estimatePosition.getValue()&&estimateOnlyXYZ.getValue()){
        helper::vector<Vector3> EulerPos;
        EulerPos.resize(pos.size());
        Vector3 eu_pos;

        for(size_t i= 0; i < pos.size(); i++)    {
            eu_pos=pos[i].getCenter();
            for(size_t j=0; j <3; j++){
                EulerPos[i][j]=eu_pos[j];
            }

        }
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++)
            for (size_t d = 0; d < 3; d++) {
                this->state(it->second + d) = EulerPos[it->first][d];
            }
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

    typename MechanicalState::ReadVecCoord pos = mechanicalState->readPositions();
    typename MechanicalState::ReadVecDeriv vel = mechanicalState->readVelocities();

    for (size_t i = 0; i < this->mStateSize; i++) {
        beginTimeStepPos[i] = pos[i];
        beginTimeStepVel[i] = vel[i];
    }
    if ( mappedState != NULL)  {
        beginTimeStepMappedPos.resize(this->mappedMStateSize);
        typename MappedMechanicalState::ReadVecCoord mapPos = mappedState->readPositions();
        for (size_t i = 0; i < this->mappedMStateSize; i++) {
            beginTimeStepMappedPos[i] = mapPos[i];
        }
    }
}

/// re-initialize SOFA state (positions and velocities) from back-up create by storeMState
template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::reinitMState(const core::MechanicalParams* _mechParams) {
    typename MechanicalState::WriteVecCoord pos = mechanicalState->writePositions();
    typename MechanicalState::WriteVecDeriv vel = mechanicalState->writeVelocities();
    for (size_t i = 0; i < this->mStateSize; i++) {
        pos[i] = beginTimeStepPos[i];
        vel[i] = beginTimeStepVel[i];
    }
    if ( mappedState != NULL)  {
        typename MappedMechanicalState::WriteVecCoord mapPos = mappedState->writePositions();
        for (size_t i = 0; i < this->mappedMStateSize; i++)
            mapPos[i] = beginTimeStepMappedPos[i] ;
    }

    sofa::simulation::MechanicalPropagateOnlyPositionAndVelocityVisitor(_mechParams).execute( this->gnode );

}


template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::getActualPosition(int _id, VecCoord& _pos) {
    _pos = sigmaStatePos[_id];
}


template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::getActualVelocity(int _id, VecDeriv& _vel) {
    _vel = sigmaStateVel[_id];
}


template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::getPos(EVectorX& _state, VecCoord& actualPos) {
    if (! (this->filterKind == CLASSIC || this->filterKind == REDORD || this->filterKind == LOCENSEMBLE) )
        return;

    EVectorX savedState;
    savedState = this->state;
    this->state = _state;


    typename MechanicalState::WriteVecCoord posW = mechanicalState->writePositions();
    if(estimatePosition.getValue()){
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
            for (size_t d = 0; d < posDim; d++) {
                posW[it->first][d] = _state(it->second + d);
            }
        }
    }

    typename MechanicalState::ReadVecCoord posRead = mechanicalState->readPositions();
    for (size_t i = 0; i < this->mStateSize; i++) {
        actualPos[i] = posRead[i];
    }


    typename MechanicalState::WriteVecCoord pos = mechanicalState->writePositions();
    if(estimatePosition.getValue()){
        for (helper::vector<std::pair<size_t, size_t> >::iterator it = positionPairs.begin(); it != positionPairs.end(); it++) {
            for (size_t d = 0; d < posDim; d++) {
                pos[it->first][d] = savedState(it->second + d);
            }
        }
    }

}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::getActualMappedPosition(int _id, Vec3dTypes::VecCoord& _mapPos) {
    if ( mappedState != NULL)
        _mapPos = sigmaMappedStatePos[_id];
}

template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::setState(EVectorX& _state, const core::MechanicalParams* _mparams) {
    double    dt = this->gnode->getDt();
    this->state = _state;

    copyStateFilter2Sofa(_mparams, true);
    sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    //std::cout << "[" << this->getName() << "]: update mapping" << std::endl;
    this->gnode->template execute<  sofa::simulation::UpdateMappingVisitor >(_mparams);
    sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        //std::cout << "[" << this->getName() << "]: update mapping end" << std::endl;
        sofa::simulation::UpdateMappingEndEvent ev ( dt );
        sofa::simulation::PropagateEventVisitor act ( _mparams , &ev );
        this->gnode->execute ( act );
    }
    sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");
    sofa::simulation::AnimateEndEvent ev ( dt );

}

template <class DataTypes, class FilterType>
typename StochasticStateWrapper<DataTypes, FilterType>::EMatrixX& StochasticStateWrapper<DataTypes, FilterType>::getStateErrorVariance() {
    if (this->stateErrorVariance.rows() == 0) {
        PRNS("Constructing state co-variance matrix")
                this->stateErrorVariance.resize(this->stateSize, this->stateSize);
        this->stateErrorVariance.setZero();

        size_t vsi = 0;
        if (estimatePosition.getValue()) {
            for (size_t index = 0; index < (size_t)this->positionVariance.size(); index++, vsi++) {
                this->stateErrorVariance(vsi,vsi) = this->positionVariance[index];
            }
        }

        /// vsi continues to increase since velocity is always after position
        if (estimateVelocity.getValue()) {
            for (size_t index = 0; index < (size_t)this->velocityVariance.size(); index++, vsi++) {
                this->stateErrorVariance(vsi,vsi) = this->velocityVariance[index];
            }
        }


        for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
            helper::vector<double> variance;
            this->vecOptimParams[opi]->getInitVariance(variance);

            for (size_t pi = 0; pi < this->vecOptimParams[opi]->size(); pi++, vsi++)
                this->stateErrorVariance(vsi,vsi) = variance[pi];
        }
    }
    //std::cout << "P0: " << this->stateErrorVariance << std::endl;
    return this->stateErrorVariance;
}


template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::updateStateErrorVariance() {
    PRNS("Constructing state co-variance matrix")
    int oldStateVarianceSize = this->stateErrorVariance.rows();
    this->stateErrorVariance.conservativeResize(this->stateSize, this->stateSize);

    // set zeros to new added elements
    for (int first_index = 0; first_index < this->stateErrorVariance.rows(); first_index++) {
        for (int second_index = oldStateVarianceSize; second_index < this->stateErrorVariance.rows(); second_index++) {
            this->stateErrorVariance(first_index, second_index) = 0.0;
            this->stateErrorVariance(second_index, first_index) = 0.0;
        }
    }

    size_t vsi = 0;
    if (estimatePosition.getValue()) {
        vsi += (size_t)this->positionVariance.size();
    //     for (size_t index = 0; index < (size_t)this->positionVariance.size(); index++, vsi++) {
    //         this->stateErrorVariance(vsi,vsi) = this->positionVariance[index];
    //     }
    }

    /// vsi continues to increase since velocity is always after position
    if (estimateVelocity.getValue()) {
        vsi += (size_t)this->velocityVariance.size();
    //     for (size_t index = 0; index < (size_t)this->velocityVariance.size(); index++, vsi++) {
    //         this->stateErrorVariance(vsi,vsi) = this->velocityVariance[index];
    //     }
    }

    for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
        helper::vector<double> variance;
        this->vecOptimParams[opi]->getInitVariance(variance);

        for (size_t pi = 0; pi < this->vecOptimParams[opi]->size(); pi++, vsi++)
            if (vsi >= static_cast<size_t>(oldStateVarianceSize)) {
                this->stateErrorVariance(vsi,vsi) = variance[pi];
            }
    }

    std::cout << "stateErrorVariance: " << this->stateErrorVariance << std::endl;
    //std::cout << "P0: " << this->stateErrorVariance << std::endl;
}


template <class DataTypes, class FilterType>
typename StochasticStateWrapper<DataTypes, FilterType>::EMatrixX& StochasticStateWrapper<DataTypes, FilterType>::getModelErrorVariance() {
    if (this->modelErrorVariance.rows() == 0) {
        helper::vector<FilterType> velModStDev, posModStDev;

        if (estimatePosition.getValue()) {
            posModStDev.resize(posDim);
            for (size_t i=0 ;i<posDim;i++) {
                if (posModelStdev.getValue().size() != posDim) {
                    posModStDev[i]=posModelStdev.getValue()[0];
                }else{
                    posModStDev[i]=posModelStdev.getValue()[i];
                }
            }
        }


        if (estimateVelocity.getValue()) {
            velModStDev.resize(velDim);
            for (size_t i=0 ;i<velDim;i++) {
                if (velModelStdev.getValue().size() != velDim) {
                    velModStDev[i]=velModelStdev.getValue()[0];
                }else{
                    velModStDev[i]=velModelStdev.getValue()[i];
                }
            }
        }

        helper::vector<FilterType> paramModelStDev = paramModelStdev.getValue();
        modelErrorVariance = EMatrixX::Identity(this->stateSize, this->stateSize);
        modelErrorVarianceInverse = EMatrixX::Identity(this->stateSize, this->stateSize) ;

        size_t kp= 0;
        size_t kv= 0;
        size_t k= 0;
        const size_t N =freeNodes.size();
        const size_t M =posDim;
        const size_t V =velDim;

        EVectorX diagPosModelStDev;
        if (estimatePosition.getValue()) {
            diagPosModelStDev.resize(N*M);
            for (size_t i = 0; i < N; i++ ){
                for (size_t j = 0; j < M; j ++)
                    diagPosModelStDev(i*M+j)=posModStDev[j%M]*posModStDev[j%M];
            }
        }

        EVectorX diagVelModelStDev;
        if (estimateVelocity.getValue()) {
            diagVelModelStDev.resize(N*V);
            for (size_t i = 0; i < N; i++){
                for (size_t j = 0; j < V; j ++)
                    diagVelModelStDev(i*V+j)=velModStDev[j%V]*velModStDev[j%V];
            }
        }

        if (estimatePosition.getValue() && estimateVelocity.getValue()){
            modelErrorVariance = EMatrixX::Identity(this->stateSize, this->stateSize);
            for (unsigned index = 0; index < this->positionVariance.size(); index++, kp++)
                modelErrorVariance(index,index) = diagPosModelStDev(kp)  ;
            for (size_t index = this->positionVariance.size(); index < this->reducedStateIndex; index++,kv++)
                modelErrorVariance(index,index) = diagVelModelStDev[kv]  ;
            for (size_t pi = this->reducedStateIndex; pi < this->stateSize; pi++,k++)
                if (paramModelStdev.isSet()==0){
                    modelErrorVariance(pi,pi) = 0;
                }else{
                    modelErrorVariance(pi,pi) = paramModelStDev[k]  *paramModelStDev[k];    /// Non-null Q for Force
                }
        }

        //NORMALLY IS THE CASE OF DATA ASSIMILATION where Parameters have no Q
        if (estimatePosition.getValue() && !estimateVelocity.getValue()){
            modelErrorVariance = EMatrixX::Identity(this->stateSize, this->stateSize);
            for (unsigned index = 0; index < this->positionVariance.size(); index++, kp++)
                modelErrorVariance(index,index) = diagPosModelStDev(kp)  ;
            for (size_t pi = this->reducedStateIndex; pi < this->stateSize; pi++,k++)
                modelErrorVariance(pi,pi) = 0;
        }

    }
    return this->modelErrorVariance;

}

template <class DataTypes, class FilterType>
typename StochasticStateWrapper<DataTypes, FilterType>::EVectorX& StochasticStateWrapper<DataTypes, FilterType>::getModelElementNoise() {
    if (this->modelElementNoise.size() == 0) {
        helper::vector<FilterType> velModStDev, posModStDev;

        if (estimatePosition.getValue()) {
            posModStDev.resize(posDim);
            for (size_t i=0 ;i<posDim;i++) {
                if (posModelStdev.getValue().size() != posDim) {
                    posModStDev[i]=posModelStdev.getValue()[0];
                }else{
                    posModStDev[i]=posModelStdev.getValue()[i];
                }
            }
        }


        if (estimateVelocity.getValue()) {
            velModStDev.resize(velDim);
            for (size_t i=0 ;i<velDim;i++) {
                if (velModelStdev.getValue().size() != velDim) {
                    velModStDev[i]=velModelStdev.getValue()[0];
                }else{
                    velModStDev[i]=velModelStdev.getValue()[i];
                }
            }
        }

        helper::vector<FilterType> paramModelStDev = paramModelStdev.getValue();
        this->modelElementNoise = EVectorX::Zero(this->stateSize);

        size_t kp= 0;
        size_t kv= 0;
        size_t k= 0;
        const size_t N =freeNodes.size();
        const size_t M =posDim;
        const size_t V =velDim;

        EVectorX vecPosModelStDev;
        if (estimatePosition.getValue()) {
            vecPosModelStDev.resize(N*M);
            for (size_t i = 0; i < N; i++ ){
                for (size_t j = 0; j < M; j ++)
                    vecPosModelStDev(i*M+j) = posModStDev[j%M];
            }
        }

        EVectorX vecVelModelStDev;
        if (estimateVelocity.getValue()) {
            vecVelModelStDev.resize(N*V);
            for (size_t i = 0; i < N; i++){
                for (size_t j = 0; j < V; j ++)
                    vecVelModelStDev(i*V+j) = velModStDev[j%V];
            }
        }

        if (estimatePosition.getValue() && estimateVelocity.getValue()){
            for (unsigned index = 0; index < this->positionVariance.size(); index++, kp++)
                this->modelElementNoise(index) = vecPosModelStDev(kp)  ;
            for (size_t index = this->positionVariance.size(); index < this->reducedStateIndex; index++,kv++)
                this->modelElementNoise(index) = vecVelModelStDev[kv]  ;
            for (size_t pi = this->reducedStateIndex; pi < this->stateSize; pi++,k++)
                if (paramModelStdev.isSet() == 0){
                    this->modelElementNoise(pi) = 0;
                } else {
                    this->modelElementNoise(pi) = paramModelStDev[k] * paramModelStDev[k];    /// Non-null Q for Force
                }
        }

        //NORMALLY IS THE CASE OF DATA ASSIMILATION where Parameters have no Q
        if (estimatePosition.getValue() && !estimateVelocity.getValue()){
            for (unsigned index = 0; index < this->positionVariance.size(); index++, kp++)
                this->modelElementNoise(index) = vecPosModelStDev(kp);
            for (size_t pi = this->reducedStateIndex; pi < this->stateSize; pi++,k++)
                this->modelElementNoise(pi) = 0;
        }

    }
    return this->modelElementNoise;

}


template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::updateModelErrorVariance() {
    PRNS("Constructing state co-variance matrix")
    int oldModelVarianceSize = this->stateErrorVariance.rows();
    modelErrorVariance.conservativeResize(this->stateSize, this->stateSize);

    // set zeros to new added elements
    for (int first_index = 0; first_index < modelErrorVariance.rows(); first_index++) {
        for (int second_index = oldModelVarianceSize; second_index < modelErrorVariance.rows(); second_index++) {
            modelErrorVariance(first_index, second_index) = 0.0;
            modelErrorVariance(second_index, first_index) = 0.0;
        }
    }
}

template <class DataTypes, class FilterType>
typename StochasticStateWrapper<DataTypes, FilterType>::EMatrixX& StochasticStateWrapper<DataTypes, FilterType>::getStateErrorVarianceReduced() {
    if (this->stateErrorVarianceReduced.rows() == 0) {
        this->stateErrorVarianceReduced.resize(this->reducedStateSize,this->reducedStateSize);
        this->stateErrorVarianceReduced.setZero();

        size_t vpi = 0;
        for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
            helper::vector<double> variance;
            this->vecOptimParams[opi]->getInitVariance(variance);

            for (size_t pi = 0; pi < this->vecOptimParams[opi]->size(); pi++, vpi++) {
                this->stateErrorVarianceReduced(vpi,vpi) = Type(Type(1.0) / variance[pi]);
            }
        }
    }
    return this->stateErrorVarianceReduced;
}

template <class DataTypes, class FilterType>
typename StochasticStateWrapper<DataTypes, FilterType>::EMatrixX& StochasticStateWrapper<DataTypes, FilterType>::getStateErrorVarianceProjector() {
    if (this->stateErrorVarianceProjector.rows() == 0) {
        this->stateErrorVarianceProjector.resize(this->stateSize, this->reducedStateSize);
        this->stateErrorVarianceProjector.setZero();

        for (size_t i = 0; i < this->reducedStateSize; i++)
            this->stateErrorVarianceProjector(i+this->reducedStateIndex,i) = Type(1.0);
    }
    return this->stateErrorVarianceProjector;
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
    if (! (this->filterKind == CLASSIC || this->filterKind == REDORD || this->filterKind == LOCENSEMBLE) )
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

    if (this->filterKind == CLASSIC || this->filterKind == LOCENSEMBLE) {
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

        if ( mappedState != NULL)  {
            Vec3dTypes::VecCoord actualMappedPos(this->mappedMStateSize);
            typename MappedMechanicalState::ReadVecCoord mapPos = mappedState->readPositions();

            for (size_t i = 0; i < this->mappedMStateSize; i++) {
                actualMappedPos[i] = mapPos[i];
            }
            sigmaMappedStatePos.push_back(actualMappedPos);
        }

        if (_stateID != nullptr) {
            *_stateID = sigmaStatePos.size() - 1;
        }
    } else if (this->filterKind == REDORD) {
        /// only for visualization purposes, save the results of simulation step
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
    }


    if ((_stateID == nullptr) || (*_stateID >= 0)) {
        this->state = savedState;
        copyStateFilter2Sofa(_mparams);
    }
}
template <class DataTypes, class FilterType>
void StochasticStateWrapper<DataTypes, FilterType>::lastApplyOperator(EVectorX& /* _vecX */, const core::MechanicalParams* /* _mparams */) {
    if (! (this->filterKind == CLASSIC || this->filterKind == LOCENSEMBLE) )
        return;

//    this->state = _vecX;
//    reinitMState(_mparams);

//    copyStateFilter2Sofa(_mparams);

//    if (this->d_langrangeMultipliers.getValue())
//        computeSofaStepWithLM(_mparams);
//    else
//        computeSofaStep(_mparams, false);
//    copyStateSofa2Filter()

    typename MechanicalState::ReadVecCoord Initpos = mechanicalState->readPositions();
    typename MechanicalState::ReadVecDeriv vel = mechanicalState->readVelocities();
    typename MechanicalState::WriteVecCoord pos = mechanicalState->writePositions();
    for (size_t i = 0; i < this->mStateSize; i++) {
        pos[i] = Initpos[i] +  vel[i]*this->gnode->getDt() ;
    }

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

                helper::types::RGBAColor color;

                switch (i) {
                case 0: color = helper::types::RGBAColor(1.0,0.0,0.0,1.0); break;
                case 1: color = helper::types::RGBAColor(0.0,1.0,0.0,1.0); break;
                case 2: color = helper::types::RGBAColor(0.0,0.0,1.0,1.0); break;
                default: color = helper::types::RGBAColor(0.5, 0.5, 0.5, 0.5);
                }

                vparams->drawTool()->setPolygonMode(0,vparams->displayFlags().getShowWireFrame());
                vparams->drawTool()->setLightingEnabled(true); //Enable lightning
                vparams->drawTool()->drawSpheres(points, d_radius_draw.getValue(), color); // sofa::defaulttype::Vec<4, float>(color[i],0.8f,colorB[i],1.0f));
                vparams->drawTool()->setPolygonMode(0,false);
                vparams->drawTool()->drawLineStrip(points, 3.0, helper::types::RGBAColor(color[i],0.8f,colorB[i],1.0f));

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
    //std::cout << "[" << this->getName() << "]: update internal data" << std::endl;
    sofa::simulation::UpdateInternalDataVisitor uid(execParams);
    this->gnode->execute ( uid );
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

    core::ConstraintParams cparams(*params);
    cparams.setX(freePos);
    cparams.setV(freeVel);
    cparams.setDx(constraintSolver->getDx());
    cparams.setLambda(constraintSolver->getLambda());
    cparams.setOrder(m_solveVelocityConstraintFirst.getValue() ? core::ConstraintParams::VEL : core::ConstraintParams::POS_AND_VEL);

    {
        core::behavior::MultiVecDeriv dx(&vop, core::VecDerivId::dx()); dx.realloc(&vop, true, true);
        core::behavior::MultiVecDeriv df(&vop, core::VecDerivId::dforce()); df.realloc(&vop, true, true);
    }


    // This solver will work in freePosition and freeVelocity vectors.
    // We need to initialize them if it's not already done.
    sofa::helper::AdvancedTimer::stepBegin("MechanicalVInitVisitor");
    simulation::MechanicalVInitVisitor< core::V_COORD >(params, core::VecCoordId::freePosition(), core::ConstVecCoordId::position(), true).execute(this->gnode);
    simulation::MechanicalVInitVisitor< core::V_DERIV >(params, core::VecDerivId::freeVelocity(), core::ConstVecDerivId::velocity(), true).execute(this->gnode);
    sofa::helper::AdvancedTimer::stepEnd("MechanicalVInitVisitor");

    {
        sofa::helper::AdvancedTimer::stepBegin("AnimateBeginEvent");
        AnimateBeginEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        this->gnode->execute ( act );
        sofa::helper::AdvancedTimer::stepEnd("AnimateBeginEvent");
    }

    sofa::simulation::BehaviorUpdatePositionVisitor beh(params , dt);

    using helper::system::thread::CTime;
    using sofa::helper::AdvancedTimer;

    // double timeScale = 1000.0 / (double)CTime::getTicksPerSec();

    // Update the BehaviorModels
    // Required to allow the RayPickInteractor interaction
    dmsg_info() << "updatePos called" ;

    AdvancedTimer::stepBegin("UpdatePosition");
    this->gnode->execute(&beh);
    AdvancedTimer::stepEnd("UpdatePosition");

    dmsg_info() << "updatePos performed - updateInternal called" ;

    sofa::simulation::UpdateInternalDataVisitor iud(params);

    dmsg_info() << "updateInternal called" ;

    {
        AdvancedTimer::stepBegin("updateInternalData");
        this->gnode->execute(&iud);
        AdvancedTimer::stepEnd("updateInternalData");
    }

    dmsg_info() << "updatePos performed - beginVisitor called" ;


    sofa::simulation::MechanicalBeginIntegrationVisitor beginVisitor(params, dt);
    this->gnode->execute(&beginVisitor);

    dmsg_info() << "beginVisitor performed - SolveVisitor for freeMotion is called" ;


    // Mapping geometric stiffness coming from previous lambda.
    {
        simulation::MechanicalVOpVisitor lambdaMultInvDt(params, cparams.lambda(), sofa::core::ConstMultiVecId::null(), cparams.lambda(), 1.0 / dt);
        lambdaMultInvDt.setMapped(true);
        this->getContext()->executeVisitor(&lambdaMultInvDt);
        simulation::MechanicalComputeGeometricStiffness geometricStiffnessVisitor(&mop.mparams, cparams.lambda());
        this->getContext()->executeVisitor(&geometricStiffnessVisitor);
    }

    // Free Motion
    sofa::helper::AdvancedTimer::stepBegin("Step2 FreeMotion");
    simulation::SolveVisitor freeMotion(params, dt, true);
    this->gnode->execute(&freeMotion);
    sofa::helper::AdvancedTimer::stepEnd("Step2 FreeMotion");


    mop.projectResponse(freeVel);
    mop.propagateDx(freeVel, true);

    if (cparams.constOrder() == core::ConstraintParams::POS ||
        cparams.constOrder() == core::ConstraintParams::POS_AND_VEL)
    {
        // xfree = x + vfree*dt
        simulation::MechanicalVOpVisitor freePosEqPosPlusFreeVelDt(params, freePos, pos, freeVel, dt);
        freePosEqPosPlusFreeVelDt.setMapped(true);
        this->getContext()->executeVisitor(&freePosEqPosPlusFreeVelDt);
    }
    dmsg_info() << " SolveVisitor for freeMotion performed" ;

    // Collision detection and response creation

    sofa::helper::AdvancedTimer::stepBegin("Step3 ComputeCollision");
    this->computeCollision(params);
    sofa::helper::AdvancedTimer::stepEnd("Step3 ComputeCollision");

    // Solve constraints
    if (constraintSolver)
    {
        sofa::helper::AdvancedTimer::stepBegin("ConstraintSolver");

        if (cparams.constOrder() == core::ConstraintParams::VEL )
        {
            constraintSolver->solveConstraint(&cparams, vel);

            // x_t+1 = x_t + ( vfree + dv ) * dt
            pos.eq(pos, vel, dt);
        }
        else
        {
            constraintSolver->solveConstraint(&cparams, pos, vel);
        }

        core::behavior::MultiVecDeriv dx(&vop, constraintSolver->getDx());
        mop.projectResponse(dx);
        mop.propagateDx(dx, true);

        sofa::helper::AdvancedTimer::stepEnd("ConstraintSolver");

    }

    simulation::MechanicalEndIntegrationVisitor endVisitor(params, dt);
    this->gnode->execute(&endVisitor);

    mop.projectPositionAndVelocity(pos, vel);
    mop.propagateXAndV(pos, vel);

    this->gnode->setTime ( startTime + dt );
    this->gnode-> template execute<UpdateSimulationContextVisitor>(params);  // propagate time

    {
        AnimateEndEvent ev ( dt );
        PropagateEventVisitor act ( params, &ev );
        this->gnode->execute ( act );
    }


    sofa::helper::AdvancedTimer::stepBegin("UpdateMapping");
    //Visual Information update: Ray Pick add a MechanicalMapping used as VisualMapping
    this->gnode->template execute<UpdateMappingVisitor>(params);
//	sofa::helper::AdvancedTimer::step("UpdateMappingEndEvent");
    {
        UpdateMappingEndEvent ev ( dt );
        PropagateEventVisitor act ( params , &ev );
        this->gnode->execute ( act );
    }
    sofa::helper::AdvancedTimer::stepEnd("UpdateMapping");



#ifndef SOFA_NO_UPDATE_BBOX
    this->gnode->template execute<UpdateBoundingBoxVisitor>(params);
#endif

    sofa::helper::AdvancedTimer::stepEnd("MechanicalVInitVisitor");

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

