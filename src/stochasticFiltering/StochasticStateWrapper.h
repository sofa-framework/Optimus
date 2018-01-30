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
#ifndef SOFASTATEWRAPPERBASE_H_
#define SOFASTATEWRAPPERBASE_H_

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBoundaryCondition/FixedConstraint.h>
#include <sofa/simulation/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/SolveVisitor.h>
#include <sofa/simulation/VectorOperations.h>
#include <sofa/simulation/PropagateEventVisitor.h>
#include <sofa/simulation/UpdateContextVisitor.h>
#include <sofa/simulation/UpdateMappingVisitor.h>
#include <sofa/simulation/UpdateMappingEndEvent.h>
#include <sofa/simulation/UpdateBoundingBoxVisitor.h>

#include <SofaConstraint/LCPConstraintSolver.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/VecId.h>



#include <Eigen/Dense>

#include "initOptimusPlugin.h"
#include "StochasticStateWrapperBase.h"
#include "../genericComponents/OptimParams.h"
#include <fstream>

namespace sofa
{
namespace component
{
namespace stochastic
{

using namespace defaulttype;

template <class DataTypes, class FilterType>
class StochasticStateWrapper: public sofa::component::stochastic::StochasticStateWrapperBaseT<FilterType>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(StochasticStateWrapper, DataTypes, FilterType), SOFA_TEMPLATE(StochasticStateWrapperBaseT, FilterType));

    typedef sofa::component::stochastic::StochasticStateWrapperBaseT<FilterType> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Coord Deriv;
    typedef FilterType Type;

    enum { Dim = Coord::spatial_dimensions, DimForces = 6};


    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef typename component::projectiveconstraintset::FixedConstraint<DataTypes> FixedConstraint;
    typedef sofa::component::container::OptimParamsBase OptimParamsBase;

    typedef typename Inherit::EMatrixX EMatrixX;
    typedef typename Inherit::EVectorX EVectorX;

    StochasticStateWrapper();
    ~StochasticStateWrapper();

protected:
    MechanicalState *mechanicalState;
    FixedConstraint* fixedConstraint;
    helper::vector<OptimParamsBase*> vecOptimParams;

    VecCoord beginTimeStepPos;
    VecDeriv beginTimeStepVel;

    helper::vector<VecCoord> sigmaStatePos;
    helper::vector<VecDeriv> sigmaStateVel;

    bool valid;
    helper::vector<size_t> fixedNodes, freeNodes;
    helper::vector<std::pair<size_t, size_t> > positionPairs;
    helper::vector<std::pair<size_t, size_t> > velocityPairs;
    helper::vector<std::pair<size_t, size_t> > externalForcesPairs;

    void copyStateFilter2Sofa(const core::MechanicalParams *_mechParams, bool _setVelocityFromPosition = false);  // copy actual DA state to SOFA state and propagate to mappings
    void copyStateSofa2Filter();  // copy the actual SOFA state to DA state
    void computeSofaStep(const core::ExecParams* execParams, bool _updateTime);
    void computeSofaStepWithLM(const core::ExecParams* params, bool _updateTime);

public:
    Data<bool> d_langrangeMultipliers;    
    Data<bool> estimatePosition;
    Data<bool> estimateVelocity;
    Data<bool> estimateExternalForces;
    Data<bool> optimForces;
    Data<FilterType> modelStdev;
    EMatrixX modelErrorVariance;
    EMatrixX modelErrorVarianceInverse;
    FilterType modelErrorVarianceValue;
    Data<Mat3x4d> d_projectionMatrix;


    Data<double> d_positionStdev;  /// standart deviation for positions
    Data<double> d_velocityStdev;  /// standart deviation for velocities

    bool estimatingPosition() {
        return this->estimatePosition.getValue();
    }

    bool estimatingVelocity() {
        return this->estimateVelocity.getValue();
    }

    bool estimatingExternalForces() {
        return this->estimateExternalForces.getValue();
    }    

    void init();
    void bwdInit();

    void transformState(EVectorX& _vecX, const core::MechanicalParams* _mparams, int* _stateID);
    //void setSofaTime(const core::ExecParams* _execParams);
    void computeSimulationStep(EVectorX& _state, const core::MechanicalParams* mparams,  int& _stateID);
    void initializeStep(size_t _stepNumber);
    void storeMState();
    void reinitMState(const core::MechanicalParams* _mechParams);
    void getActualPosition(int _id, VecCoord& _pos);

    void setState(EVectorX& _state, const core::MechanicalParams* _mparams) {
        this->state = _state;
        copyStateFilter2Sofa(_mparams, true);
    }

    void setSofaVectorFromFilterVector(EVectorX& _state, typename DataTypes::VecCoord& _vec);


    virtual EMatrixX& getStateErrorVariance() {
        if (this->stateErrorVariance.rows() == 0) {
            PRNS("Constructing state co-variance matrix")
            this->stateErrorVariance.resize(this->stateSize, this->stateSize);
            this->stateErrorVariance.setZero();

            size_t vpi = 0;
            if (estimatePosition.getValue()) {
                for (size_t index = 0; index < (size_t)this->positionVariance.size(); index++, vpi++) {
                    this->stateErrorVariance(vpi,vpi) = this->positionVariance[index];
                }
            }

            /// vpi continues to increase since velocity is always after position
            if (estimateVelocity.getValue()) {
                for (size_t index = 0; index < (size_t)this->velocityVariance.size(); index++, vpi++) {
                    this->stateErrorVariance(vpi,vpi) = this->velocityVariance[index];
                }
            }


            for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
                helper::vector<double> variance;
                this->vecOptimParams[opi]->getInitVariance(variance);

                for (size_t pi = 0; pi < this->vecOptimParams[opi]->size(); pi++, vpi++)
                    this->stateErrorVariance(vpi,vpi) = variance[pi];
                    //this->stateErrorVariance(vpi,vpi) = Type(Type(1.0) / (stdev[pi] * stdev[pi]));
            }
        }
        return this->stateErrorVariance;
    } 


    virtual EMatrixX& getModelErrorVariance() {
        if (this->modelErrorVariance.rows() == 0) {

            FilterType modelStDev = modelStdev.getValue();
            modelErrorVarianceValue = modelStDev * modelStDev;
            modelErrorVariance = EMatrixX::Identity(this->stateSize, this->stateSize) * modelErrorVarianceValue;
            modelErrorVarianceInverse = EMatrixX::Identity(this->stateSize, this->stateSize) / modelErrorVarianceValue;

            size_t vpi = 0;
            for (size_t pi = this->positionVariance.size(); pi < this->stateSize; pi++){
                    this->modelErrorVariance(pi,pi) = 0; /// Q is zero for parameters
            }

            if (estimatePosition.getValue() && estimateVelocity.getValue()){
                modelErrorVariance = EMatrixX::Identity(this->stateSize, this->stateSize) ;
                for (size_t index = 0; index < (size_t)this->positionVariance.size(); index++, vpi++) {
                    modelErrorVariance(vpi,vpi) = this->positionVariance[index];
                }
                for (size_t index = this->positionVariance.size(); index < (size_t)this->stateSize; index++, vpi++) {
                    for (size_t indexV = 0; indexV < (size_t)this->velocityVariance.size(); indexV++)
                    modelErrorVariance(vpi,vpi) = this->velocityVariance[indexV];
                }
                for (size_t pi = (this->positionVariance.size()+this->velocityVariance.size()); pi < this->stateSize; pi++){
                        this->modelErrorVariance(pi,pi) = 0; /// Q is zero for parameters
                }
            }


        }
        return this->modelErrorVariance;
    }

    /// get the state error variant for the reduced order filters (stdev^2 of the parameters being estimated)
    virtual EMatrixX& getStateErrorVarianceReduced() {
        if (this->stateErrorVarianceReduced.rows() == 0) {
            this->stateErrorVarianceReduced.resize(this->reducedStateSize,this->reducedStateSize);
            this->stateErrorVarianceReduced.setZero();

            size_t vpi = 0;
            for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
                helper::vector<double> variance;
                this->vecOptimParams[opi]->getInitVariance(variance);

                for (size_t pi = 0; pi < this->vecOptimParams[opi]->size(); pi++, vpi++)
                    this->stateErrorVarianceReduced(vpi,vpi) = Type(Type(1.0) / variance[pi]);
            }
        }
        return this->stateErrorVarianceReduced;
    }

    virtual EMatrixX& getStateErrorVarianceProjector() {
        if (this->stateErrorVarianceProjector.rows() == 0) {
            this->stateErrorVarianceProjector.resize(this->stateSize, this->reducedStateSize);
            this->stateErrorVarianceProjector.setZero();

            for (size_t i = 0; i < this->reducedStateSize; i++)
                this->stateErrorVarianceProjector(i+this->reducedStateIndex,i) = Type(1.0);
        }
        return this->stateErrorVarianceProjector;
    }

    virtual EVectorX& getMinimumBound() {
        if (this->estimMinimumBound.rows() == 0) {
            size_t paramsSize = 0;
            for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
                paramsSize += this->vecOptimParams[opi]->size();
            }

            this->estimMinimumBound.resize(paramsSize);
            this->estimMinimumBound.setZero();
        }

        for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
            helper::vector<double> minimumBounds;
            this->vecOptimParams[opi]->getMinimumBounds(minimumBounds);

            for (size_t index = 0; index < this->vecOptimParams[opi]->size(); index++)
                this->estimMinimumBound(index) = minimumBounds[index];
        }
        return this->estimMinimumBound;
    }

    virtual EVectorX& getMaximumBound() {
        if (this->estimMaximumBound.rows() == 0) {
            size_t paramsSize = 0;
            for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
                paramsSize += this->vecOptimParams[opi]->size();
            }

            this->estimMaximumBound.resize(paramsSize);
            this->estimMaximumBound.setZero();
        }

        for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
            helper::vector<double> maximumBounds;
            this->vecOptimParams[opi]->getMaximumBounds(maximumBounds);

            for (size_t index = 0; index < this->vecOptimParams[opi]->size(); index++)
                this->estimMaximumBound(index) = maximumBounds[index];
        }
        return this->estimMaximumBound;
    }

    Data<bool> m_solveVelocityConstraintFirst;

protected :

    sofa::core::behavior::ConstraintSolver *constraintSolver;
    component::constraintset::LCPConstraintSolver::SPtr defaultSolver;


/****** ADDED FOR COLLISIONS  *****/
    /// Activate collision pipeline
    virtual void computeCollision(const core::ExecParams* params = core::ExecParams::defaultInstance());

    /// Activate OdeSolvers
    virtual void integrate(const core::ExecParams* params, SReal dt);


    //typedef simulation::Node::Sequence<core::behavior::OdeSolver> Solvers;
    //typedef core::collision::Pipeline Pipeline;
    //const Solvers& getSolverSequence();

    // the parent Node of CollisionAnimationLoop its self (usually, this parent node is the root node of the simulation graph)
    // This pointer is initialized one time at the construction, avoiding dynamic_cast<Node*>(context) every time step
    //simulation::Node* gnode;
/****** ADDED FOR COLLISIONS  *****/

}; /// class


} // simulation
} // component
} // sofa

#endif // SOFASTATEWRAPPERBASE_H


