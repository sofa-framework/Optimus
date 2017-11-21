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
#ifndef SOFASTATEOBSERVATIONWRAPPER_H_
#define SOFASTATEOBSERVATIONWRAPPER_H_

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
class StochasticStateObservationWrapper: public sofa::component::stochastic::StochasticStateWrapperBaseT<FilterType>
{

public:
    SOFA_CLASS(SOFA_TEMPLATE2(StochasticStateObservationWrapper, DataTypes, FilterType), SOFA_TEMPLATE(StochasticStateWrapperBaseT, FilterType));

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

    StochasticStateObservationWrapper();
    ~StochasticStateObservationWrapper();

protected:
    MechanicalState *mechanicalState;
    FixedConstraint* fixedConstraint;
    helper::vector<OptimParamsBase*> vecOptimParams;

    bool valid;
    helper::vector<size_t> fixedNodes, freeNodes;
    helper::vector<std::pair<size_t, size_t> > positionPairs;
    helper::vector<std::pair<size_t, size_t> > velocityPairs;
    helper::vector<std::pair<size_t, size_t> > externalForcesPairs;

    EVectorX obsState;

    helper::vector<std::pair<size_t, size_t> > observationPosPairs;

    void copyStateFilter2Sofa(const core::MechanicalParams *_mechParams);  // copy actual DA state to SOFA state and propagate to mappings
    void copyStateSofa2Filter();  // copy the actual SOFA state to DA state
    void computeSofaStep(const core::ExecParams* execParams, bool _updateTime);
    void computeSofaStepWithLM(const core::ExecParams* params, bool _updateTime);

public:
    Data<bool> d_langrangeMultipliers;
    Data<bool> estimatePosition;
    Data<bool> estimateVelocity;
    Data<bool> estimateExternalForces;
    Data<bool> optimForces;

    Data< double> m_stdev;          /// standard deviation

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

    void applyOperator(EVectorX& _vecX, const core::MechanicalParams* _mparams, bool _preserveState, bool _updateForce);
    //void setSofaTime(const core::ExecParams* _execParams);

    void setState(EVectorX& _state, const core::MechanicalParams* _mparams) {
        this->state = _state;
        copyStateFilter2Sofa(_mparams);
    }

    void setSofaVectorFromFilterVector(EVectorX& _state, typename DataTypes::VecCoord& _vec);
    void setSofaVectorFromObservationsStateVector(EVectorX& _observationsState, typename DataTypes::VecCoord& _vec);

    virtual EVectorX& getStateForObservations() {
        return obsState;
    }

    virtual EMatrixX& getStateErrorVariance() {
        if (this->stateErrorVariance.rows() == 0) {
            this->stateErrorVariance.resize(this->stateSize, this->stateSize);
            this->stateErrorVariance.setZero();

            size_t vpi = 0;
            for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
                helper::vector<double> stdev;
                this->vecOptimParams[opi]->getStDev(stdev);

                PRNS("stdev: " << stdev);
                for (size_t pi = 0; pi < this->vecOptimParams[opi]->size(); pi++, vpi++)
                    this->stateErrorVariance(vpi,vpi) = Type(Type(1.0) / (stdev[pi] * stdev[pi]));
            }
        }
        return this->stateErrorVariance;
    }

    virtual EMatrixX& getStateErrorVarianceDevUKF() {
        if (this->stateErrorVariance.rows() == 0) {
            this->stateErrorVariance.resize(this->stateSize, this->stateSize);
            this->stateErrorVariance.setZero();

            size_t vpi = 0;
            for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
                helper::vector<double> stdev;
                this->vecOptimParams[opi]->getStDev(stdev);

                PRNS("stdev: " << stdev);
                for (size_t pi = 0; pi < this->vecOptimParams[opi]->size(); pi++, vpi++)
                    this->stateErrorVariance(vpi,vpi) = Type(stdev[pi] * stdev[pi]);
            }
        }
        return this->stateErrorVariance;
    }

    virtual EMatrixX& getStateErrorVarianceUKF() {
        if (this->stateErrorVariance.rows() == 0) {
            this->stateErrorVariance.resize(this->stateSize, this->stateSize);
            this->stateErrorVariance.setZero();

            for (size_t pi = 0; pi < this->stateSize; pi++)
                this->stateErrorVariance(pi,pi) = Type((m_stdev.getValue()* m_stdev.getValue()));
        }
        return this->stateErrorVariance;
    }

    /// get the state error variant for the reduced order filters (stdev^2 of the parameters being estimated)
    virtual EMatrixX& getStateErrorVarianceReduced() {
        if (this->stateErrorVarianceReduced.rows() == 0) {
            this->stateErrorVarianceReduced.resize(this->reducedStateSize,this->reducedStateSize);
            this->stateErrorVarianceReduced.setZero();

            size_t vpi = 0;
            for (size_t opi = 0; opi < this->vecOptimParams.size(); opi++) {
                helper::vector<double> stdev;
                this->vecOptimParams[opi]->getStDev(stdev);

                for (size_t pi = 0; pi < this->vecOptimParams[opi]->size(); pi++, vpi++)
                    this->stateErrorVarianceReduced(vpi,vpi) = Type(Type(1.0) / (stdev[pi] * stdev[pi]));
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

#endif // SOFASTATEOBSERVATIONWRAPPER_H


