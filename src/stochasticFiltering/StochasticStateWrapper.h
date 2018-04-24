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

#include <SofaBaseLinearSolver/FullMatrix.h>

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
template<class DataTypes>
class InternalCopy {
public :

    typedef typename DataTypes::VecCoord VecCoord;

    void copyStateToFilter(helper::vector<std::pair<size_t, size_t> > & pairs, const VecCoord & ) {}

    void copyFilterToSofa(helper::vector<std::pair<size_t, size_t> > & pairs, VecCoord & ) {}

    void stateDim() {}

};

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

    typedef typename sofa::component::linearsolver::FullMatrix<FilterType> FullMatrix;

    enum { Dim = Coord::spatial_dimensions};


    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;
//    typedef typename core::behavior::MechanicalState<DataTypes> MappedMechanicalState;
    typedef typename component::projectiveconstraintset::FixedConstraint<DataTypes> FixedConstraint;
    typedef sofa::component::container::OptimParamsBase OptimParamsBase;

    typedef typename Inherit::EMatrixX EMatrixX;
    typedef typename Inherit::EVectorX EVectorX;

    StochasticStateWrapper();
    ~StochasticStateWrapper();

protected:
    MechanicalState *mechanicalState;
//    MappedMechanicalState *mappedState;
    FixedConstraint* fixedConstraint;
    helper::vector<OptimParamsBase*> vecOptimParams;
    InternalCopy<DataTypes> m_internalCopy;
    size_t posDim;
    size_t velDim;


    VecCoord beginTimeStepPos;
    VecDeriv beginTimeStepVel;
    VecCoord beginTimeStepMappedPos;

    helper::vector<VecCoord> sigmaStatePos;
    helper::vector<VecDeriv> sigmaStateVel;
    helper::vector<VecCoord> sigmaMappedStatePos;

    bool valid;
    helper::vector<size_t> fixedNodes, freeNodes;
    helper::vector<std::pair<size_t, size_t> > positionPairs;
    helper::vector<std::pair<size_t, size_t> > velocityPairs;

    void copyStateFilter2Sofa(const core::MechanicalParams *_mechParams, bool _setVelocityFromPosition = false);  // copy actual DA state to SOFA state and propagate to mappings
    void copyStateSofa2Filter();  // copy the actual SOFA state to DA state
    void computeSofaStep(const core::ExecParams* execParams, bool _updateTime);
    void computeSofaStepWithLM(const core::ExecParams* params);

public:
    Data<bool> d_langrangeMultipliers;    
    Data<bool> estimatePosition;
    Data<bool> estimateVelocity;
    Data<FilterType>  posModelStdev, velModelStdev;
    Data<helper::vector<FilterType>> paramModelStdev;
    Data<double> d_positionStdev;  /// standart deviation for positions
    Data<double> d_velocityStdev;  /// standart deviation for velocities
    Data <std::string> d_mappedStatePath;
    Data< bool  > d_draw;
    Data< double  > d_radius_draw;
    Data<FullMatrix> d_fullMatrix;

    EMatrixX modelErrorVariance;
    EMatrixX modelErrorVarianceInverse;
    FilterType modelErrorVarianceValue;

    void stateDim();

    void init();
    void bwdInit();

    void transformState(EVectorX& _vecX, const core::MechanicalParams* _mparams, int* _stateID);
    void lastApplyOperator(EVectorX& _vecX, const core::MechanicalParams* _mparams);

    //void setSofaTime(const core::ExecParams* _execParams);
    void computeSimulationStep(EVectorX& _state, const core::MechanicalParams* mparams,  int& _stateID);
    void initializeStep(size_t _stepNumber);
    void storeMState();
    void reinitMState(const core::MechanicalParams* _mechParams);
    void getActualPosition(int _id, VecCoord& _pos);
    void getActualMappedPosition(/*int _id, VecCoord& _mapPos*/);
    void draw(const core::visual::VisualParams* vparams);

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
            }
        }
        return this->stateErrorVariance;
    } 


    virtual EMatrixX& getModelErrorVariance() {
        if (this->modelErrorVariance.rows() == 0) {

            FilterType posModelStDev = posModelStdev.getValue();
            FilterType velModelStDev = velModelStdev.getValue();
            helper::vector<FilterType> paramModelStDev = paramModelStdev.getValue();
            modelErrorVarianceValue = posModelStDev * posModelStDev;
            modelErrorVariance = EMatrixX::Identity(this->stateSize, this->stateSize)*modelErrorVarianceValue;
            modelErrorVarianceInverse = EMatrixX::Identity(this->stateSize, this->stateSize) / modelErrorVarianceValue;

            for (size_t pi = this->reducedStateIndex; pi < this->stateSize; pi++){
                    this->modelErrorVariance(pi,pi) = 0; /// Q is zero for parameters
            }

            std::cout<< "posModelStDev" << paramModelStdev.getValue() <<std::endl;
            size_t k= 0;
            if (estimatePosition.getValue() && estimateVelocity.getValue()){
                modelErrorVariance = EMatrixX::Identity(this->stateSize, this->stateSize);
                for (unsigned index = 0; index < this->positionVariance.size(); index++)
                    modelErrorVariance(index,index) = posModelStDev*posModelStDev  ;

                for (size_t index = this->positionVariance.size(); index < this->reducedStateIndex; index++)
                    modelErrorVariance(index,index) = velModelStDev*velModelStDev  ;

                for (size_t pi = this->reducedStateIndex; pi < this->stateSize; pi++,k++)
                    modelErrorVariance(pi,pi) = paramModelStDev[k]  *paramModelStDev[k];
            }

            if (!estimatePosition.getValue() && estimateVelocity.getValue()){
                modelErrorVariance = EMatrixX::Identity(this->stateSize, this->stateSize);

                for (size_t index = 0; index < this->reducedStateIndex; index++)
                    modelErrorVariance(index,index) = velModelStDev*velModelStDev  ;

                for (size_t pi = this->reducedStateIndex; pi < this->stateSize; pi++,k++)
                    modelErrorVariance(pi,pi) = paramModelStDev[k]  *paramModelStDev[k];
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

    Data<bool> m_solveVelocityConstraintFirst;

    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        //        if (dynamic_cast<MState *>(context->getMechanicalState()) == NULL) return false;
        return sofa::core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }
    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const StochasticStateWrapper<DataTypes, FilterType>* = NULL)
    {
        return DataTypes::Name();
    }

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


