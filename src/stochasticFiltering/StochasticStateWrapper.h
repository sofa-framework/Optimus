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

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
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
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/IntegrateBeginEvent.h>
#include <sofa/simulation/IntegrateEndEvent.h>

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

/**
 * Class which implements an interface between a filter (working with Eigen vectors and matrices) and SOFA and its own data types as Vec3d, Rigid3d etc.).
 * Presence of the component StochasticStateWrapper in a SOFA scene node indicates that the physical simulation is employed for predictions.
 * It closely interacts with mechanical state (set/get mechanism) and OptimParams which is a container for storing stochastic quantities.
 */

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
    typedef typename core::behavior::MechanicalState<Vec3dTypes> MappedMechanicalState;
    typedef typename component::projectiveconstraintset::FixedConstraint<DataTypes> FixedConstraint;
    typedef sofa::component::container::OptimParamsBase OptimParamsBase;

    typedef typename Inherit::EMatrixX EMatrixX;
    typedef typename Inherit::EVectorX EVectorX;

    typedef defaulttype::Vec<4,float> Vec4f;

    StochasticStateWrapper();
    ~StochasticStateWrapper();

protected:
    MechanicalState *mechanicalState;
    MappedMechanicalState *mappedState;
    FixedConstraint* fixedConstraint;
    helper::vector<OptimParamsBase*> vecOptimParams;
    InternalCopy<DataTypes> m_internalCopy;
    size_t posDim;
    size_t velDim;
    sofa::component::topology::EdgeSetTopologyContainer* m_container;


    VecCoord beginTimeStepPos;
    VecDeriv beginTimeStepVel;
    Vec3dTypes::VecCoord beginTimeStepMappedPos;

    helper::vector<VecCoord> sigmaStatePos;
    helper::vector<VecDeriv> sigmaStateVel;
    helper::vector<Vec3dTypes::VecCoord> sigmaMappedStatePos;
    helper::vector<double> color,colorB;

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
    Data<bool> estimateOnlyXYZ;

    Data<bool> estimateVelocity;
    Data<helper::vector<FilterType>>  posModelStdev, velModelStdev;
    Data<helper::vector<FilterType>> paramModelStdev;
    Data<helper::vector<double>> d_positionStdev;  /// standart deviation for positions
    Data<helper::vector<double>> d_velocityStdev;  /// standart deviation for velocities
    Data <std::string> d_mappedStatePath;
    Data< bool  > d_draw;
    Data< double  > d_radius_draw;
    Data<FullMatrix> d_fullMatrix;
    Data<bool> m_solveVelocityConstraintFirst;

    EMatrixX modelErrorVariance;
    EMatrixX modelErrorVarianceInverse;
    FilterType modelErrorVarianceValue;

    /// initialization before the data assimilation starts
    void init() override;
    void bwdInit() override;

    virtual void updateState(bool addData = false) override;

    /// function to prepare data used during prediction and correction phase
    void initializeStep(size_t _stepNumber) override;

    /// functions calling one step of SOFA simulations in different manner given by the type of filter.
    void transformState(EVectorX& _vecX, const core::MechanicalParams* _mparams, int* _stateID) override;
    void lastApplyOperator(EVectorX& /* _vecX */, const core::MechanicalParams* /* _mparams */) override;
    void computeSimulationStep(EVectorX& _state, const core::MechanicalParams* mparams,  int& _stateID) override;

    /// set/get of data demand
    void stateDim();
    void storeMState();
    void reinitMState(const core::MechanicalParams* _mechParams);
    void getActualPosition(int _id, VecCoord& _pos);
    void getPos(EVectorX& _state, VecCoord& actualPos);

    void getActualVelocity(int _id, VecDeriv& _vel);
    void getActualMappedPosition(int _id, Vec3dTypes::VecCoord& _mapPos);    
    void setState(EVectorX& _state, const core::MechanicalParams* _mparams) override;

    void setSofaVectorFromFilterVector(EVectorX& _state, typename DataTypes::VecCoord& _vec);
    void setSofaVelocityFromFilterVector(EVectorX& _state, typename DataTypes::VecDeriv& _vel);

    /// get the variance of error of the state
    virtual EMatrixX& getStateErrorVariance() override;
    void updateStateErrorVariance();
    virtual EMatrixX& getModelErrorVariance() override;
    virtual EVectorX& getModelElementNoise() override;
    void updateModelErrorVariance();

    /// get the state error variant for the reduced order filters (stdev^2 of the parameters being estimated)
    virtual EMatrixX& getStateErrorVarianceReduced() override;
    virtual EMatrixX& getStateErrorVarianceProjector() override;

    void draw(const core::visual::VisualParams* vparams) override;

    /// SOFA-imposed methods for object factory

    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        //        if (dynamic_cast<MState *>(context->getMechanicalState()) == NULL) return false;
        return sofa::core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }
    virtual std::string getTemplateName() const override
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

