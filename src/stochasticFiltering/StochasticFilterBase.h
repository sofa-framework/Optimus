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

#include <sofa/simulation/UpdateContextVisitor.h>

#include <sofa/simulation/Node.h>

#include "initOptimusPlugin.h"
#include "StochasticStateWrapperBase.h"
#include "ObservationManagerBase.h"

namespace sofa
{

namespace component
{

namespace stochastic
{

using namespace defaulttype;

/// ganaeral filter interface
class StochasticFilterBase : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(StochasticFilterBase, BaseObject);
    typedef sofa::core::objectmodel::BaseObject Inherit;

    StochasticFilterBase()
        : Inherit()        
        , mechParams(0)
        , verbose( initData(&verbose, false, "verbose", "print tracing informations") )
        , useUnbiasedVariance( initData(&useUnbiasedVariance, false, "useUnbiasedVariance", "if true, the unbiased variance is computed (normalization by 1/(n-1) [not activated for UKFClassic!") )
        //, useModelIncertitude( initData(&useModelIncertitude, false, "useModelIncertitude", "if true, the state covariance is computed by adding Q") )
        , initialiseObservationsAtFirstStep( initData(&initialiseObservationsAtFirstStep, false, "initialiseObservationsAtFirstStep", "if true initialise component during first iteration") )
    {

    }

    ~StochasticFilterBase() {}

protected:
    sofa::helper::system::thread::CTime *timer;
    double startTime, stopTime;

    sofa::simulation::Node* gnode;    
    size_t stepNumber;
    double actualTime;
    const core::ExecParams* execParams;
    const core::MechanicalParams* mechParams;

public:
    Data<bool> verbose;
    Data<bool> useUnbiasedVariance;
    //Data<bool> useModelIncertitude;
    Data<bool> initialiseObservationsAtFirstStep;


    void init() override {
        Inherit::init();
        gnode = dynamic_cast<sofa::simulation::Node*>(this->getContext());

        if (!gnode) {
            PRNE("Cannot find node!");         
        }

        stepNumber = 0;
        actualTime = 0.0;
    }

    virtual void initializeStep(const core::ExecParams* _params, const size_t _step) {
        if (mechParams==0)
            mechParams = new core::MechanicalParams(*_params);
        execParams=_params;
        stepNumber=_step;
        actualTime = double(stepNumber)*gnode->getDt();
        this->gnode->setTime(this->actualTime);
        this->gnode->execute< sofa::simulation::UpdateSimulationContextVisitor >(execParams);
    }

    virtual void computePrediction() = 0;
    virtual void computeCorrection() = 0;

    // change filter state
    virtual void updateState() = 0;

}; /// class


/// ganeral interface for uncertain filter types
class StochasticUnscentedFilterBase : public StochasticFilterBase
{
public:
    SOFA_ABSTRACT_CLASS(StochasticUnscentedFilterBase, StochasticFilterBase);
    typedef StochasticFilterBase Inherit;

    StochasticUnscentedFilterBase()
        : Inherit()
        , reducedOrder( initData(&reducedOrder, false, "reducedOrder", "reduced order type of the filter") )
        , lambdaScale( initData(&lambdaScale, 0.5, "lambdaScale", "scaling for sigma points") )
        , d_sigmaTopologyType( initData(&d_sigmaTopologyType, "sigmaTopology", "sigma topology type") )
    {

    }

    ~StochasticUnscentedFilterBase() {}

protected:
    typedef enum SigmaTopology {
        SIMPLEX = 0,
        STAR = 1
    } SigmaTopologyType;

    SigmaTopologyType m_sigmaTopology;

public:
    Data<bool> reducedOrder;
    Data<double> lambdaScale;
    Data< std::string > d_sigmaTopologyType;

    void init() override {
        Inherit::init();

        m_sigmaTopology = SIMPLEX;
        std::string topology = d_sigmaTopologyType.getValue();
        if (std::strcmp(topology.c_str(), "Star") == 0)
            m_sigmaTopology = STAR;
    }

    virtual void initializeStep(const core::ExecParams* _params, const size_t _step) {
        Inherit::initializeStep(_params, _step);
    }

    virtual void computePrediction() = 0;
    virtual void computeCorrection() = 0;

    // change filter state
    virtual void updateState() = 0;

}; /// class


} // stochastic

} // component

} // sofa

