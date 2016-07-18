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
#ifndef STOCHASTICFILTER_H_
#define STOCHASTICFILTER_H_

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
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

class StochasticFilterBase: public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(StochasticFilterBase, BaseObject);
    typedef sofa::core::objectmodel::BaseObject Inherit;

    StochasticFilterBase()
        : Inherit()        
        , mechParams(0)
        , verbose( initData(&verbose, false, "verbose", "print tracing informations") ) {
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

    void init() {
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


}; /// class


} // stochastic
} // component
} // sofa

#endif // STOCHASTICFILTER_H


