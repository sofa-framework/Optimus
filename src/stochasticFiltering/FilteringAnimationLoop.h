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

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseAnimationLoop.h>
#include <sofa/core/ExecParams.h>
#include <SofaSimulationCommon/config.h>
#include <sofa/simulation/Node.h>
#include <sofa/helper/AdvancedTimer.h>

#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/Visitor.h>
#include <sofa/simulation/PropagateEventVisitor.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include "initOptimusPlugin.h"
#include "StochasticFilterBase.h"
#include "PreStochasticWrapper.h"
#include "../genericComponents/FilterEvents.h"
#include "../genericComponents/TimeProfiling.h"



namespace sofa
{

namespace component
{

namespace stochastic
{



class SingleLevelEventVisitor : public sofa::simulation::Visitor
{
protected:
    sofa::core::objectmodel::Event* m_event;
    sofa::simulation::Node* m_node;


public:
    SingleLevelEventVisitor(const core::ExecParams* params, sofa::core::objectmodel::Event* e, sofa::simulation::Node* node);
    ~SingleLevelEventVisitor();

    Visitor::Result processNodeTopDown(sofa::simulation::Node* node);
    void processObject(sofa::simulation::Node*, core::objectmodel::BaseObject* obj);

    virtual const char* getClassName() const { return "PropagateEventVisitor"; }
    virtual std::string getInfos() const { return std::string(m_event->getClassName());  }
};



/** Class which implements the main loop of SOFA simulation: requires stochastic filter that will be called
 *  It requires filter component on the same level (only one filter is supposed in the scene for now.
 *  The filter API is defined in StochasticFilterBase.
 */
class FilteringAnimationLoop : public sofa::core::behavior::BaseAnimationLoop
{
public:
    SOFA_CLASS(FilteringAnimationLoop,sofa::core::behavior::BaseAnimationLoop);

    typedef sofa::core::behavior::BaseAnimationLoop Inherit;
    typedef sofa::component::stochastic::StochasticFilterBase StochasticFilterBase;

protected:    
    sofa::helper::system::thread::CTime *timer;
    double startTime, stopTime;

    size_t actualStep;
    type::vector<stochastic::PreStochasticWrapper*> preStochasticWrappers;

    int numStep;
    sofa::component::stochastic::TimeProfiling m_timeProfiler;

    StochasticFilterBase* filter;

public:        
    sofa::simulation::Node* gnode;
    Data<bool> verbose;
    core::objectmodel::DataFileName d_timeDataFile;

    FilteringAnimationLoop();
    FilteringAnimationLoop(sofa::simulation::Node* _gnode);
    ~FilteringAnimationLoop() {}

    virtual void step(const core::ExecParams* _params, SReal _dt) override;

    void init() override;
    void bwdInit() override;
};



} // stochastic

} // component

} // sofa

