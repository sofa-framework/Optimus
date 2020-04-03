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
#ifndef FILTERINGANIMATIONLOOP_H_
#define FILTERINGANIMATIONLOOP_H_

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseAnimationLoop.h>
#include <sofa/core/ExecParams.h>
#include <SofaSimulationCommon/common.h>
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

using namespace defaulttype;

class SingleLevelEventVisitor : public sofa::simulation::Visitor
{
public:
    SingleLevelEventVisitor(const core::ExecParams* params, sofa::core::objectmodel::Event* e, sofa::simulation::Node* node);

    ~SingleLevelEventVisitor();

    Visitor::Result processNodeTopDown(sofa::simulation::Node* node);
    void processObject(sofa::simulation::Node*, core::objectmodel::BaseObject* obj);

    virtual const char* getClassName() const { return "PropagateEventVisitor"; }
    virtual std::string getInfos() const { return std::string(m_event->getClassName());  }
protected:
    sofa::core::objectmodel::Event* m_event;
    sofa::simulation::Node* m_node;
};

/** Class which implements the main loop of SOFA simulation: requires stochastic filter that will be called
 *  It requires filter component on the same level (only one filter is supposed in the scene for now.
 *  The filter API is defined in StochasticFilterBase.
 */

class FilteringAnimationLoop: public sofa::core::behavior::BaseAnimationLoop
{
public:
    SOFA_CLASS(FilteringAnimationLoop,sofa::core::behavior::BaseAnimationLoop);

    typedef sofa::core::behavior::BaseAnimationLoop Inherit;
    typedef sofa::component::stochastic::StochasticFilterBase StochasticFilterBase;

    ~FilteringAnimationLoop() {}

    FilteringAnimationLoop();
    FilteringAnimationLoop(sofa::simulation::Node* _gnode);


virtual void step(const core::ExecParams* _params, SReal _dt) override;

protected:    
    sofa::helper::system::thread::CTime *timer;
    double startTime, stopTime;

    size_t actualStep;
    helper::vector<stochastic::PreStochasticWrapper*> preStochasticWrappers;


    int numStep;
    sofa::component::stochastic::TimeProfiling m_timeProfiler;

    StochasticFilterBase* filter;

public:        
    sofa::simulation::Node* gnode;
    Data<bool> verbose;
    core::objectmodel::DataFileName d_timeDataFile;

    void init() override;
    void bwdInit() override;




}; /// class


} // stochastic
} // component
} // sofa

#endif // FILTERINGANIMATIONLOOP_H


