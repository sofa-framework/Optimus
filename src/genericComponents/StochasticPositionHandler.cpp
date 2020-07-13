/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2020 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include <genericComponents/StochasticPositionHandler.inl>
#include <sofa/core/ObjectFactory.h>



namespace sofa
{

namespace component
{

namespace misc
{


SOFA_DECL_CLASS(StochasticPositionHandler)

using namespace defaulttype;



int StochasticPositionHandlerClass = core::RegisterObject("Write State vectors to file at each timestep")
    .add< StochasticPositionHandler >();


StochasticPositionHandlerCreator::StochasticPositionHandlerCreator(const core::ExecParams* params)
    :simulation::Visitor(params)
    , sceneName("")
    , recordX(true)
    , recordV(true)
    , createInMapping(false)
    , counterStochasticPositionHandler(0)
{
}

StochasticPositionHandlerCreator::StochasticPositionHandlerCreator(const core::ExecParams* params, const std::string &n, bool _recordX, bool _recordV, bool _recordF, bool _createInMapping, int c)
    :simulation::Visitor(params)
    , sceneName(n)
    , recordX(_recordX)
    , recordV(_recordV)
    , recordF(_recordF)
    , createInMapping(_createInMapping)
    , counterStochasticPositionHandler(c)
{
}


//Create a Write State component each time a mechanical state is found
simulation::Visitor::Result StochasticPositionHandlerCreator::processNodeTopDown( simulation::Node* gnode)
{
    sofa::core::behavior::BaseMechanicalState * mstate=gnode->mechanicalState;
    if (!mstate)   return simulation::Visitor::RESULT_CONTINUE;
    core::behavior::OdeSolver *isSimulated;
    mstate->getContext()->get(isSimulated);
    if (!isSimulated) return simulation::Visitor::RESULT_CONTINUE;

    //We have a mechanical state
    addStochasticPositionHandler(mstate, gnode);
    return simulation::Visitor::RESULT_CONTINUE;
}


void StochasticPositionHandlerCreator::addStochasticPositionHandler(sofa::core::behavior::BaseMechanicalState *ms, simulation::Node* gnode)
{
    sofa::core::objectmodel::BaseContext* context = gnode->getContext();
    sofa::core::BaseMapping *mapping;
    context->get(mapping);
    if ( createInMapping || mapping == NULL)
    {
        sofa::component::misc::StochasticPositionHandler::SPtr ws;
        context->get(ws, this->subsetsToManage, core::objectmodel::BaseContext::Local);
        if ( ws == NULL )
        {
            ws = sofa::core::objectmodel::New<StochasticPositionHandler>();
            gnode->addObject(ws);
            ws->f_writeX.setValue(recordX);

            for (core::objectmodel::TagSet::iterator it=this->subsetsToManage.begin(); it != this->subsetsToManage.end(); ++it)
                ws->addTag(*it);

        }
        std::ostringstream ofilename;
        ofilename << sceneName << "_" << counterStochasticPositionHandler << "_" << ms->getName()  << "_mstate" << extension ;

        ws->f_filename.setValue(ofilename.str()); ws->init(); ws->f_listening.setValue(true);  //Activated at init

        ++counterStochasticPositionHandler;

    }
}



//if state is true, we activate all the write states present in the scene.
simulation::Visitor::Result StochasticPositionHandlerActivator::processNodeTopDown( simulation::Node* gnode)
{
    sofa::component::misc::StochasticPositionHandler *ws = gnode->get< sofa::component::misc::StochasticPositionHandler >(this->subsetsToManage);
    if (ws != NULL) { changeStateWriter(ws);}
    return simulation::Visitor::RESULT_CONTINUE;
}

void StochasticPositionHandlerActivator::changeStateWriter(sofa::component::misc::StochasticPositionHandler*ws)
{
    if (!state) ws->reset();
    ws->f_listening.setValue(state);
}



} // namespace misc

} // namespace component

} // namespace sofa

