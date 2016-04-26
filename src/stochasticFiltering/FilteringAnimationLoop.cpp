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

//#define FILTERINGANIMATIONLOOP_CPP

#include "FilteringAnimationLoop.h"

#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace simulation
{

SOFA_DECL_CLASS(FilteringAnimationLoop)

int FilteringAnimationLoopClass = core::RegisterObject("Animation loop for stochastic filtering, requires a filter")
        .add< FilteringAnimationLoop >()
        ;

FilteringAnimationLoop::FilteringAnimationLoop()
    : Inherit()
    , gnode(0)
    , verbose( initData(&verbose, false, "verbose", "print out traces") )
{
}


FilteringAnimationLoop::FilteringAnimationLoop(sofa::simulation::Node* _gnode)
    : Inherit()
    , gnode(_gnode)
    , verbose( initData(&verbose, false, "verbose", "print out traces") )
{
    assert(gnode);
}

void FilteringAnimationLoop::init() {
    Inherit::init();

    if (!gnode)
        gnode = dynamic_cast<sofa::simulation::Node*>(this->getContext());

    if (!gnode) {
        PRNE("Cannot find node!");
        return;
    }

    gnode->get(filter, core::objectmodel::BaseContext::SearchDown);

    if (filter) {
        PRNS("found stochastic filter: " << filter->getName());
    } else
        PRNE(" no stochastic filter found!");

    gnode->get<sofa::component::stochastic::PreStochasticWrapper>(&preStochasticWrappers, this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);

    for (size_t i = 0; i < preStochasticWrappers.size(); i++)
        PRNS("pre-stochastic filter " << preStochasticWrappers[i]->getName() << " found");

    actualStep = 0;
}

void FilteringAnimationLoop::bwdInit() {
    //PRNS("bwdInit");
}


void FilteringAnimationLoop::step(const core::ExecParams* _params, SReal /*_dt*/) {
    actualStep++;
    if (verbose.getValue()) {
        PRNS("======================= TIME STEP " << actualStep << "=======================");
    }
    if (!filter) {
        PRNE("No filter defined!");
        return;
    }

    for (size_t i = 0; i < preStochasticWrappers.size(); i++)
        preStochasticWrappers[i]->step(_params, actualStep);

    filter->initializeStep(_params, actualStep);
    TIC
    filter->computePrediction();
    TOCTIC("== prediction total");
    filter->computeCorrection();
    TOC("== correction total");

}


} // namespace simulation
} // namespace component
} // namespace sofa




