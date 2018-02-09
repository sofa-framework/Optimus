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
#include <sofa/helper/AdvancedTimer.h>

// quick fix
# include <unistd.h>
# include <sys/time.h>
# include <iomanip>

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
    , d_timeDataFile( initData(&d_timeDataFile, std::string(""), "computationTimeFile", "file to save computation results") )
{
}


FilteringAnimationLoop::FilteringAnimationLoop(sofa::simulation::Node* _gnode)
    : Inherit()
    , gnode(_gnode)
    , verbose( initData(&verbose, false, "verbose", "print out traces") )
    , d_timeDataFile( initData(&d_timeDataFile, std::string(""), "computationTimeFile", "file to save computation results") )
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

    if (d_timeDataFile.getValue().size() > 0) {
        std::ofstream outputFile;
        outputFile.open(d_timeDataFile.getFullPath(), std::fstream::app);
        outputFile << "This file contains data about computation time:";
        outputFile << std::endl;
        outputFile.close();
    }
}

void FilteringAnimationLoop::bwdInit() {
    //PRNS("bwdInit");
}


void FilteringAnimationLoop::step(const core::ExecParams* _params, SReal /*_dt*/) {    
    SReal dt = gnode->getDt();
    sofa::simulation::AnimateBeginEvent ev(dt);
    SingleLevelEventVisitor act ( _params, &ev, gnode );
    gnode->execute ( act );

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

    // add advanced timer to the system
    //sofa::helper::AdvancedTimer::stepBegin("KalmanFilterStep");
    struct timeval tv;

    if (d_timeDataFile.getValue().size() > 0) {
        gettimeofday(&tv,0);
        std::ofstream outputFile;
        outputFile.open(d_timeDataFile.getFullPath(), std::fstream::app);
        outputFile << "Iteration step: " << actualStep << std::endl;
        outputFile << "Start time: " << tv.tv_sec;
        outputFile.fill('0');
        outputFile << std::setw(6) << tv.tv_usec << std::endl;
        outputFile.fill(' ');
        outputFile.close();
    }

    filter->initializeStep(_params, actualStep);
    //TIC
    filter->computePrediction();
    //TOCTIC("== prediction total");
    filter->computeCorrection();
    //TOC("== correction total");

    // compute signle iteration step
    //sofa::helper::AdvancedTimer::stepEnd("KalmanFilterStep");

    if (d_timeDataFile.getValue().size() > 0) {
        gettimeofday(&tv,0);
        std::ofstream outputFile;
        outputFile.open(d_timeDataFile.getFullPath(), std::fstream::app);
        outputFile << "End time: " << tv.tv_sec;
        outputFile.fill('0');
        outputFile << std::setw(6) << tv.tv_usec << std::endl;
        outputFile.fill(' ');
        outputFile.close();
    }

    sofa::simulation::AnimateEndEvent ev2(dt);
    SingleLevelEventVisitor act2 ( _params, &ev2, gnode );
    gnode->execute ( act2 );

}

SingleLevelEventVisitor::SingleLevelEventVisitor(const core::ExecParams* params, sofa::core::objectmodel::Event* e, sofa::simulation::Node* node)
    : sofa::simulation::Visitor(params)
    , m_event(e)
    , m_node(node)
{}


SingleLevelEventVisitor::~SingleLevelEventVisitor()
{}

sofa::simulation::Visitor::Result SingleLevelEventVisitor::processNodeTopDown(sofa::simulation::Node* node)
{
    if (node == m_node) {
        for_each(this, node, node->object, &SingleLevelEventVisitor::processObject);

        if( m_event->isHandled() )
            return Visitor::RESULT_PRUNE;
        else
            return Visitor::RESULT_CONTINUE;
    } else
        return Visitor::RESULT_PRUNE;
}

void SingleLevelEventVisitor::processObject(sofa::simulation::Node*, core::objectmodel::BaseObject* obj)
{
    if( obj->f_listening.getValue()==true )
        obj->handleEvent( m_event );
}

} // namespace simulation
} // namespace component
} // namespace sofa




