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
#pragma once

#include "SimulatedStateObservationStreamer.h"

#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Data.h>

#include <string>
#include <iterator>
#include <sstream>



namespace sofa
{

namespace component
{

namespace container
{


using namespace sofa::core::objectmodel;

template<class DataTypes>
SimulatedStateObservationStreamer<DataTypes>::SimulatedStateObservationStreamer()
    : SimulatedStateObservationSource<DataTypes>()
    , m_streamObservations( initData (&m_streamObservations, "streamObservation", "observations obtained from another source") )
    , m_actualTime( initData (&m_actualTime, "actualTime", "actual iteration time") )
    , m_prevTime( initData (&m_prevTime, "previousTime", "previous iteration time") )
{

}

template<class DataTypes>
SimulatedStateObservationStreamer<DataTypes>::~SimulatedStateObservationStreamer()
{
}


template<class DataTypes>
void SimulatedStateObservationStreamer<DataTypes>::init()
{    
    helper::ReadAccessor<Data<VecCoord> > tracObs = m_streamObservations;

    /// take observations from another component (tracking)
    if (tracObs.size() > 0) {
        nParticles = tracObs.size();
        std::cout << "[" << this->getName() << "]: taking observations from other source, size: " << tracObs.size() << std::endl;
        m_actualObservation = (m_streamObservations.getValue());
    }

    this->f_listening.setValue(true);
    //std::cout << "Init done" << std::endl;
}



template<class DataTypes>
void SimulatedStateObservationStreamer<DataTypes>::draw(const core::visual::VisualParams* vparams) {
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

//    helper::ReadAccessor<Data<VecCoord> > tracObs = m_trackedObservations;

//    std::vector<sofa::defaulttype::Vec3d> points;
//    if (tracObs.size() > 0 ) {

//    } else {
//        double time = this->getTime();
//        size_t ix = (fabs(dt) < 1e-10) ? 0 : size_t(round(time/dt));
//        if (ix >= int(positions.size()))
//            ix = positions.size() - 1;

//        points.resize(positions[ix].size());

//        for (size_t i = 0;  i < points.size(); i++)
//            points[i] = positions[ix][i];
//    }

//    vparams->drawTool()->drawSpheres(points, float(m_drawSize.getValue()), sofa::defaulttype::Vec<4, float> (0.0f, 0.0f, 1.0f, 1.0f));
}

template<class DataTypes>
void SimulatedStateObservationStreamer<DataTypes>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        if (m_prevObservation.size() < m_actualObservation.size()) {
            m_prevObservation.resize(m_actualObservation.size());
        }
        for (size_t index = 0; index < m_actualObservation.size(); index++)
            m_prevObservation[index] = m_actualObservation[index];

        helper::ReadAccessor<Data<VecCoord> > tracObs = m_streamObservations;
        if (tracObs.size() > 0) {
            nParticles = tracObs.size();
            std::cout << "[" << this->getName() << "]: taking observations from other source, size: " << tracObs.size() << std::endl;

            core::behavior::MechanicalState<DataTypes> *mState = dynamic_cast<core::behavior::MechanicalState<DataTypes>*> (this->getContext()->getMechanicalState());
            helper::WriteAccessor<Data<VecCoord> > pos = *mState->write(sofa::core::VecCoordId::position());

            //std::cout << "Sizes: mstate: " << x.size() << " positions: " << positions[ix].size() << std::endl;
            for (size_t index = 0; index < pos.size(); index++) {
                pos[index] = tracObs[index];
                m_actualObservation[index] = tracObs[index];
            }
        }
    }
}



} // namespace container

} // namespace component

} // namespace sofa

