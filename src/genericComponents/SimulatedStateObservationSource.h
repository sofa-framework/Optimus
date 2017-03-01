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
#ifndef SOFA_CONTAINER_SIMULATED_STATE_OBSERVATIONSOURCE_H
#define SOFA_CONTAINER_SIMULATED_STATE_OBSERVATIONSOURCE_H


#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Data.h>
#include <SofaBaseMechanics/MechanicalObject.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/gl/BasicShapes.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <string>

#include "../initOptimusPlugin.h"
#include "ObservationSource.h"

using namespace sofa::core::objectmodel;

namespace sofa
{

namespace component
{

namespace container
{

/**
  Event fired when needed to stop the animation.
*/

template<class DataTypes>
class SOFA_SIMULATION_COMMON_API SimulatedStateObservationSource : public BaseObject //  ObservationSource
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SimulatedStateObservationSource, DataTypes) ,BaseObject);

    typedef sofa::core::objectmodel::BaseObject Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename std::vector<VecCoord> VecVecCoord;
    typedef typename std::map<double, VecCoord> ObservationTable;

protected:
    ObservationTable observationTable;

    int nParticles, nObservations;
    double initTime, finalTime;
    double dt;

public:

    SimulatedStateObservationSource();
    ~SimulatedStateObservationSource();

    Data<bool> verbose;
    Data<std::string> m_monitorPrefix;
    Data<VecCoord> m_actualObservation;
    Data<SReal> m_drawSize;
    Data<bool> m_controllerMode;

    Data<VecCoord> m_trackedObservations;

    /// maps:  time + vector
    //std::map<double, VecCoord> positions;
    std::vector<VecCoord> positions;    

    void init();

    void bwdInit() {}

    void draw(const core::visual::VisualParams* vparams);

    void parseMonitorFile(std::string& _name);

    /// an "alias" for better naming
    bool getStateAtTime(double _time, VecCoord& _state) {
        return(getObservation(_time, _state));
    }

    bool getObservation(double _time, VecCoord& _observation) {

        helper::ReadAccessor<Data<VecCoord> > tracObs = m_trackedObservations;
        if (tracObs.size() > 0 ) {
            m_actualObservation.setValue(m_trackedObservations.getValue());
            _observation = m_actualObservation.getValue();
        } else {
            size_t ix = (fabs(dt) < 1e-10) ? 0 : size_t(round(_time/dt));
            //PRNS("Getting observation for time " << _time << " index: " << ix);
            if (ix >= int(positions.size())) {
                PRNE("No observation for time " << _time << " , using the last one from " << positions.size()-1);
                ix = positions.size() - 1;
            }
            m_actualObservation.setValue(positions[ix]);
            _observation = positions[ix];
        }

        return(true);

        /// TODO: switch to the new way of storing and gettting the observations!!!
        /*double firstTime = observationTable.begin()->first;
        if (_time <= firstTime) {
            PRNW("Requested observation time " << _time << ", given observation for time " << firstTime);
            _observation = observationTable.begin()->second;
            return(true);
        }

        double lastTime = observationTable.rbegin()->first;
        if (_time >= lastTime) {
            PRNW("Requested observation time " << _time << ", given observation for time " << lastTime);
            _observation = observationTable.rbegin()->second;
            return(true);
        }

        typename ObservationTable::iterator it = observationTable.find(_time);
        if (it != observationTable.end()) {
            PRNS("Giving observation in time " << _time);
            _observation = it->second;
            return(true);
        }

        PRNE("No observation found for time " << _time);
        return(false);*/

    }

    int getNParticles() {
        return nParticles;
    }

    int getStateSize() {
        return nParticles;
    }

    int getNStates() {
        return nObservations;
    }

    void handleEvent(core::objectmodel::Event *event);


};

} // namespace container

} // namespace component

} // namespace sofa

#endif
