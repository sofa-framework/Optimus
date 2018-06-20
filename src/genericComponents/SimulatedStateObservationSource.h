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
class SOFA_SIMULATION_COMMON_API SimulatedStateObservationSourceBase : public BaseObject //  ObservationSource
{
public:
    virtual int getObsDimention() = 0;
};

template<class DataTypes>
class SOFA_SIMULATION_COMMON_API SimulatedStateObservationSource : public SimulatedStateObservationSourceBase //  ObservationSource
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SimulatedStateObservationSource, DataTypes) ,BaseObject);

    typedef SimulatedStateObservationSourceBase Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename std::vector<VecCoord> VecVecCoord;
    typedef typename std::map<double, VecCoord> ObservationTable;

protected:
    ObservationTable observationTable;

    int nParticles, nObservations, dim;
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
    Data< bool  > d_asynObs;


    /// maps:  time + vector

    std::vector<VecCoord> positions;

    void init();

    void bwdInit() {}

    void draw(const core::visual::VisualParams* vparams);

    void parseMonitorFile(std::string& _name);
    void parseAsynMonitorFile(std::string& _name);
    /// an "alias" for better naming
    bool getStateAtTime(double _time, VecCoord& _state) {
        return(getObservation(_time, _state));
    }

#define ROUND 10000000.0

    bool getObservation(double _time, VecCoord& _observation) {
        if (d_asynObs.getValue()) {
            helper::ReadAccessor<Data<VecCoord> > tracObs = m_trackedObservations;
            if (tracObs.size() > 0 ) {
                m_actualObservation.setValue(m_trackedObservations.getValue());
                _observation = m_actualObservation.getValue();
            } else {
                int tround = (int) (_time * ROUND);
                _time = tround / ROUND; // round the value time
                typename std::map<double, VecCoord>::iterator it = observationTable.find(_time);
                if (it == observationTable.end()) {
                    PRNE("No observation for time " << _time << " Computing Only Prediction ");
                    m_actualObservation.setValue(VecCoord());
                    _observation = VecCoord();
                    return false;
                } else {
                    m_actualObservation.setValue(it->second);
                    _observation = it->second;
                    if (it->second.empty()){
                        PRNE("No observation for time " << _time << " Computing Only Prediction ");
                        return false;
                    }
                }
            }
            return(true);
        } else {
            helper::ReadAccessor<Data<VecCoord> > tracObs = m_trackedObservations;
            if (tracObs.size() > 0 ) {
                m_actualObservation.setValue(m_trackedObservations.getValue());
                _observation = m_actualObservation.getValue();
            } else {
                size_t ix = (fabs(dt) < 1e-10) ? 0 : size_t(round(_time/dt));
                //PRNS("Getting observation for time " << _time << " index: " << ix);
                if (ix >= size_t(positions.size())) {
                    PRNE("No observation for time " << _time << " , using the last one from " << positions.size()-1);
                    ix = positions.size() - 1;
                }
                m_actualObservation.setValue(positions[ix]);
                _observation = positions[ix];
            }

            return(true);
        }

    }

    int getNParticles() {
        return nParticles;
    }

    int getStateSize() {
        return nParticles;
    }

    int getObsDimention() {
        return dim;
    }

    int getNStates() {
        return nObservations;
    }

    void handleEvent(core::objectmodel::Event *event);

    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        //        if (dynamic_cast<MState *>(context->getMechanicalState()) == NULL) return false;
        return sofa::core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const SimulatedStateObservationSource<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }


};

} // namespace container

} // namespace component

} // namespace sofa

#endif
