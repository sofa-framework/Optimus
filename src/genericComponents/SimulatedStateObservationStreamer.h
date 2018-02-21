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
#ifndef SOFA_CONTAINER_SIMULATED_STATE_OBSERVATIONSTREAMER_H
#define SOFA_CONTAINER_SIMULATED_STATE_OBSERVATIONSTREAMER_H


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
#include "SimulatedStateObservationSource.h"

using namespace sofa::core::objectmodel;

namespace sofa
{

namespace component
{

namespace container
{


template<class DataTypes>
class SOFA_SIMULATION_COMMON_API SimulatedStateObservationStreamer : public SimulatedStateObservationSourceBase //  ObservationSource
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SimulatedStateObservationStreamer, DataTypes) ,BaseObject);

    typedef SimulatedStateObservationSourceBase Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename std::vector<VecCoord> VecVecCoord;
    //typedef typename std::map<double, VecCoord> ObservationTable;

protected:
    //ObservationTable observationTable;

    int nParticles, nObservations, dim;

public:

    SimulatedStateObservationStreamer();
    ~SimulatedStateObservationStreamer();

    Data<bool> verbose;
    Data<VecCoord> m_trackedObservations;
    VecCoord m_actualObservation;
    Data<double> m_actualTime;
    Data<SReal> m_drawSize;
    Data<bool> m_controllerMode;

    VecCoord m_prevObservation;
    Data<double> m_prevTime;

    /// maps:  time + vector
    //std::map<double, VecCoord> positions;
    std::vector<VecCoord> positions;    

    void init();

    void bwdInit() {}

    void draw(const core::visual::VisualParams* vparams);

    /// an "alias" for better naming
    bool getStateAtTime(double _time, VecCoord& _state) {
        return(getObservation(_time, _state));
    }

    bool getObservation(double _time, VecCoord& _observation) {

        if (fabs(m_actualTime.getValue() - _time) < 1e-10) {
            _observation = m_actualObservation;
        } else {
            PRNE("No observation for time " << _time << " , using the last one from " << m_prevTime.getValue());
            _observation = m_prevObservation;
        }
        return(true);
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

    static std::string templateName(const SimulatedStateObservationStreamer<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }


};

} // namespace container

} // namespace component

} // namespace sofa

#endif  //SOFA_CONTAINER_SIMULATED_STATE_OBSERVATIONSTREAMER_H
