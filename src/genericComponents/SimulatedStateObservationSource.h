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



namespace sofa
{

namespace component
{

namespace container
{


using namespace sofa::core::objectmodel;

/**
  Class implementing an observation source: it reads observations exported in direct simulation using OptimMonitor component and provides data to the observation manager.
*/
class SOFA_OPTIMUSPLUGIN_API SimulatedStateObservationSourceBase : public BaseObject //  ObservationSource
{
public:
    virtual unsigned int getObsDimention() = 0;
};

template<class DataTypes>
class SimulatedStateObservationSource : public SimulatedStateObservationSourceBase //  ObservationSource
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SimulatedStateObservationSource, DataTypes) , SimulatedStateObservationSourceBase);

    typedef SimulatedStateObservationSourceBase Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename helper::vector<unsigned int> VecIndex;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename std::vector<VecCoord> VecVecCoord;
    typedef typename std::map<double, VecCoord> ObservationTable;
    typedef typename std::map<double, VecIndex > PresenceTable;
    typedef typename std::map<double, VecIndex > CorrespondenceTable;

protected:
    ObservationTable m_observationTable;
    PresenceTable m_indexTable;
    CorrespondenceTable m_correspondentTable;

    unsigned int m_nParticles, m_nObservations, m_dim;
    double m_initTime, m_finalTime;
    double m_dt;

    bool m_exportIndices;

public:

    SimulatedStateObservationSource();
    virtual ~SimulatedStateObservationSource() override;

    Data<bool> d_verbose;
    Data<std::string> d_monitorPrefix;
    Data<bool> d_velocitiesData;
    Data<VecCoord> d_actualObservation;
    Data<VecIndex> d_correspondentIndices;
    Data<SReal> d_drawSize;
    Data<bool> d_controllerMode;
    Data<VecCoord> d_trackedObservations;
    Data< bool  > d_asynObs;


    /// maps:  time + vector

    std::vector<VecCoord> m_positions;
    std::vector<VecIndex> m_correspondentIndices;

    void init() override ;
    void bwdInit() override {}

    void draw(const core::visual::VisualParams* vparams) override;

    void parseMonitorFile(const std::string& _name);
    void parseAsynMonitorFile(const std::string &_name);
    void parseAsynMonitorFileVariable(const std::string& _name);

    /// an "alias" for better naming
    bool getStateAtTime(double _time, VecCoord& _state) {
        return(getObservation(_time, _state));
    }

    bool getObservation(double _time, VecCoord& _observation);
    bool getObservation(double _time, VecCoord& _observation, VecIndex &_index);
    bool getCorrespondentIndices(double _time, VecIndex &_index);

    unsigned int getNParticles() const {
        return m_nParticles;
    }

    unsigned int getStateSize() const {
        return m_nParticles;
    }

    unsigned int getObsDimention() override {
        return m_dim;
    }

    unsigned int getNStates() const {
        return m_nObservations;
    }

    void handleEvent(core::objectmodel::Event *event) override;

    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        //        if (dynamic_cast<MState *>(context->getMechanicalState()) == NULL) return false;
        return sofa::core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const SimulatedStateObservationSource<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

private:

};


extern template class SOFA_OPTIMUSPLUGIN_API SimulatedStateObservationSource<sofa::defaulttype::Vec2Types>;
extern template class SOFA_OPTIMUSPLUGIN_API SimulatedStateObservationSource<sofa::defaulttype::Vec3Types>;
extern template class SOFA_OPTIMUSPLUGIN_API SimulatedStateObservationSource<sofa::defaulttype::Rigid3Types>;



} // namespace container

} // namespace component

} // namespace sofa

