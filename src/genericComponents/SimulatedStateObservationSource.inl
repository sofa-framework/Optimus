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
#ifndef SOFA_CONTAINER_SIMULATED_STATE_OBSERVATIONSOURCE_INL
#define SOFA_CONTAINER_SIMULATED_STATE_OBSERVATIONSOURCE_INL

#include "SimulatedStateObservationSource.h"

#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Data.h>

#include <string>
#include <iterator>
#include <sstream>

using namespace sofa::core::objectmodel;

namespace sofa
{

namespace component
{

namespace container
{


template<class DataTypes>
SimulatedStateObservationSource<DataTypes>::SimulatedStateObservationSource()
    : Inherit()
    , verbose( initData(&verbose, false, "verbose", "print tracing informations") )
    , m_monitorPrefix( initData(&m_monitorPrefix, std::string("monitor1"), "monitorPrefix", "prefix of the monitor-generated file") )
    , m_actualObservation( initData (&m_actualObservation, "actualObservation", "actual observation") )    
    , m_drawSize( initData(&m_drawSize, SReal(0.0),"drawSize","size of observation spheres in each time step") )
    , m_controllerMode( initData(&m_controllerMode, false,"controllerMode","if true, sets the mechanical object in begin animation step") )
    , m_trackedObservations( initData (&m_trackedObservations, "trackedObservations", "tracked observations: temporary solution!!!") )
{

}

template<class DataTypes>
SimulatedStateObservationSource<DataTypes>::~SimulatedStateObservationSource()
{
}


template<class DataTypes>
void SimulatedStateObservationSource<DataTypes>::init()
{    
    helper::ReadAccessor<Data<VecCoord> > tracObs = m_trackedObservations;

    /// take observations from another component (tracking)
    if (tracObs.size() > 0) {
        nParticles = tracObs.size();
        std::cout << "[" << this->getName() << "]: taking observations from other source, size: " << tracObs.size() << std::endl;
        m_actualObservation.setValue(m_trackedObservations.getValue());
    } else {   /// read observations from a file
        //std::cout << this->getName() << " Init started" << std::endl;
        std::string posFile = m_monitorPrefix.getValue() + "_x.txt";

        parseMonitorFile(posFile);
        m_actualObservation.setValue(positions[0]);  // OK, since there is at least the dummy observation
    }

    if (m_controllerMode.getValue())
        this->f_listening.setValue(true);

    //std::cout << "Init done" << std::endl;
}

template<class DataTypes>
void SimulatedStateObservationSource<DataTypes>::parseMonitorFile(std::string& _name) {
    observationTable.clear();
    std::setlocale(LC_ALL, "C");


    std::ifstream file(_name.c_str());

    nParticles = 0;
    nObservations = 0;
    initTime = -1.0;

    if (file.good()) {
        /// parse the header of a monitor-generated file:
        std::string line;
        size_t nLine = 0;

        getline(file, line);
        nLine++;
        //std::cout << "Here: " << line << std::endl;
        if (line[0] != '#') {
            PRNE(" On line " << nLine << " in " << name);
            return;
        }

        getline(file, line);
        nLine++;
        //std::cout << "Here: " << line << std::endl;
        if (line[0] != '#') {
            PRNE(" On line " << nLine << " in " << name);
            return;
        }

        std::stringstream ss(line);         
        std::istream_iterator<std::string> it(ss);         
        std::istream_iterator<std::string> end;         
        std::vector<std::string> tokens(it, end);        

        int tki = 0;        
        while (std::strcmp(tokens[tki++].c_str(),"number")!=0) ;

        nParticles = tokens.size() - tki;

        PRNS("Number of observed particles: " << nParticles);

        getline(file, line);
        nLine++;
        std::stringstream ss2(line);
        std::istream_iterator<std::string> it2(ss2);
        std::istream_iterator<std::string> end2;
        std::vector<std::string> tk2(it2, end2);

        tokens=tk2;
        int dim = (tokens.size() -1)/nParticles;
        if (dim == 2) {
            PRNS(" Working with 2D observations" << " dim: " << dim);
        }
        while (tokens.size() > 1) {
            if (dim != 3) {
//                PRNE(" On line " << nLine << " dim: " << dim);
//                return;
            }

            double lineTime = atof(tokens[0].c_str()) ;
            if (initTime < 0.0)
                initTime = lineTime;
            finalTime = lineTime;

            if (nObservations == 0)
                dt = atof(tokens[0].c_str());

            if (nObservations == 1)
                dt = atof(tokens[0].c_str()) - dt;

            VecCoord position(nParticles);
            if (dim == 2) {
                for (int i = 0; i < nParticles; i++)
                    for (int d = 0; d < 2; d++)
                        position[i][d] = atof(tokens[2*i+d+1].c_str()); ;
            } else {
                for (int i = 0; i < nParticles; i++)
                    for (int d = 0; d < 3; d++)
                        position[i][d] = atof(tokens[3*i+d+1].c_str());
            }


            positions.push_back(position);
            //std::cout << "###### adding position to obsTable at " << lineTime << std::endl;
            observationTable[lineTime] = position;
            nObservations++;

            //std::cout << " positions size: " << positions.size() << std::endl;

            getline(file, line);
            nLine++;
            std::stringstream ss(line);
            std::istream_iterator<std::string> it(ss);
            std::istream_iterator<std::string> end;
            std::vector<std::string> tk(it, end);

            tokens=tk;
        }
    } else {
        PRNE("Cannot open " << name);
        return;
    }

    std::cout << "Nunmber of observations: " << nObservations << " |tbl| = " << observationTable.size() << std::endl;

    if (nObservations > 0) {
        sout << "Valid observations available: #observations: " << nObservations << " #particles: " << nParticles << std::endl;
    } /*else {
        // workaround in the case when no observations are available
        if (nParticles > 0) {
            VecCoord position(size_t(nParticles), Coord(0.0,0.0,0.0));
            positions.push_back(position);
            std::cout << "ERROR: no positions, adding zero observations of length " << nParticles << std::endl;
        } else  {
            VecCoord position(size_t(1000), Coord(0.0,0.0,0.0));
            positions.push_back(position);
            std::cout << "OBSERVATION FATAL ERROR: no positions and no particles, adding dummy zero observation vector of length 1000!" << std::endl;
        }
    }*/
    std::setlocale(LC_ALL, NULL);

}


template<class DataTypes>
void SimulatedStateObservationSource<DataTypes>::draw(const core::visual::VisualParams* vparams) {
//    if (!vparams->displayFlags().getShowBehaviorModels())
//        return;

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
void SimulatedStateObservationSource<DataTypes>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        helper::ReadAccessor<Data<VecCoord> > tracObs = m_trackedObservations;
        if (tracObs.size() > 0 ) {
        } else {
            double time = this->getTime();
            size_t ix = (fabs(dt) < 1e-10) ? 0 : size_t(round(time/dt));
            if (ix >= int(positions.size()))
                ix = positions.size() - 1;

            core::behavior::MechanicalState<DataTypes> *mState = dynamic_cast<core::behavior::MechanicalState<DataTypes>*> (this->getContext()->getMechanicalState());

            helper::WriteAccessor<Data<VecCoord> > x = *mState->write(sofa::core::VecCoordId::position());

            //std::cout << "Sizes: mstate: " << x.size() << " positions: " << positions[ix].size() << std::endl;
            for (size_t i = 0; i < x.size(); i++)
                x[i] = positions[ix][i];
        }
    }
}


} // namespace container

} // namespace component

} // namespace sofa

#endif
