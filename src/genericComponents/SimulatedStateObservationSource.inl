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


#include "SimulatedStateObservationSource.h"

#include <sofa/helper/system/FileRepository.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Data.h>
#include <iterator>
#include <fstream>
#include <iostream>
#include <string>
#include <numeric>
#define ROUND 10000000.0



namespace sofa
{

namespace component
{

namespace container
{


using namespace sofa::core::objectmodel;


template<class DataTypes>
SimulatedStateObservationSource<DataTypes>::SimulatedStateObservationSource()
    : Inherit()
    , d_verbose( initData(&d_verbose, false, "verbose", "print tracing informations") )
    , d_monitorPrefix( initData(&d_monitorPrefix, std::string("monitor1"), "monitorPrefix", "prefix of the monitor-generated file") )
    , d_velocitiesData( initData(&d_velocitiesData, false, "velocitiesData", "load velocities data") )
    , d_actualObservation( initData (&d_actualObservation, "actualObservation", "actual observation") )
    , d_drawSize( initData(&d_drawSize, SReal(0.0),"drawSize","size of observation spheres in each time step") )
    , d_controllerMode( initData(&d_controllerMode, false,"controllerMode","if true, sets the mechanical object in begin animation step") )
    , d_trackedObservations( initData (&d_trackedObservations, "trackedObservations", "tracked observations: temporary solution!!!") )
    , d_asynObs(initData(&d_asynObs, false, "asynObs","Asynchronous or Missing Observations"))
{
    m_exportIndices = false;
}

template<class DataTypes>
SimulatedStateObservationSource<DataTypes>::~SimulatedStateObservationSource()
{
}


template<class DataTypes>
void SimulatedStateObservationSource<DataTypes>::init()
{
    if(d_asynObs.getValue())
        std::cout << "[SimulatedStateObservationSource] Asynchronous Observations Used "<<std::endl;

    helper::ReadAccessor<Data<VecCoord> > tracObs = d_trackedObservations;
    /// take observations from another component (tracking)
    if (tracObs.size() > 0) {
        m_nParticles = tracObs.size();
        std::cout << "[" << this->getName() << "]: taking observations from other source, size: " << tracObs.size() << std::endl;
        d_actualObservation.setValue(d_trackedObservations.getValue());
    } else {   /// read observations from a file
        //std::cout << this->getName() << " Init started" << std::endl;
        std::string dataFile;
        if (d_velocitiesData.getValue()) {
            dataFile = d_monitorPrefix.getValue() + "_v.txt";
        } else {
            dataFile = d_monitorPrefix.getValue() + "_x.txt";
        }

        if(sofa::helper::system::DataRepository.findFile(dataFile))
        {
            if (d_asynObs.getValue()) {
                parseAsynMonitorFile(dataFile);
            } else {
                parseMonitorFile(dataFile);
            }
        }
        else
        {
            msg_error(this) << "Could not find " << dataFile;
        }

        if (m_positions.empty())
            d_actualObservation.setValue(VecCoord());  // OK, since there is at least the dummy observation
        else
            d_actualObservation.setValue(m_positions[0]);  // OK, since there is at least the dummy observation
    }

    if (d_controllerMode.getValue())
        this->f_listening.setValue(true);

}


/*
 * File format:
 *  time | indices | positions
 *  e.g (in 2d)
 *  0.1 | 0 1 3 | 0.1 0.2 0.2 0.4 0.5 0.6
 */
template<class DataTypes>
void SimulatedStateObservationSource<DataTypes>::parseAsynMonitorFileVariable(const std::string &_name)
{
    m_observationTable.clear();
    m_indexTable.clear();
#ifdef __APPLE__
    setlocale(LC_ALL, "C");
#else
    std::setlocale(LC_ALL, "C");
#endif

    m_nParticles = 0;
    m_nObservations = 0;

    //info to fill
    helper::vector<Real> vdt;
    helper::vector<helper::vector<unsigned int> > vindices;
    helper::vector<helper::vector<Coord> > vpositions;
    unsigned int maxIndex = 0;

    std::ifstream in(_name);
    if (!in.good())
        return;

    std::string line;
    while (std::getline(in, line))
    {
        if (line.find("|") == std::string::npos)
            continue;

        Real dt;
        helper::vector<unsigned int> indices;
        helper::vector<Coord> positions;

        std::istringstream ss(line);
        std::string substr;
        std::getline(ss, substr, '|');
        dt = std::stod(substr);

        std::getline(ss, substr, '|');
        std::istringstream ssI(substr);
        unsigned int index;
        while (ssI >> index)
        {
            indices.push_back(index);
            maxIndex = std::max(index, maxIndex);
        }

        std::getline(ss, substr);
        std::istringstream ssP(substr);
        Coord position;
        while (ssP >> position)
            positions.push_back(position);

        vdt.push_back(dt);
        vindices.push_back(indices);
        vpositions.push_back(positions);

        m_nObservations++;
    }

    m_nParticles = maxIndex + 1; // size is biggest index we found + 1

    for(unsigned int i=0; i < m_nObservations; i++)
    {
        VecCoord position(m_nParticles);

        const helper::vector<unsigned int>& indices = vindices[i];
        for(unsigned int j=0 ; j<indices.size() ; j++)
        {
            position[j] = vpositions[i][j];
        }

        m_observationTable[vdt[i]] = position;
        m_indexTable[vdt[i]] = indices;

        m_positions.push_back(position);
    }
}

template<class DataTypes>
void SimulatedStateObservationSource<DataTypes>::parseAsynMonitorFile(const std::string& _name)
{
    m_observationTable.clear();
    m_indexTable.clear();
#ifdef __APPLE__
    setlocale(LC_ALL, "C");
#else
    std::setlocale(LC_ALL, "C");
#endif

    std::ifstream file(_name.c_str());

    m_nParticles = 0;
    m_nObservations = 0;
    m_initTime = -1.0;


    if (file.good()) {
        /// parse the header of a monitor-generated file:
        std::string line;

        m_nParticles = 0;

        do {
            getline(file, line);

            if (line.substr(0,14) == std::string("# nParticles :")) {
                std::istringstream stm(line.substr(15));
                stm >> m_nParticles;
                if(m_nParticles==0){
                    std::cerr << "[SimulatedStateObservationSource] ERROR nParticles is zero. Please check the header of Observations File " << std::endl;
                    return;
                }
                else
                std::cout << "[SimulatedStateObservationSource] nParticles observed: " << m_nParticles << std::endl;
            }
        } while (line[0] == '#');
        do {
            std::istringstream stm(line) ;
            std::vector<double> numbers ;

            double time;
            stm >> time; //skip dt

            double n ;
            while( stm >> n ) numbers.push_back(n) ;

            VecCoord position;
            for (unsigned i=0; i<numbers.size(); i+= Coord::size()) {
                Coord pos;
                for (unsigned j=0;j<Coord::size();j++) {
                    pos[j] = numbers[i+j];
                }
                position.push_back( pos );
            }

            m_positions.push_back(position);

            m_observationTable[time] = position;
            VecIndex index(position.size());
            std::iota(index.begin(), index.end(), 0); // fill vector with increment, from 0
            m_indexTable[time] = index;
            m_nObservations++;

        } while (getline(file, line));
    } else {
         serr <<"Cannot open " << name<<sendl;
        return;
    }

    PRNS("Number of observations: " << m_nObservations << " |tbl| = " << m_observationTable.size())

    if (m_nObservations > 0)
    {
        sout << "Valid observations available: #observations: " << m_nObservations << " #particles: " << m_nParticles << std::endl;
    }
#ifdef __APPLE__
    setlocale(LC_ALL, "C");
#else
    std::setlocale(LC_ALL, "C");
#endif


}

template<class DataTypes>
void SimulatedStateObservationSource<DataTypes>::parseMonitorFile(const std::string &_name)
{
    m_observationTable.clear();
    m_indexTable.clear();
#ifdef __APPLE__
    setlocale(LC_ALL, "C");
#else
    std::setlocale(LC_ALL, "C");
#endif


    std::ifstream file(_name.c_str());

    m_nParticles = 0;
    m_nObservations = 0;
    m_initTime = -1.0;

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
        std::size_t found = line.find("indices monitored: 1");
        if (found != std::string::npos) {
            m_exportIndices = true;
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

        m_nParticles = tokens.size() - tki;

        PRNS("Number of observed particles: " << m_nParticles);

        getline(file, line);
        nLine++;
        std::stringstream ss2(line);
        std::istream_iterator<std::string> it2(ss2);
        std::istream_iterator<std::string> end2;
        std::vector<std::string> tk2(it2, end2);

        tokens=tk2;
        m_dim = (tokens.size() -1)/m_nParticles;
        if (m_exportIndices) {
            if ((tokens.size() - 1) % 3 == 0)
                m_dim = 2;
            else if ((tokens.size() - 1) % 4 == 0)
                m_dim = 3;
            else
                PRNE(" Incorrect file format");
        }
        if (m_dim == 2) {
            PRNS(" Working with 2D observations" << " dim: " << m_dim);
        }
        while (tokens.size() > 1) {
//            if (dim != 3) {
//                PRNE(" On line " << nLine << " dim: " << dim);
//                return;
//            }

            double lineTime = atof(tokens[0].c_str()) ;
            if (m_initTime < 0.0)
                m_initTime = lineTime;
            m_finalTime = lineTime;

            if (m_nObservations == 0)
                m_dt = atof(tokens[0].c_str());

            if (m_nObservations == 1)
                m_dt = atof(tokens[0].c_str()) - m_dt;

            // when we export points with indices we know some of them are missing
            if (m_exportIndices) {
                m_nParticles = m_dim == 3 ? (tokens.size() - 1) / 4 : (tokens.size() - 1) / 3;
            }

            VecCoord position(m_nParticles);
            VecIndex index(position.size());
            VecIndex correspondentIndex(position.size());
            std::iota(index.begin(), index.end(), 0); // fill vector with increment, from 0

            if (m_dim == 2) {
                for (unsigned int i = 0; i < m_nParticles; i++) {
                    if (m_exportIndices) {
                        correspondentIndex[i] = atoi(tokens[3*i+1].c_str());
                        for (unsigned int d = 0; d < 2; d++)
                            position[i][d] = atof(tokens[3*i+d+2].c_str());
                    } else {
                        for (unsigned int d = 0; d < 2; d++)
                            position[i][d] = atof(tokens[2*i+d+1].c_str());
                    }
                }
            } else {
                for (unsigned int i = 0; i < m_nParticles; i++) {
                    if (m_exportIndices) {
                        correspondentIndex[i] = atoi(tokens[4*i+1].c_str());
                        for (unsigned int d = 0; d < 3; d++)
                            position[i][d] = atof(tokens[4*i+d+2].c_str());
                    } else {
                        for (unsigned int d = 0; d < 3; d++)
                            position[i][d] = atof(tokens[3*i+d+1].c_str());
                    }
                }
            }


            m_positions.push_back(position);
            m_correspondentIndices.push_back(correspondentIndex);
            // std::cout << "###### adding position to obsTable at " << lineTime << std::endl;
            m_observationTable[lineTime] = position;
            m_indexTable[lineTime] = index;
            m_correspondentTable[lineTime] = correspondentIndex;
            m_nObservations++;
            // std::cout << " positions size: " << positions.size() << std::endl;


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

    PRNS("Number of observations: " << m_nObservations << " |tbl| = " << m_observationTable.size())

    if (m_nObservations > 0) {
        sout << "Valid observations available: #observations: " << m_nObservations << " #particles: " << m_nParticles << std::endl;
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
#ifdef __APPLE__
    setlocale(LC_ALL, "C");
#else
    std::setlocale(LC_ALL, "C");
#endif


}

template<class DataTypes>
bool SimulatedStateObservationSource<DataTypes>::getObservation(double _time, VecCoord& _observation)
{
    VecIndex index;
    return getObservation(_time, _observation, index);
}

template<class DataTypes>
bool SimulatedStateObservationSource<DataTypes>::getObservation(double _time, VecCoord& _observation, VecIndex& _index)
{
    if (d_asynObs.getValue())
    {
        helper::ReadAccessor<Data<VecCoord> > tracObs = d_trackedObservations;
        if (tracObs.size() > 0 )
        {
            d_actualObservation.setValue(d_trackedObservations.getValue());
            _observation = d_actualObservation.getValue();

            // assume that actualObservation gives all data
            _index.resize(_observation.size());
            std::iota(_index.begin(), _index.end(), 0);
        }
        else
        {
            int tround = (int) (_time * ROUND);
            _time = tround / ROUND; // round the value time
            auto it = m_observationTable.find(_time);
            if (it == m_observationTable.end())
            {
                PRNE("No observation for time " << _time << " Computing Only Prediction ");
                d_actualObservation.setValue(VecCoord());
                _observation = VecCoord();
                _index = VecIndex();
                return false;
            }
            else
            {
                auto indexIt = m_indexTable.find(_time);
                d_actualObservation.setValue(it->second);
                _observation = it->second;
                _index = indexIt->second;
                if (it->second.empty())
                {
                    PRNE("No observation for time " << _time << " Computing Only Prediction ");
                    return false;
                }
            }
        }
        return true;
    }
    else
    {
        helper::ReadAccessor<Data<VecCoord> > tracObs = d_trackedObservations;
        if (tracObs.size() > 0 )
        {
            d_actualObservation.setValue(d_trackedObservations.getValue());
            _observation = d_actualObservation.getValue();
        }
        else
        {
            size_t ix = (fabs(m_dt) < 1e-10) ? 0 : size_t(round(_time/m_dt));
            //PRNS("Getting observation for time " << _time << " index: " << ix);
            if (ix >= size_t(m_positions.size()))
            {
                PRNE("No observation for time " << _time << " , using the last one from " << m_positions.size()-1);
                ix = m_positions.size() - 1;
            }

            d_actualObservation.setValue(m_positions[ix]);
            _observation = m_positions[ix];
        }

        return true;
    }
}

template<class DataTypes>
bool SimulatedStateObservationSource<DataTypes>::getCorrespondentIndices(double _time, VecIndex &_index)
{
    size_t ix = (fabs(m_dt) < 1e-10) ? 0 : size_t(round(_time/m_dt));
    if (m_correspondentIndices[ix].size() == 0)
        return false;
    _index = m_correspondentIndices[ix];
    return true;
}



template<class DataTypes>
void SimulatedStateObservationSource<DataTypes>::draw(const core::visual::VisualParams* vparams) {
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
void SimulatedStateObservationSource<DataTypes>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        helper::ReadAccessor<Data<VecCoord> > tracObs = d_trackedObservations;
        if (tracObs.size() > 0 ) {
        } else {
            double time = this->getTime();
            size_t ix = (fabs(m_dt) < 1e-10) ? 0 : size_t(round(time/m_dt));
            if (ix >= unsigned(m_positions.size()))
                ix = m_positions.size() - 1;

            core::behavior::MechanicalState<DataTypes> *mState = dynamic_cast<core::behavior::MechanicalState<DataTypes>*> (this->getContext()->getMechanicalState());

            helper::WriteAccessor<Data<VecCoord> > x = *mState->write(sofa::core::VecCoordId::position());

            // std::cout << " Sizes: mstate: " << x.size() << " positions: " << m_positions[ix].size() << std::endl;
            for (size_t i = 0; i < x.size(); i++)
                x[i] = m_positions[ix][i];
            d_actualObservation.setValue(m_positions[ix]);
        }
    }
}



} // namespace container

} // namespace component

} // namespace sofa

