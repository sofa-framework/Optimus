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
#include <sofa/simulation/common/common.h>
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
    , m_monitorPrefix( initData(&m_monitorPrefix, std::string("monitor1"), "monitorPrefix", "prefix of the monitor-generated file") )
    , m_actualObservation( initData (&m_actualObservation, "actualObservation", "actual observation") )
{

}

template<class DataTypes>
SimulatedStateObservationSource<DataTypes>::~SimulatedStateObservationSource()
{
}


template<class DataTypes>
void SimulatedStateObservationSource<DataTypes>::init()
{
    sout << this->getName() << " Init started" << sendl;
    std::string posFile = m_monitorPrefix.getValue() + "_x.txt";

    parseMonitorFile(posFile);
    m_actualObservation.setValue(positions[0]);  // OK, since there is at least the dummy observation

    sout << "Init done" << sendl;
}

template<class DataTypes>
void SimulatedStateObservationSource<DataTypes>::parseMonitorFile(std::string& _name) {
    std::ifstream file(_name.c_str());

    nParticles = 0;
    nObservations = 0;
    if (file.good()) {
        /// parse the header of a monitor-generated file:
        std::string line;

        getline(file, line);
        if (line[0] != '#') {
            serr << "ERROR on line 2 in " << name << sendl;
            return;
        }

        getline(file, line);
        if (line[0] != '#') {
            serr << "ERROR on line 2 in " << name << sendl;
            return;
        }

        std::stringstream ss(line);
        std::istream_iterator<std::string> it(ss);
        std::istream_iterator<std::string> end;
        std::vector<std::string> tokens(it, end);

        int tki = 0;
        while (std::strcmp(tokens[tki++].c_str(),"number")!=0) ;

        nParticles = tokens.size() - tki;

        sout << "Number of observed particles: " << nParticles << sendl;

        getline(file, line);
        std::stringstream ss2(line);
        std::istream_iterator<std::string> it2(ss2);
        std::istream_iterator<std::string> end2;
        std::vector<std::string> tk2(it2, end2);

        tokens=tk2;
        while (tokens.size() > 1) {
            int dim = (tokens.size() -1)/nParticles;
            if (dim != 3) {
                serr << "ERROR on line " << 3+nObservations << " dim: " << dim << sendl;
                return;
            }

            if (nObservations == 0)
                dt = atof(tokens[0].c_str());

            if (nObservations == 1)
                dt = atof(tokens[0].c_str()) - dt;

            VecCoord position(nParticles);
            for (int i = 0; i < nParticles; i++)
                for (int d = 0; d < 3; d++)
                    position[i][d] = atof(tokens[3*i+d+1].c_str());

            positions.push_back(position);
            nObservations++;

            //sout << " positions size: " << positions.size() << sendl;

            getline(file, line);
            std::stringstream ss(line);
            std::istream_iterator<std::string> it(ss);
            std::istream_iterator<std::string> end;
            std::vector<std::string> tk(it, end);

            tokens=tk;
        }
    } else {
        serr << ": ERROR cannot open " << name << sendl;
        return;
    }

    if (nObservations > 0) {
        sout << "Valid observations available: #observations: " << nObservations << " #particles: " << nParticles << sendl;
    } else {
        // workaround in the case when no observations are available
        if (nParticles > 0) {
            VecCoord position(size_t(nParticles), Coord(0.0,0.0,0.0));
            positions.push_back(position);
            serr << "ERROR: no positions, adding zero observations of length " << nParticles << sendl;
        } else  {
            VecCoord position(size_t(1000), Coord(0.0,0.0,0.0));
            positions.push_back(position);
            serr << "OBSERVATION FATAL ERROR: no positions and no particles, adding dummy zero observation vector of length 1000!" << sendl;
        }
    }
}



} // namespace container

} // namespace component

} // namespace sofa

#endif
