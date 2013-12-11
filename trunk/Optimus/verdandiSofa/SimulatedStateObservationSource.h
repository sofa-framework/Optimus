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

#include <sofa/simulation/common/common.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Data.h>
#include <string>

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
    typedef sofa::core::objectmodel::BaseObject Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename std::vector<VecCoord> VecVecCoord;

protected:
    int nParticles, nStates;
    double dt;

public:

    SimulatedStateObservationSource();
    ~SimulatedStateObservationSource();

    Data<std::string> m_monitorPrefix;

    /// maps:  time + vector
    //std::map<double, VecCoord> positions;
    std::vector<VecCoord> positions;



    void init();

    int parseMonitorFile(std::string& _name);
    VecCoord& getObservation(double time) {
        int ix = int(time/dt);

        if (ix >= int(positions.size())) {
            std::cerr << this->getName() << " ERROR: no observation for time " << time << " , using the last one from " << positions.size()-1 << std::endl;
            ix = positions.size() - 1;
        }

        return(positions[ix]);
    }

    int getNParticles() {
        return nParticles;
    }

    int getNStates() {
        return nStates;
    }


};

} // namespace container

} // namespace component

} // namespace sofa

#endif
