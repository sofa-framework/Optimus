/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_MISC_VERDANDI_ACTIVE_STATE_MONITOR_INL
#define SOFA_COMPONENT_MISC_VERDANDI_ACTIVE_STATE_MONITOR_INL

#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/Simulation.h>
#include <sofa/simulation/common/MechanicalComputeEnergyVisitor.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/core/objectmodel/Context.h>
#include <sofa/core/objectmodel/Data.h>
#include <fstream>
#include <sofa/defaulttype/Vec.h>
#include <cmath>
#include <limits>

#include "VerdandiActiveStateMonitor.h"

namespace sofa
{

namespace component
{

namespace misc
{

//using namespace sofa::defaulttype;
//using namespace std;

template <class DataTypes>
VerdandiActiveStateMonitor<DataTypes>::VerdandiActiveStateMonitor()
    : Inherit()    
    , verdandiLoopLink(initLink("verdandiLoop", "link to the verdandi animation loop"))
    //, m_objectID( initData (&m_objectID, "objectID", "unknown", "name for the object being monitored") )
{
}

/*template <class DataTypes>
VerdandiActiveStateMonitor<DataTypes>::~VerdandiActiveStateMonitor()
{
    Inherit::~Inherit();
}*/


template<class DataTypes>
void VerdandiActiveStateMonitor<DataTypes>::init()
{
    Inherit::init();
    TVerdandiAnimationLoop* vloop = verdandiLoopLink.get();

    if ( vloop != NULL) {
        std::cout << this->getName() << ": associated with loop " << vloop->getName() << std::endl;
        //std::cout << this->getName() << ": size of object positions: " << contr->objectPositions.size() << std::endl;
    } else {
        std::cerr << this->getName() << ": no verdandi loop associated!" << std::endl;
        return;
    }
}



}

}

}


#endif
