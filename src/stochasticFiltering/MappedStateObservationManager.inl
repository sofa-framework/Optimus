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
#ifndef MAPPEDSTATEOBSERVATIONMANAGER_INL
#define MAPPEDSTATEOBSERVATIONMANAGER_INL

#include <sofa/simulation/common/Node.h>

#include "MappedStateObservationManager.h"


namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType, class DataTypes1, class DataTypes2>
MappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::MappedStateObservationManager()
    : Inherit()
    , inputObservationData( this->initData (&inputObservationData, "observations", "observations read from a file") )
    , mappedObservationData( this->initData (&mappedObservationData, "mappedObservations", "mapped observations") )
    , noiseStdev( this->initData(&noiseStdev, double(0.0), "noiseStdev", "standard deviation of generated noise") )
    , abberantIndex( this->initData(&abberantIndex, int(-1), "abberantIndex", "index of an aberrant point") )
    , doNotMapObservations( this->initData(&doNotMapObservations, false, "doNotMapObservations", "if real observations are read from a file (not the mechanical object)") )
{    
}

template <class FilterType, class DataTypes1, class DataTypes2>
void MappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::init()
{
    Inherit::init();
}

template <class FilterType, class DataTypes1, class DataTypes2>
void MappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::bwdInit()
{
    Inherit::bwdInit();
    //observationsNumber =
}

template <class FilterType, class DataTypes1, class DataTypes2>
typename MappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::EVectorX &
    MappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::getInnovation(EVectorX& _state)
{
    Inherit::bwdInit();
}



//template <class DataTypes>
//void MappedStateObservationManager<DataTypes>::init()
//{
//
//}
//
//template <class DataTypes>
//void MappedStateObservationManager<DataTypes>::reinit()
//{
//}

} // stochastic
} // component
} // sofa

#endif // MAPPEDSTATEOBSERVATIONMANAGER_INL
