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
#pragma once

#include <sofa/simulation/Node.h>
#include "SimpleObservationManager.h"



namespace sofa
{

namespace component
{

namespace stochastic
{



template <class FilterType, class DataTypes1, class DataTypes2>
SimpleObservationManager<FilterType, DataTypes1, DataTypes2>::SimpleObservationManager()
    : Inherit()
    , d_projectionMatrix( initData(&d_projectionMatrix, sofa::type::Mat3x4d(type::Vec<4,float>(1.0,0.0,0.0,0.0),
                                                                            type::Vec<4,float>(0.0,1.0,0.0,0.0),
                                                                            type::Vec<4,float>(0.0,0.0,1.0,0.0)), "projectionMatrix", "Projection matrix"))
    , stateWrapperLink(initLink("stateWrapper", "link to the state wrapper needed to perform apply (perhaps to be changed)"))
{ }



template <class FilterType, class DataTypes1, class DataTypes2>
void SimpleObservationManager<FilterType, DataTypes1, DataTypes2>::init()
{
    Inherit::init();

    this->gnode->get(observationSource);
    if (observationSource) {
        PRNS("Found observation source: " << observationSource->getName());
    } else {
        PRNE("No observation source found!");
    }

    stateWrapper = stateWrapperLink.get();
    if (stateWrapper) {
        std::cout << "[SimpleObservationManager] Link to state wrapper: " << stateWrapper->getName() << std::endl;
    } else {
        std::cout << "[SimpleObservationManager] Link to state wrapper not initialized!" << std::endl;
        return;
    }

    this->gnode->get(masterState);
    if (masterState != NULL) {
        PRNS("Found master mechanical state: " << masterState->getName());
    } else {
        PRNE("No master mechanical state found!");
    }
}



template <class FilterType, class DataTypes1, class DataTypes2>
void SimpleObservationManager<FilterType, DataTypes1, DataTypes2>::bwdInit()
{
    this->observationSize = observationSource->getStateSize() * DataTypes1::spatial_dimensions;
    Inherit::bwdInit();
}



template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleObservationManager<FilterType, DataTypes1, DataTypes2>::hasObservation(double _time)
{
    bool hasObservation;
    if (this->actualTime == 0) {
        hasObservation = true;
    } else {
        hasObservation = observationSource->getObservation(this->actualTime, realObservations);
    }

    if (!hasObservation) {
        PRNE("No observation for time " << _time);
        return (false);
    }

    return (true);
}



template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleObservationManager<FilterType, DataTypes1, DataTypes2>::getInnovation(double _time, EVectorX& _state, EVectorX& _innovation)
{
    if (_time != this->actualTime) {
        PRNE("Observation for time " << this->actualTime << " not prepared, call hasObservation first!");
        return (false);
    }

    if (_innovation.rows() != long(this->observationSize)) {
        PRNE("Wrong innovation size: " << _innovation.rows() << " should be " << this->observationSize);
        return (false);
    }

    if ((stateWrapper->getFilterKind() == SIMCORR) || (stateWrapper->getFilterKind() == CLASSIC) || (stateWrapper->getFilterKind() == ENSEMBLTRANSF)) {
            for (size_t i = 0; i < this->observationSize; i++)
                _innovation(i) = realObservations[0](i) - _state(i);
    }

    return (true);
}



template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleObservationManager<FilterType, DataTypes1, DataTypes2>::getRealObservation(double _time, EVectorX& _realObs)
{
    if (_time != this->actualTime) {
        PRNE("Observation for time " << this->actualTime << " not prepared, call hasObservation first!");
        return (false);
    }

    if (_realObs.rows() != long(this->observationSize)) {
        PRNE("Wrong innovation size: " << _realObs.rows() << " should be " << this->observationSize);
        return (false);
    }

    if ((stateWrapper->getFilterKind() == SIMCORR) || (stateWrapper->getFilterKind() == CLASSIC) || (stateWrapper->getFilterKind() == ENSEMBLTRANSF)) {
        for (size_t i = 0; i < this->observationSize; i++) {
            _realObs(i) = realObservations[0](i);
        }
    }

    return true;
}



template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleObservationManager<FilterType, DataTypes1, DataTypes2>::obsFunction(EVectorX& _state, EVectorX& _predictedObservation)
{
    const sofa::type::Mat3x4d& P = d_projectionMatrix.getValue();
    _predictedObservation.resize(this->observationSize);

    Data< typename DataTypes1::VecCoord > predicted2DState;
    Data< typename DataTypes2::VecCoord > predicted3DState;

    typename DataTypes1::VecCoord& predicted2DStateEdit = *predicted2DState.beginEdit();
    typename DataTypes2::VecCoord& predicted3DStateEdit = *predicted3DState.beginEdit();

    predicted3DStateEdit.resize(masterState->getSize());

    stateWrapper->getPos(_state, predicted3DStateEdit);
    predicted2DStateEdit.resize(predicted3DStateEdit.size());

    //2D observations
    if (ObservationSource::Coord::total_size == 2)
    {
        for (unsigned i = 0; i < predicted3DStateEdit.size(); i++)
        {
            double rx = P[0][0] * predicted3DStateEdit[i][0] + P[0][1] * predicted3DStateEdit[i][1] + P[0][2] * predicted3DStateEdit[i][2] + P[0][3];
            double ry = P[1][0] * predicted3DStateEdit[i][0] + P[1][1] * predicted3DStateEdit[i][1] + P[1][2] * predicted3DStateEdit[i][2] + P[1][3];
            double rz = P[2][0] * predicted3DStateEdit[i][0] + P[2][1] * predicted3DStateEdit[i][1] + P[2][2] * predicted3DStateEdit[i][2] + P[2][3];
            predicted2DStateEdit[i][0] = rx * (1.0 / rz);
            predicted2DStateEdit[i][1] = ry * (1.0 / rz);
        }

        for (size_t i = 0; i < predicted3DStateEdit.size(); i++)
        {
            for (size_t d = 0; d < 2; d++) {
                _predictedObservation(2*i+d) = predicted2DStateEdit[i][d];
            }
        }

    }
    else // 3D Observations
    {
        for (size_t i = 0; i < predicted3DStateEdit.size(); i++)
        {
            for (size_t d = 0; d < 3; d++)
                _predictedObservation(3*i+d) = predicted3DStateEdit[i][d];
        }
    }

    return (true);
}



template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleObservationManager<FilterType, DataTypes1, DataTypes2>::getPredictedObservation(int _id, EVectorX& _predictedObservation)
{
    const sofa::type::Mat3x4d & P = d_projectionMatrix.getValue();
    _predictedObservation.resize(this->observationSize);

    Data< typename DataTypes1::VecCoord > predicted2DState;
    Data< typename DataTypes2::VecCoord > predicted3DState;

    typename DataTypes1::VecCoord& predicted2DStateEdit = *predicted2DState.beginEdit();
    typename DataTypes2::VecCoord& predicted3DStateEdit = *predicted3DState.beginEdit();

    predicted3DStateEdit.resize(masterState->getSize());

    stateWrapper->getActualPosition(_id, predicted3DStateEdit);
    predicted2DStateEdit.resize(predicted3DStateEdit.size());

    //2D observations
    if(ObservationSource::Coord::total_size == 2)
    {
        for (unsigned i = 0; i < predicted3DStateEdit.size(); i++)
        {
            double rx = P[0][0] * predicted3DStateEdit[i][0] + P[0][1] * predicted3DStateEdit[i][1] + P[0][2] * predicted3DStateEdit[i][2] + P[0][3];
            double ry = P[1][0] * predicted3DStateEdit[i][0] + P[1][1] * predicted3DStateEdit[i][1] + P[1][2] * predicted3DStateEdit[i][2] + P[1][3];
            double rz = P[2][0] * predicted3DStateEdit[i][0] + P[2][1] * predicted3DStateEdit[i][1] + P[2][2] * predicted3DStateEdit[i][2] + P[2][3];
            predicted2DStateEdit[i][0] = rx * (1.0 / rz);
            predicted2DStateEdit[i][1] = ry * (1.0 / rz);
        }

        for (size_t i = 0; i < predicted3DStateEdit.size(); i++)
        {
            for (size_t d = 0; d < 2; d++) {
                _predictedObservation(2 * i + d) = predicted2DStateEdit[i][d];
            }
        }
    } else {
        // 3D Observations
        for (size_t i = 0; i < predicted3DStateEdit.size(); i++)
        {
            for (size_t d = 0; d < 3; d++)
                _predictedObservation(3 * i + d) = predicted3DStateEdit[i][d];
        }
    }
    return (true);
}



} // stochastic

} // component

} // sofa

