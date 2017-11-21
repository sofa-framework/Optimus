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
#ifndef SOFASTATEOBSERVATIONWRAPPER_INL
#define SOFASTATEOBSERVATIONWRAPPER_INL


#include "StochasticStateObservationWrapper.h"
#include "StochasticStateWrapper.inl"


namespace sofa
{
namespace component
{
namespace stochastic
{

using namespace sofa::simulation;

template <class DataTypes, class FilterType>
StochasticStateObservationWrapper<DataTypes, FilterType>::StochasticStateObservationWrapper()
    :Inherit()
{
}

template <class DataTypes, class FilterType>
StochasticStateObservationWrapper<DataTypes, FilterType>::~StochasticStateObservationWrapper()
{
    Inherit::~StochasticStateWrapper();
}

template <class DataTypes, class FilterType>
void StochasticStateObservationWrapper<DataTypes, FilterType>::init()
{
    Inherit::init();
}

template <class DataTypes, class FilterType>
void StochasticStateObservationWrapper<DataTypes, FilterType>::bwdInit() {
    Inherit::bwdInit();

    observationPosPairs.clear();

    size_t vsi = 0;

    for (size_t i = 0; i < this->freeNodes.size(); i++) {
        std::pair<size_t, size_t> pr(this->freeNodes[i], vsi++);
        observationPosPairs.push_back(pr);
    }

    this->obsState.resize(Inherit::Dim * vsi);
    copyStateSofa2Filter();
}

template <class DataTypes, class FilterType>
void StochasticStateObservationWrapper<DataTypes, FilterType>::setSofaVectorFromFilterVector(EVectorX& _state, typename DataTypes::VecCoord& _vec) {
    Inherit::setSofaVectorFromFilterVector(_state, _vec);
}

template <class DataTypes, class FilterType>
void StochasticStateObservationWrapper<DataTypes, FilterType>::setSofaVectorFromObservationsStateVector(EVectorX& _observationsState, typename DataTypes::VecCoord& _vec) {
    if (_vec.size() != Inherit::mechanicalState->getSize()) {
        PRNE("Input vector not compatible with the actual Sofa state size");
        return;
    }

    typename MechanicalState::ReadVecCoord pos = Inherit::mechanicalState->readPositions();
    for (size_t fni = 0; fni < Inherit::fixedNodes.size(); fni++) {
        size_t fn = Inherit::fixedNodes[fni];
        _vec[fn] = pos[fn];
        //PRNS("Setting fixed[" << fn << "] = " << _vec[fn]);
    }

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = observationPosPairs.begin(); it != observationPosPairs.end(); it++) {
        if (it->first >= _vec.size()) {
            PRNE("Accessing Sofa vector out of bounds: " << it->first <<  " vs. " << _vec.size());
            return;
        }

        if ((Inherit::Dim*it->second + Inherit::Dim) > _observationsState.rows()) {
            PRNE("Accessing DA vector out of bounds: " << Inherit::Dim*it->second + Inherit::Dim <<  " vs. " << _observationsState.rows());
            return;
        }

        for (size_t d = 0; d < Inherit::Dim; d++) {
            _vec[it->first][d] = _observationsState(Inherit::Dim*it->second + d);
        }
        //PRNS("Setting free[" << it->first << "] = " << _vec[it->first]);
    }
}

template <class DataTypes, class FilterType>
void StochasticStateObservationWrapper<DataTypes, FilterType>::copyStateFilter2Sofa(const core::MechanicalParams* _mechParams) {
    Inherit::copyStateFilter2Sofa(_mechParams);
}

template <class DataTypes, class FilterType>
void StochasticStateObservationWrapper<DataTypes, FilterType>::copyStateSofa2Filter() {
    Inherit::copyStateSofa2Filter();
    typename MechanicalState::ReadVecCoord pos = this->mechanicalState->readPositions();

    for (helper::vector<std::pair<size_t, size_t> >::iterator it = observationPosPairs.begin(); it != observationPosPairs.end(); it++)
        for (size_t d = 0; d < Inherit::Dim; d++) {
            this->obsState(Inherit::Dim*it->second + d) = pos[it->first][d];
        }
}

template <class DataTypes, class FilterType>
void StochasticStateObservationWrapper<DataTypes, FilterType>::applyOperator(EVectorX &_vecX, const core::MechanicalParams *_mparams, bool _preserveState, bool _updateForce) {
    Inherit::applyOperator(_vecX, _mparams, _preserveState, _updateForce);
}

template <class DataTypes, class FilterType>
void StochasticStateObservationWrapper<DataTypes, FilterType>::computeSofaStep(const core::ExecParams* execParams, bool _updateTime) {
    Inherit::computeSofaStep(execParams, _updateTime);
}

template <class DataTypes, class FilterType>
void StochasticStateObservationWrapper<DataTypes, FilterType>::computeSofaStepWithLM(const core::ExecParams* params, bool _updateTime) {
    Inherit::computeSofaStepWithLM(params, _updateTime);
}


} // simulation
} // component
} // sofa

#endif // SOFASTATEOBSERVATIONWRAPPER_INL
