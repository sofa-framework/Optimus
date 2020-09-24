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

#include "SimpleUncorrespondentObservationManager.h"


namespace sofa
{

namespace component
{

namespace stochastic
{

template <class FilterType, class DataTypes1, class DataTypes2>
SimpleUncorrespondentObservationManager<FilterType,DataTypes1,DataTypes2>::SimpleUncorrespondentObservationManager()
    : Inherit()
    , inputObservationData( initData (&inputObservationData, "observations", "observations read from a file") )
    , inputIndices( initData (&inputIndices, "indices", "indices read from a file") )
    , mappedObservationData( initData (&mappedObservationData, "mappedObservations", "mapped observations") )
    , noiseStdev( initData(&noiseStdev, double(0.0), "noiseStdev", "standard deviation of generated noise") )
    , abberantIndex( initData(&abberantIndex, int(-1), "abberantIndex", "index of an aberrant point") )
    , d_observationIndices( initData(&d_observationIndices, "observationIndices", "take these indices from vector of observations (implies not using mapping)") )
    , stateWrapperLink(initLink("stateWrapper", "link to the state wrapper needed to perform apply (perhaps to be changed)"))
{    
}

template <class FilterType, class DataTypes1, class DataTypes2>
void SimpleUncorrespondentObservationManager<FilterType,DataTypes1,DataTypes2>::init()
{
    Inherit::init();

    this->gnode->get(mapping);
    if (mapping) {
        PRNS("Found mapping: " << mapping->getName());
    } else {
        PRNE("No mapping found!");
    }

    this->gnode->get(observationSource);
    if (observationSource) {
        PRNS("Found observation source: " << observationSource->getName());
    } else {
        PRNE("No observation source found!");
    }

    stateWrapper = stateWrapperLink.get();
    if (stateWrapper) {
        PRNS("Link to state wrapper: " << stateWrapper->getName());
    } else {
        PRNE("Link to state wrapper not initialized!");
    }

    this->gnode->get(mappedState);
    if (mappedState) {
        PRNS("Found slave mechanical state: " << mappedState->getName());
    } else {
        PRNE("No slave mechanical state found!");
    }

    masterState = dynamic_cast<MasterState*>(mapping->getFromModel());
    if (masterState != NULL) {
        PRNS("Found master mechanical state: " << masterState->getName());
    }
    else {
        PRNE("No master mechanical state found!");
    }

    /// initialize noise generator:
    if (noiseStdev.getValue() != 0.0) {
        pRandGen = new boost::mt19937;
        pNormDist = new boost::normal_distribution<>(0.0, noiseStdev.getValue());
        pVarNorm = new boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >(*pRandGen, *pNormDist);        
    }

    observationIndices = d_observationIndices.getValue();
    if (observationIndices.size() > 0) {
        PRNS("Found indices which will be taken as observations: " << observationIndices);
    }
}


template <class FilterType, class DataTypes1, class DataTypes2>
void SimpleUncorrespondentObservationManager<FilterType,DataTypes1,DataTypes2>::bwdInit()
{
    if (!Inherit::initialiseObservationsAtFirstStep.getValue()) {
        initializeObservationData();
    }

    Inherit::bwdInit();
}

template <class FilterType, class DataTypes1, class DataTypes2>
void SimpleUncorrespondentObservationManager<FilterType, DataTypes1, DataTypes2>::initializeObservationData()
{   
    masterStateSize = masterState->getSize();
    mappedStateSize = mappedState->getSize();

    if (observationIndices.size() == 0) {
        observationIndices.resize(mappedStateSize);
        for (size_t i = 0; i < mappedStateSize; i++)
            observationIndices[i] = i;
    }
    inputStateSize = observationIndices.size();

    inputVectorSize = inputStateSize*DataTypes1::spatial_dimensions;
    masterVectorSize = masterStateSize*DataTypes1::spatial_dimensions;
    mappedVectorSize = mappedStateSize*DataTypes1::spatial_dimensions;

    typename DataTypes1::VecCoord& inputObsState = *inputObservationData.beginEdit();
    inputObsState.resize(inputStateSize);
    observationSource->getStateAtTime(0.0, inputObsState);

    if (inputStateSize != mappedStateSize) {
        PRNE("In non-mapping version, |input observation state| !=  |mapped state| " << inputStateSize << " vs. " << mappedStateSize);
        return;
    }
    PRNS("Non-mapping version, |observations| = |input observation state| = |mapped state| = " << inputStateSize);
    sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObsState = mappedObservationData;
    mappedObsState.resize(mappedStateSize);
    sofa::helper::WriteAccessor< Data<VecIndex> > vMappedMask = mappedMask;
    vMappedMask.resize(mappedStateSize);

    this->observationSize = mappedVectorSize;
    for (size_t i = 0; i < mappedStateSize; i++)
        mappedObsState[i] = inputObsState[observationIndices[i]];

    actualObservation.resize(this->observationSize);
    noise.clear();
    noise.resize(this->observationSize);

    Inherit::initializeObservationData();
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleUncorrespondentObservationManager<FilterType, DataTypes1, DataTypes2>::hasObservation(double _time) {
    if (Inherit::initialiseObservationsAtFirstStep.getValue()) {
        initializeObservationData();
        Inherit::initialiseObservationsAtFirstStep.setValue(false);
    }

    typename DataTypes1::VecCoord& inputObsState = *inputObservationData.beginEdit();
    VecIndex& inputIndexData = *inputIndices.beginEdit();
    //PRNS("Getting observation at time " << this->actualTime);
    bool hasObservation = observationSource->getObservation(this->actualTime, inputObsState);
    bool hasIndices = observationSource->getCorrespondentIndices(this->actualTime, inputIndexData);

    if (!hasObservation || ! hasIndices) {
        PRNE("No observation for time " << _time);
        return (false);
    }

    actualObservation.setZero();

    sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObsState = mappedObservationData;
    sofa::helper::WriteAccessor< Data<VecIndex> > vMappedMask = mappedMask;
    for (size_t i = 0; i < vMappedMask.size(); i++)
        vMappedMask[i] = 0;


    // get set of available vertices
    for (size_t i = 0; i < inputObsState.size(); i++) {
        // found index in state correspondent to input
        for (unsigned int foundIndex = 0; foundIndex < observationIndices.size(); foundIndex++) {
            if (inputIndexData[i] == observationIndices[foundIndex]) {
                mappedObsState[foundIndex] = inputObsState[i];
                vMappedMask[foundIndex] = 1;
            }
        }
    }

    for (size_t i = 0; i < mappedObsState.size(); i++) {
        for (size_t d = 0; d < DataTypes1::spatial_dimensions; d++) {
            if (noiseStdev.getValue() != 0.0)
                mappedObsState[i][d] += (*pVarNorm)();
            actualObservation(DataTypes1::spatial_dimensions * i + d) = mappedObsState[i][d];
        }
    }

    return (true);
}


template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleUncorrespondentObservationManager<FilterType, DataTypes1, DataTypes2>::getPredictedObservation(int _id, EVectorX& _predictedObservation) {


    Data<typename DataTypes1::VecCoord> predictedMasterState;
    Data<typename DataTypes2::VecCoord> predictedMappedState;

    typename DataTypes1::VecCoord& predictedMasterStateEdit = *predictedMasterState.beginEdit();
    typename DataTypes2::VecCoord& predictedMappedStateEdit = *predictedMappedState.beginEdit();

    predictedMasterStateEdit.resize(masterState->getSize());
    predictedMappedStateEdit.resize(mappedState->getSize());

    stateWrapper->getActualPosition(_id, predictedMasterStateEdit);
    //stateWrapper->setSofaVectorFromFilterVector(_state, predictedMasterStateEdit);
    sofa::core::MechanicalParams mp;

    //sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > masterState = predictedMasterState;

    mapping->apply(&mp, predictedMappedState, predictedMasterState);

    _predictedObservation.resize(this->observationSize);
    for (size_t i = 0; i < predictedMappedStateEdit.size(); i++)
        for (size_t d = 0; d < DataTypes1::spatial_dimensions; d++)
            _predictedObservation(DataTypes1::spatial_dimensions * i + d) = predictedMappedStateEdit[i][d];

    return true;
}
template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleUncorrespondentObservationManager<FilterType, DataTypes1, DataTypes2>::obsFunction(EVectorX& /* _state */, EVectorX& /* _predictedObservation */)
{
    return 0;
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleUncorrespondentObservationManager<FilterType, DataTypes1, DataTypes2>::getRealObservation(double /* _time */, EVectorX& /* _realObs */)
{
    return 0;
}


template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleUncorrespondentObservationManager<FilterType, DataTypes1, DataTypes2>::getInnovation(double _time, EVectorX& _state, EVectorX& _innovation)
{
    if (_time != this->actualTime) {
        PRNE("Observation for time " << this->actualTime << " not prepared, call hasObservation first!");
        return (false);
    }

    if (_innovation.rows() != long(this->observationSize)) {
        PRNE("Wrong innovation size: " << _innovation.rows() << " should be " << this->observationSize);
        return (false);
    }

    _innovation.resize(this->observationSize);
    sofa::helper::ReadAccessor< Data<VecIndex> > vMappedMask = mappedMask;
    if (stateWrapper->getFilterKind() == REDORD) {
        Data<typename DataTypes1::VecCoord> predictedMasterState;
        Data<typename DataTypes2::VecCoord> predictedMappedState;

        typename DataTypes1::VecCoord& predictedMasterStateEdit = *predictedMasterState.beginEdit();
        typename DataTypes2::VecCoord& predictedMappedStateEdit = *predictedMappedState.beginEdit();

        predictedMasterStateEdit.resize(masterState->getSize());
        predictedMappedStateEdit.resize(mappedState->getSize());

        stateWrapper->setSofaVectorFromFilterVector(_state, predictedMasterStateEdit);
        sofa::core::MechanicalParams mp;
        mapping->apply(&mp, predictedMappedState, predictedMasterState);

        for (size_t i = 0; i < predictedMappedStateEdit.size(); i++)
            for (size_t d = 0; d < DataTypes1::spatial_dimensions; d++)
                _innovation(DataTypes1::spatial_dimensions * i + d) = vMappedMask[i] * (actualObservation(DataTypes1::spatial_dimensions * i + d) - predictedMappedStateEdit[i][d]);
    }

    /// TEMPORARY: _state here is the predicted observation computed before
    if ((stateWrapper->getFilterKind() == SIMCORR) || (stateWrapper->getFilterKind() == CLASSIC)) {
            for (size_t i = 0; i < this->observationSize; i++)
                _innovation(i) = vMappedMask[i / DataTypes1::spatial_dimensions] * (actualObservation(i) - _state(i));
    }

    return (true);


    /*
    //std::cout << this->getName() << ": size of mapped state: " << mappedState.size() << std::endl;
    this->innovation_.Reallocate(mappedState.size()*DataTypes1::spatial_dimensions);
    for (size_t i = 0; i < mappedState.size(); i++)
        for (size_t d = 0; d < DataTypes1::spatial_dimensions; d++)
            this->innovation_(DataTypes1::spatial_dimensions * i + d) = mappedState[i][d];

    //std::cout << "AKDEBUG " << this->innovation_ << std::endl;


    //this->innovation_.Reallocate(this->Nobservation_);
    //this->ApplyOperator(x, this->innovation_);
    //Inherit::observation predObs  = this->innovation_;
    Mlt(double(-1.0), this->innovation_);
    //Add(double(1.0), this->GetObservation(), this->innovation_);
    Add(double(1.0), actualObs, this->innovation_);
    //std::cout << this->getName() << ": innovation updated" << std::endl;

    //for (size_t i = 0; i < this->innovation_.GetM(); i++)
    //    std::cout << actualObs(i) << " " << predObs(i) << " " << this->innovation_(i) << std::endl;

    //std::cout << "ERROR VARIANCE: " << this->error_variance_ << std::endl;

    inputObservationData.endEdit();
    //mappedObservationData.endEdit();
    actualStateData.endEdit();
    mappedStateData.endEdit();

    //std::cout << "MPP: " << mappedObservationData.getValue() << std::endl;

    return this->innovation_;
    */

}



//template <class DataTypes>
//void MappedStateUncorrespondentObservationManager<DataTypes>::init()
//{
//
//}
//
//template <class DataTypes>
//void MappedStateUncorrespondentObservationManager<DataTypes>::reinit()
//{
//}


} // namespace stochastic

} // namespace component

} // namespace sofa

