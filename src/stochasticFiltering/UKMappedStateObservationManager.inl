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
#ifndef UKMAPPEDSTATEOBSERVATIONMANAGER_INL
#define UKMAPPEDSTATEOBSERVATIONMANAGER_INL

#include <sofa/simulation/Node.h>

#include "UKMappedStateObservationManager.h"


namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType, class DataTypes1, class DataTypes2>
UKMappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::UKMappedStateObservationManager()
    : Inherit()
    , inputObservationData( initData (&inputObservationData, "observations", "observations read from a file") )
    , mappedObservationData( initData (&mappedObservationData, "mappedObservations", "mapped observations") )
    , noiseStdev( initData(&noiseStdev, double(0.0), "noiseStdev", "standard deviation of generated noise") )
    , abberantIndex( initData(&abberantIndex, int(-1), "abberantIndex", "index of an aberrant point") )
    , doNotMapObservations( initData(&doNotMapObservations, false, "doNotMapObservations", "if real observations are read from a file (not the mechanical object)") )
    , stateWrapperLink(initLink("stateWrapper", "link to the state wrapper needed to perform apply (perhaps to be changed)"))
{    
}

template <class FilterType, class DataTypes1, class DataTypes2>
void UKMappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::init()
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
}

template <class FilterType, class DataTypes1, class DataTypes2>
void UKMappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::bwdInit()
{
    inputStateSize = observationSource->getStateSize();
    masterStateSize = masterState->getSize();
    mappedStateSize = mappedState->getSize();    

    inputVectorSize = inputStateSize*DataTypes1::spatial_dimensions;
    masterVectorSize = masterStateSize*DataTypes1::spatial_dimensions;
    mappedVectorSize = mappedStateSize*DataTypes1::spatial_dimensions;

    typename DataTypes1::VecCoord& inputObsState = *inputObservationData.beginEdit();
    inputObsState.resize(inputStateSize);
    observationSource->getStateAtTime(0.0, inputObsState);

    if (doNotMapObservations.getValue()) {
        if (inputStateSize != mappedStateSize) {
            PRNE("In non-mapping version, |input observation state| !=  |mapped state| " << inputStateSize << " vs. " << mappedStateSize);
            return;
        }
        PRNS("Non-mapping version, |observations| = |input observation state| = |mapped state| = " << inputStateSize);
        sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObsState = mappedObservationData;
        mappedObsState.resize(mappedStateSize);
        this->observationSize = mappedVectorSize;
        for (size_t i = 0; i < mappedStateSize; i++)
            mappedObsState[i] = inputObsState[i];

    } else {
        if (inputStateSize != masterStateSize) {
            PRNE("In mapping version, |input observation state| !=  |master state| " << inputStateSize << " vs. " << masterStateSize);
            return;
        }
        PRNS("Mapping version, |observations| = " << mappedVectorSize << " |input observation state| = |master state| = " << inputStateSize);
        sofa::core::MechanicalParams mp;
        mapping->apply(&mp, mappedObservationData, inputObservationData);
        this->observationSize = mappedVectorSize;
    }

    actualObservation.resize(this->observationSize);
    noise.clear();
    noise.resize(this->observationSize);
    Inherit::bwdInit();    
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool UKMappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::hasObservation(double _time) {
    typename DataTypes1::VecCoord& inputObsState = *inputObservationData.beginEdit();
    //PRNS("Getting observation at time " << this->actualTime);
    bool hasObservation = observationSource->getObservation(this->actualTime, inputObsState);

    if (!hasObservation) {
        PRNE("No observation for time " << _time);
        return(false);
    }

    actualObservation.setZero();
    if (!this->doNotMapObservations.getValue()) {
       sofa::core::MechanicalParams mp;
       //std::cout << "Input observation: " << inputObsState << std::endl;
       mapping->apply(&mp, mappedObservationData, inputObservationData);
       sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObsState = mappedObservationData;
       //std::cout << "Mapped observation: " << mappedObsState << std::endl;

        for (size_t i = 0; i < mappedStateSize; i++) {
            for (size_t d = 0; d < 3; d++) {
                //mappedObsState[i][d] += noise[3*i+d];
                actualObservation(3*i+d) = mappedObsState[i][d];
            }
        }
    } else {
        sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObsState = mappedObservationData;
        if (mappedObsState.size() != inputObsState.size()) {
            PRNE("Different mapped and input observation size: " << mappedObsState.size() << " vs " << inputObsState.size());
            return(false);
        }

        for (size_t i = 0; i < mappedObsState.size(); i++)
            mappedObsState[i] = inputObsState[i];

        for (size_t i = 0; i < mappedObsState.size(); i++) {
            for (size_t d = 0; d < 3; d++) {
                actualObservation(3*i+d) = mappedObsState[i][d];
            }
        }
    }
    return(true);
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool UKMappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::predictedObservation(double _time, EVectorX& _state, EVectorX& _predictedObservation, EVectorX& _innovation)
{
    if (_time != this->actualTime) {
        PRNE("Observation for time " << this->actualTime << " not prepare, call hasObservation first!");
        return(false);
    }

    if (_innovation.rows() != long(this->observationSize)) {
        PRNE("Wrong innovation size: " << _innovation.rows() << " should be " << this->observationSize);
        return(false);
    }

    Data<typename DataTypes1::VecCoord> predictedMasterState;
    Data<typename DataTypes2::VecCoord> predictedMappedState;

    typename DataTypes1::VecCoord& predictedMasterStateEdit = *predictedMasterState.beginEdit();
    typename DataTypes2::VecCoord& predictedMappedStateEdit = *predictedMappedState.beginEdit();

    predictedMasterStateEdit.resize(masterState->getSize());
    predictedMappedStateEdit.resize(mappedState->getSize());

    stateWrapper->setSofaVectorFromFilterVector(_state, predictedMasterStateEdit);
    sofa::core::MechanicalParams mp;
    mapping->apply(&mp, predictedMappedState, predictedMasterState);

    _predictedObservation.resize(this->observationSize);
    for (size_t i = 0; i < predictedMappedStateEdit.size(); i++)
        for (size_t d = 0; d < 3; d++)
            _predictedObservation(3*i+d) = predictedMappedStateEdit[i][d];

    return(true);
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool UKMappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::getInnovation(double _time, EVectorX& _state, EVectorX& _innovation)
{
    if (_time != this->actualTime) {
        PRNE("Observation for time " << this->actualTime << " not prepare, call hasObservation first!");
        return(false);
    }

    _innovation.resize(this->observationSize);

    for (size_t i = 0; i < (size_t)_state.size() / 3; i++)
        _innovation(i) = actualObservation(i) - _state(i);

    return(true);
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
