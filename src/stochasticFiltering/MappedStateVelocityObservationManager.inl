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
#ifndef MAPPEDSTATEVELOCITYOBSERVATIONMANAGER_INL
#define MAPPEDSTATEVELOCITYOBSERVATIONMANAGER_INL

#include <sofa/simulation/Node.h>

#include "MappedStateVelocityObservationManager.h"


namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType, class DataTypes1, class DataTypes2>
MappedStateVelocityObservationManager<FilterType,DataTypes1,DataTypes2>::MappedStateVelocityObservationManager()
    : Inherit()
    , inputObservationData( initData (&inputObservationData, "observations", "observations read from a file") )
    , mappedObservationData( initData (&mappedObservationData, "mappedObservations", "mapped observations") )
    , noiseStdev( initData(&noiseStdev, double(0.0), "noiseStdev", "standard deviation of generated noise") )
    , abberantIndex( initData(&abberantIndex, int(-1), "abberantIndex", "index of an aberrant point") )
    , doNotMapObservations( initData(&doNotMapObservations, false, "doNotMapObservations", "if real observations are read from a file (not the mechanical object)") )
    , d_observeVelocities( initData(&d_observeVelocities, false, "observeVelocities", "predict and get from real data velocity values together with observations") )
    , d_velocityObservationStdev( initData(&d_velocityObservationStdev, FilterType(0.0), "velocityObservationStdev", "standard deviation in velocity observations") )
    , d_observationIndices( initData(&d_observationIndices, "observationIndices", "take these indices from vector of observations (implies not using mapping)") )
    , stateWrapperLink(initLink("stateWrapper", "link to the state wrapper needed to perform apply (perhaps to be changed)"))
{    
}

template <class FilterType, class DataTypes1, class DataTypes2>
void MappedStateVelocityObservationManager<FilterType,DataTypes1,DataTypes2>::init()
{
    Inherit::init();

    this->gnode->get(mapping);
    if (mapping) {
        PRNS("Found mapping: " << mapping->getName());
    } else {
        PRNE("No mapping found!");
    }

    this->gnode->template get<ObservationSource >(&observationSources, this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);
    if (observationSources.size() == 0) {
        PRNE("No observation source found!");
    } else {
        PRNS("Found " << observationSources.size() << " observation sources");
        for (unsigned int index = 0; index < observationSources.size(); index++) {
            PRNS("Found observation source: " << observationSources[index]->getName());
        }
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
        doNotMapObservations.setValue(true);
    }

}

template <class FilterType, class DataTypes1, class DataTypes2>
void MappedStateVelocityObservationManager<FilterType,DataTypes1,DataTypes2>::bwdInit()
{
    if (!Inherit::initialiseObservationsAtFirstStep.getValue()) {
        initializeObservationData();
    }

    Inherit::bwdInit();
}

template <class FilterType, class DataTypes1, class DataTypes2>
void MappedStateVelocityObservationManager<FilterType,DataTypes1,DataTypes2>::initializeObservationData()
{   
    masterStateSize = masterState->getSize();
    mappedStateSize = mappedState->getSize();

    if (doNotMapObservations.getValue()) {
        if (observationIndices.size() == 0) {
            observationIndices.resize(mappedStateSize);
            for (size_t i = 0; i < mappedStateSize; i++)
                observationIndices[i] = i;
        }
        inputStateSize = observationIndices.size();
    } else {
        inputStateSize = observationSources[0]->getStateSize();
    }

    inputVectorSize = inputStateSize*DataTypes1::spatial_dimensions;
    masterVectorSize = masterStateSize*DataTypes1::spatial_dimensions;
    mappedVectorSize = mappedStateSize*DataTypes1::spatial_dimensions;

    typename DataTypes1::VecCoord& inputObsState = *inputObservationData.beginEdit();
    inputObsState.resize(inputStateSize);
    observationSources[0]->getStateAtTime(0.0, inputObsState);

    // copy velocities to observation data
    typename DataTypes1::VecCoord& inputVelocityObsState = *inputVelocityObservationData.beginEdit();
    if (d_observeVelocities.getValue()) {
        inputObsState.resize(inputStateSize);
        observationSources[1]->getStateAtTime(0.0, inputVelocityObsState);
    }

    if (doNotMapObservations.getValue()) {
        if (inputStateSize != mappedStateSize) {
            PRNE("In non-mapping version, |input observation state| !=  |mapped state| " << inputStateSize << " vs. " << mappedStateSize);
            return;
        }
        PRNS("Non-mapping version, |observations| = |input observation state| = |mapped state| = " << inputStateSize);
        sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObsState = mappedObservationData;
        unsigned int stateSizeScale = d_observeVelocities.getValue() ? 2 : 1;
        mappedObsState.resize(stateSizeScale * mappedStateSize);
        this->observationSize = stateSizeScale * mappedVectorSize;
        for (size_t i = 0; i < mappedStateSize; i++)
            mappedObsState[i] = inputObsState[observationIndices[i]];

        if (d_observeVelocities.getValue()) {
            for (size_t i = 0; i < mappedStateSize; i++)
                mappedObsState[mappedStateSize + i] = inputVelocityObsState[observationIndices[i]];
        }

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

    Inherit::initializeObservationData();
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool MappedStateVelocityObservationManager<FilterType,DataTypes1,DataTypes2>::hasObservation(double _time) {
    if (Inherit::initialiseObservationsAtFirstStep.getValue()) {
        initializeObservationData();
        Inherit::initialiseObservationsAtFirstStep.setValue(false);
    }

    typename DataTypes1::VecCoord& inputObsState = *inputObservationData.beginEdit();
    //PRNS("Getting observation at time " << this->actualTime);
    bool hasObservation = observationSources[0]->getObservation(this->actualTime, inputObsState);

    if (!hasObservation) {
        PRNE("No observation for time " << _time);
        return(false);
    }

    /// put the observation imported from a file via mapping  (yields actualObservation)
    actualObservation.setZero();
    if (!this->doNotMapObservations.getValue()) {
        sofa::core::MechanicalParams mp;
        //std::cout << "Input observation: " << inputObsState << std::endl;
        mapping->apply(&mp, mappedObservationData, inputObservationData);
        sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObsState = mappedObservationData;
        //std::cout << "Mapped observation: " << mappedObsState << std::endl;

        for (size_t i = 0; i < mappedStateSize; i++) {
            for (size_t d = 0; d < 3; d++) {
                if (noiseStdev.getValue() != 0.0)
                    mappedObsState[i][d] += (*pVarNorm)();
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
            mappedObsState[i] = inputObsState[observationIndices[i]];

        for (size_t i = 0; i < mappedObsState.size(); i++) {
            for (size_t d = 0; d < 3; d++) {
                if (noiseStdev.getValue() != 0.0)
                    mappedObsState[i][d] += (*pVarNorm)();
                actualObservation(3*i+d) = mappedObsState[i][d];                
            }
        }
    }
    return(true);
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool MappedStateVelocityObservationManager<FilterType,DataTypes1,DataTypes2>::getPredictedObservation(int _id, EVectorX& _predictedObservation) {


    Data<typename DataTypes1::VecCoord> predictedMasterState;
    Data<typename DataTypes2::VecCoord> predictedMappedState;

    typename DataTypes1::VecCoord& predictedMasterStateEdit = *predictedMasterState.beginEdit();
    typename DataTypes2::VecCoord& predictedMappedStateEdit = *predictedMappedState.beginEdit();

    predictedMasterStateEdit.resize(masterState->getSize());
    predictedMappedStateEdit.resize(mappedState->getSize());

    stateWrapper->getActualPosition(_id, predictedMasterStateEdit);
    //stateWrapper->setSofaVectorFromFilterVector(_state, predictedMasterStateEdit);
    sofa::core::MechanicalParams mp;

    sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > masterState = predictedMasterState;

    mapping->apply(&mp, predictedMappedState, predictedMasterState);

    _predictedObservation.resize(this->observationSize);
    for (size_t i = 0; i < predictedMappedStateEdit.size(); i++)
        for (size_t d = 0; d < 3; d++)
            _predictedObservation(3*i+d) = predictedMappedStateEdit[i][d];        

    return true;
}
template <class FilterType, class DataTypes1, class DataTypes2>
bool MappedStateVelocityObservationManager<FilterType,DataTypes1,DataTypes2>::obsFunction(EVectorX& _state, EVectorX& _predictedObservation)
{
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool MappedStateVelocityObservationManager<FilterType,DataTypes1,DataTypes2>::getRealObservation(double _time, EVectorX& _realObs)
{
}


template <class FilterType, class DataTypes1, class DataTypes2>
bool MappedStateVelocityObservationManager<FilterType,DataTypes1,DataTypes2>::getInnovation(double _time, EVectorX& _state, EVectorX& _innovation)
{
    if (_time != this->actualTime) {
        PRNE("Observation for time " << this->actualTime << " not prepared, call hasObservation first!");
        return(false);
    }

    if (_innovation.rows() != long(this->observationSize)) {
        PRNE("Wrong innovation size: " << _innovation.rows() << " should be " << this->observationSize);
        return(false);
    }

    _innovation.resize(this->observationSize);
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
            for (size_t d = 0; d < 3; d++)
                _innovation(3*i+d) = actualObservation(3*i+d) - predictedMappedStateEdit[i][d];
    }

    /// TEMPORARY: _state here is the predicted observation computed before
    if ((stateWrapper->getFilterKind() == SIMCORR) || (stateWrapper->getFilterKind() == CLASSIC)) {
            for (size_t i = 0; i < this->observationSize; i++)
                _innovation(i) = actualObservation(i) - _state(i);
    }

    return(true);


    /*
    //std::cout << this->getName() << ": size of mapped state: " << mappedState.size() << std::endl;
    this->innovation_.Reallocate(mappedState.size()*3);
    for (size_t i = 0; i < mappedState.size(); i++)
        for (size_t d = 0; d < 3; d++)
            this->innovation_(3*i+d) = mappedState[i][d];

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

#endif // MAPPEDSTATEVELOCITYOBSERVATIONMANAGER_INL
