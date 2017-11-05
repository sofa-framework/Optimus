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
#ifndef SIMPLEOBSERVATIONMANAGER_INL_
#define SIMPLEOBSERVATIONMANAGER_INL_

#include <sofa/simulation/Node.h>

#include "SimpleObservationManager.h"


namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType, class DataTypes1, class DataTypes2>
SimpleObservationManager<FilterType,DataTypes1,DataTypes2>::SimpleObservationManager()
    : Inherit()
    , noiseStdev( initData(&noiseStdev, double(0.0), "noiseStdev", "standard deviation of generated noise") )
    , abberantIndex( initData(&abberantIndex, int(-1), "abberantIndex", "index of an aberrant point") )
    , doNotMapObservations( initData(&doNotMapObservations, false, "doNotMapObservations", "if real observations are read from a file (not the mechanical object)") )
    , stateWrapperLink(initLink("stateWrapper", "link to the state wrapper needed to perform apply (perhaps to be changed)"))
{
}

template <class FilterType, class DataTypes1, class DataTypes2>
void SimpleObservationManager<FilterType,DataTypes1,DataTypes2>::init()
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
        PRNS("Link to state wrapper: " << stateWrapper->getName());
    } else {
        PRNE("Link to state wrapper not initialized!");
    }

//    masterState = dynamic_cast<MasterState*>(mapping->getFromModel());
//    if (masterState != NULL) {
//        PRNS("Found master mechanical state: " << masterState->getName());
//    }
//    else {
//        PRNE("No master mechanical state found!");
//    }

//    /// initialize noise generator:
//    if (noiseStdev.getValue() != 0.0) {
//        pRandGen = new boost::mt19937;
//        pNormDist = new boost::normal_distribution<>(0.0, noiseStdev.getValue());
//        pVarNorm = new boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >(*pRandGen, *pNormDist);
//    }
}

template <class FilterType, class DataTypes1, class DataTypes2>
void SimpleObservationManager<FilterType,DataTypes1,DataTypes2>::bwdInit()
{
//    std::cout << "!!!!! size: " << observationSource->getStateSize() << std::endl;
    this->observationSize = observationSource->getStateSize() * DataTypes1::spatial_dimensions;
//    inputStateSize = observationSource->getStateSize();
//    masterStateSize = masterState->getSize();
//    mappedStateSize = mappedState->getSize();

//    inputVectorSize = inputStateSize*DataTypes1::spatial_dimensions;
//    masterVectorSize = masterStateSize*DataTypes1::spatial_dimensions;
//    mappedVectorSize = mappedStateSize*DataTypes1::spatial_dimensions;

//    typename DataTypes1::VecCoord& inputObsState = *inputObservationData.beginEdit();
//    inputObsState.resize(inputStateSize);
//    observationSource->getStateAtTime(0.0, inputObsState);

//    if (doNotMapObservations.getValue()) {
//        if (inputStateSize != mappedStateSize) {
//            PRNE("In non-mapping version, |input observation state| !=  |mapped state| " << inputStateSize << " vs. " << mappedStateSize);
//            return;
//        }
//        PRNS("Non-mapping version, |observations| = |input observation state| = |mapped state| = " << inputStateSize);
//        sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObsState = mappedObservationData;
//        mappedObsState.resize(mappedStateSize);
//        this->observationSize = mappedVectorSize;
//        for (size_t i = 0; i < mappedStateSize; i++)
//            mappedObsState[i] = inputObsState[i];

//    } else {
//        if (inputStateSize != masterStateSize) {
//            PRNE("In mapping version, |input observation state| !=  |master state| " << inputStateSize << " vs. " << masterStateSize);
//            return;
//        }
//        PRNS("Mapping version, |observations| = " << mappedVectorSize << " |input observation state| = |master state| = " << inputStateSize);
//        sofa::core::MechanicalParams mp;
//        mapping->apply(&mp, mappedObservationData, inputObservationData);
//        this->observationSize = mappedVectorSize;
//    }

//    actualObservation.resize(this->observationSize);
//    noise.clear();
//    noise.resize(this->observationSize);
//    Inherit::bwdInit();
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleObservationManager<FilterType,DataTypes1,DataTypes2>::hasObservation(double _time) {
//    //PRNS("Getting observation at time " << this->actualTime);
      bool hasObservation = observationSource->getObservation(this->actualTime, realObservations);

    if (!hasObservation) {
        PRNE("No observation for time " << _time);
        return(false);
    }


//    actualObservation.setZero();
//    if (!this->doNotMapObservations.getValue()) {
//       sofa::core::MechanicalParams mp;
//       //std::cout << "Input observation: " << inputObsState << std::endl;
//       mapping->apply(&mp, mappedObservationData, inputObservationData);
//       sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObsState = mappedObservationData;
//       //std::cout << "Mapped observation: " << mappedObsState << std::endl;

//        for (size_t i = 0; i < mappedStateSize; i++) {
//            for (size_t d = 0; d < 3; d++) {
//                //mappedObsState[i][d] += noise[3*i+d];
//                actualObservation(3*i+d) = mappedObsState[i][d];
//            }
//        }
//    } else {
//        sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObsState = mappedObservationData;
//        if (mappedObsState.size() != inputObsState.size()) {
//            PRNE("Different mapped and input observation size: " << mappedObsState.size() << " vs " << inputObsState.size());
//            return(false);
//        }

//        for (size_t i = 0; i < mappedObsState.size(); i++)
//            mappedObsState[i] = inputObsState[i];

//        for (size_t i = 0; i < mappedObsState.size(); i++) {
//            for (size_t d = 0; d < 3; d++) {
//                actualObservation(3*i+d) = mappedObsState[i][d];
//            }
//        }
//    }
    return(true);
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleObservationManager<FilterType,DataTypes1,DataTypes2>::getInnovation(double _time, EVectorX& _predictedObservationMean, EVectorX& _innovation)
{
    if (_time != this->actualTime) {
        PRNE("Observation for time " << this->actualTime << " not prepare, call hasObservation first!");
        return(false);
    }

    if(this->stateWrapper->estimatingExternalForces())
        for (size_t ii = 0; ii < 1; ii++) {
            for (size_t jj = 0; jj < 12; jj++)
                _innovation(jj) = realObservations[ii][jj] - _predictedObservationMean(jj);
        }
//    PRNS("Innovation =" << _innovation);
//    PRNS("Real Obs =" << realObservations);
//    PRNS("Predicted Obs =" << _predictedObservationMean);

//        /// ONLY TRUE IF OBSERVATION = POSITION
//        for (size_t ii = 0; ii < 1; ii++) {
//            for (size_t jj = 0; jj < 3; jj++)
//                _innovation(jj) = realObservations[ii][jj] - _predictedObservationMean(jj);
//        }
//        PRNS("real obs =" << realObservations);


    return true;

//    Data<typename DataTypes1::VecCoord> predictedMasterState;
//    Data<typename DataTypes2::VecCoord> predictedMappedState;

//    typename DataTypes1::VecCoord& predictedMasterStateEdit = *predictedMasterState.beginEdit();
//    typename DataTypes2::VecCoord& predictedMappedStateEdit = *predictedMappedState.beginEdit();

//    predictedMasterStateEdit.resize(masterState->getSize());
//    predictedMappedStateEdit.resize(mappedState->getSize());

//    stateWrapper->setSofaVectorFromFilterVector(_state, predictedMasterStateEdit);
//    sofa::core::MechanicalParams mp;
//    mapping->apply(&mp, predictedMappedState, predictedMasterState);

//    _innovation.resize(this->observationSize);
//    for (size_t i = 0; i < predictedMappedStateEdit.size(); i++)
//        for (size_t d = 0; d < 3; d++)
//            _innovation(3*i+d) = actualObservation(3*i+d) - predictedMappedStateEdit[i][d];

//    return(true);


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

template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleObservationManager<FilterType,DataTypes1,DataTypes2>::predictedObservation(double _time, EVectorX& _state, EVectorX& _observationPrec, EVectorX& _observation)
{
    if (_time != this->actualTime) {
        PRNE("Observation for time " << this->actualTime << " not prepare, call hasObservation first!");
        return(false);
    }
    size_t ssize = _state.rows();
    size_t osize = 3;

    double dt = this-> gnode->getDt();
    matH.resize(osize,ssize);
    matH.setZero();
    /// matH needs to be redifined according to State Vector and Observation


    if (this->stateWrapper->estimatingPosition() && this->stateWrapper->estimatingVelocity()) {
        for (size_t i = 3; i < ssize; i++)
            matH(i,i)=0;
        _observation=matH*_state;
    }

    if (this->stateWrapper->estimatingPosition() && !this->stateWrapper->estimatingVelocity()) {
        for (size_t i = (ssize-osize); i < ssize ; i++){
            for (size_t j = 0 ; j < osize ; j++){
                matH(i,j)=1;
                _observation=matH*_state;
            }
        }
    }

    if (!this->stateWrapper->estimatingPosition() && this->stateWrapper->estimatingVelocity()) {
        for (size_t i = 0; i < ssize; i++)
            matH(i,i)=dt;
        _observation=_observationPrec +matH*_state;
    }

    return true;
}



//template <class DataTypes>
//void SimpleObservationManager<DataTypes>::init()
//{
//
//}
//
//template <class DataTypes>
//void SimpleObservationManager<DataTypes>::reinit()
//{
//}

} // stochastic
} // component
} // sofa

#endif // SIMPLEOBSERVATIONMANAGER
