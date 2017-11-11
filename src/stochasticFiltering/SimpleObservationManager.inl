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
    this->observationSize = observationSource->getStateSize() * DataTypes1::spatial_dimensions;
    obsSize= observationSource->getStateSize() * DataTypes1::spatial_dimensions;
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleObservationManager<FilterType,DataTypes1,DataTypes2>::hasObservation(double _time) {
//    //PRNS("Getting observation at time " << this->actualTime);
      bool hasObservation = observationSource->getObservation(this->actualTime, realObservations);

    if (!hasObservation) {
        PRNE("No observation for time " << _time);
        return(false);
    }

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
            for (size_t jj = 0; jj < obsSize; jj++)
                _innovation(jj) = realObservations[ii][jj] - _predictedObservationMean(jj);
        }


    return true;


}

template <class FilterType, class DataTypes1, class DataTypes2>
bool SimpleObservationManager<FilterType,DataTypes1,DataTypes2>::predictedObservation(double _time, EVectorX& _state, EVectorX& _observationPrec, EVectorX& _observation)
{

    ///****TO REVIEW ONLY USED FOR STATE ESTIMATION (POSITION VELOCITY)*****///

    if (_time != this->actualTime) {
        PRNE("Observation for time " << this->actualTime << " not prepare, call hasObservation first!");
        return(false);
    }
//    size_t ssize = _state.rows();
//    size_t osize = 3;

//    double dt = this-> gnode->getDt();
//    matH.resize(osize,ssize);
//    matH.setZero();

//    if (this->stateWrapper->estimatingPosition() && this->stateWrapper->estimatingVelocity()) {
//        for (size_t i = 3; i < ssize; i++)
//            matH(i,i)=0;
//        _observation=matH*_state;
//    }

//    if (this->stateWrapper->estimatingPosition() && !this->stateWrapper->estimatingVelocity()) {
//        for (size_t i = (ssize-osize); i < ssize ; i++){
//            for (size_t j = 0 ; j < osize ; j++){
//                matH(i,j)=1;
//                _observation=matH*_state;
//            }
//        }
//    }

//    if (!this->stateWrapper->estimatingPosition() && this->stateWrapper->estimatingVelocity()) {
//        for (size_t i = 0; i < ssize; i++)
//            matH(i,i)=dt;
//        _observation=_observationPrec +matH*_state;
//    }

    return true;
}



} // stochastic
} // component
} // sofa

#endif // SIMPLEOBSERVATIONMANAGER
