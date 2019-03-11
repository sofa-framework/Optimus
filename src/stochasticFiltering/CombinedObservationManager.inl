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
#ifndef CombinedObservationManager_INL_
#define CombinedObservationManager_INL_

#include <sofa/simulation/Node.h>

#include "CombinedObservationManager.h"


namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType, class DataTypes1, class DataTypes2>
CombinedObservationManager<FilterType,DataTypes1,DataTypes2>::CombinedObservationManager()
    : Inherit()
    , d_use2dObservations(initData(&d_use2dObservations, false, "use2dObservation", "Set to True if using 2D observations"))
    , d_projectionMatrix( initData(&d_projectionMatrix, Mat3x4d(defaulttype::Vec<4,float>(1.0,0.0,0.0,0.0),
                                                                defaulttype::Vec<4,float>(0.0,1.0,0.0,0.0),
                                                                defaulttype::Vec<4,float>(0.0,0.0,1.0,0.0)), "projectionMatrix","Projection matrix"))
    , stateWrapperLink(initLink("stateWrapper", "link to the state wrapper needed to perform apply (perhaps to be changed)"))
    , observationWrapperLink1(initLink("obsSource1", "link to the observation wrapper needed to perform apply (perhaps to be changed)"))
    , observationWrapperLink2(initLink("obsSource2", "link to the observation wrapper needed to perform apply (perhaps to be changed)"))

{
}







template <class FilterType, class DataTypes1, class DataTypes2>
void CombinedObservationManager<FilterType,DataTypes1,DataTypes2>::init()
{
    Inherit::init();

    observationSource1 = observationWrapperLink1.get();
    if (observationSource1) {
        std::cout<<"[CombinedObservationManager] Link to observation source1: " << observationSource1->getName()<<std::endl;
    } else {
        std::cout<<"[CombinedObservationManager] Link to observation source1 not initialized!"<<std::endl;
        return;
    }

    observationSource2 = observationWrapperLink2.get();
    if (observationSource2) {
        std::cout<<"[CombinedObservationManager] Link to observation source2: " << observationSource2->getName()<<std::endl;
    } else {
        std::cout<<"[CombinedObservationManager] Link to observation source2 not initialized!"<<std::endl;
        return;
    }


    stateWrapper = stateWrapperLink.get();
    if (stateWrapper) {
        std::cout<<"[CombinedObservationManager] Link to state wrapper: " << stateWrapper->getName()<<std::endl;
    } else {
        std::cout<<"[CombinedObservationManager] Link to state wrapper not initialized!"<<std::endl;
        return;
    }

    this->gnode->get(masterState);
    if (masterState != NULL) {
        PRNS("Found master mechanical state: " << masterState->getName());
    }
    else {
        PRNE("No master mechanical state found!");
    }

}

template <class FilterType, class DataTypes1, class DataTypes2>
void CombinedObservationManager<FilterType,DataTypes1,DataTypes2>::bwdInit()
{
    //    size_t obsSize1 = observationSource1->getStateSize() * DataTypes1::spatial_dimensions;
    //    size_t obsSize2 = observationSource2->getStateSize() * DataTypes1::spatial_dimensions;

    //    this->observationSize =obsSize1+obsSize2;

    Inherit::bwdInit();

}

template <class FilterType, class DataTypes1, class DataTypes2>
bool CombinedObservationManager<FilterType,DataTypes1,DataTypes2>::hasObservation(double _time) {
    if(this->actualTime==0){
        hasObservation1=true;
        hasObservation2=true;

    } else{
        hasObservation1= observationSource1->getObservation(this->actualTime, realObservations1);
        hasObservation2= observationSource2->getObservation(this->actualTime, realObservations2);

        obsSize1 = observationSource1->getStateSize() * DataTypes1::spatial_dimensions;
        obsSize2 = observationSource2->getStateSize() * DataTypes1::spatial_dimensions;

    }

    if (!hasObservation1 && hasObservation2) {
        this->observationSize =obsSize2;
    }
    if (hasObservation1 && !hasObservation2) {
        this->observationSize =obsSize1;
    }
    if (hasObservation1 && hasObservation2) {
        this->observationSize =obsSize1+obsSize2;
    }

    if (!hasObservation1 && !hasObservation2) {
        PRNE("No observation for time " << _time);
        return(false);
    }

    return(true);
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool CombinedObservationManager<FilterType,DataTypes1,DataTypes2>::getInnovation(double _time, EVectorX& _state, EVectorX& _innovation)
{

    if (_time != this->actualTime) {
        PRNE("Observation for time " << this->actualTime << " not prepared, call hasObservation first!");
        return(false);
    }

    if (_innovation.rows() != long(this->observationSize)) {
        PRNE("Wrong innovation size: " << _innovation.rows() << " should be " << this->observationSize);
        return(false);
    }

    if ((stateWrapper->getFilterKind() == SIMCORR) || (stateWrapper->getFilterKind() == CLASSIC)) {
        if (hasObservation1 && !hasObservation2) {
            for (size_t i = 0; i < this->observationSize; i++)
                _innovation(i) = realObservations1[0](i) - _state(i);
        }
        if (!hasObservation1 && hasObservation2) {
            for (size_t i = 0; i < this->observationSize; i++)
                _innovation(i) = realObservations2[0](i) - _state(i);
        }
        if (hasObservation1 && hasObservation2) {
            EVectorX temp;
            temp.resize(this->observationSize);
            for (size_t d = 0; d < obsSize1; d++){
                 for (size_t i = 0; i < 3; i++)
                     temp(d*6 + i) = realObservations1[0](d*3+i);
                 for (size_t i = 0; i < 3; i++)
                     temp(d*6 + i + 3) = realObservations2[0](d*3+i);

            }
            for (size_t i = 0; i < this->observationSize; i++)
                _innovation(i) =temp(i) - _state(i);

        }



    }

    return true;


}

template <class FilterType, class DataTypes1, class DataTypes2>
bool CombinedObservationManager<FilterType,DataTypes1,DataTypes2>::getRealObservation(double _time, EVectorX& _realObs)
{

    //    if (_time != this->actualTime) {
    //        PRNE("Observation for time " << this->actualTime << " not prepared, call hasObservation first!");
    //        return(false);
    //    }

    //    if (_realObs.rows() != long(this->observationSize)) {
    //        PRNE("Wrong innovation size: " << _realObs.rows() << " should be " << this->observationSize);
    //        return(false);
    //    }

    //    if ((stateWrapper->getFilterKind() == SIMCORR) || (stateWrapper->getFilterKind() == CLASSIC)) {
    //        for (size_t i = 0; i < this->observationSize; i++)
    //            _realObs(i) = realObservations[0](i);
    //    }

    //    return true;


}
template <class FilterType, class DataTypes1, class DataTypes2>
bool CombinedObservationManager<FilterType,DataTypes1,DataTypes2>::obsFunction(EVectorX& _state, EVectorX& _predictedObservation)
{
    //    const Mat3x4d & P = d_projectionMatrix.getValue();
    //    _predictedObservation.resize(this->observationSize);

    //    Data<typename DataTypes1::VecCoord> predicted2DState;
    //    Data<typename DataTypes2::VecCoord> predicted3DState;


    //    typename DataTypes1::VecCoord& predicted2DStateEdit = *predicted2DState.beginEdit();
    //    typename DataTypes2::VecCoord& predicted3DStateEdit = *predicted3DState.beginEdit();

    //    predicted3DStateEdit.resize(masterState->getSize());

    //    stateWrapper->getPos(_state, predicted3DStateEdit);
    //    predicted2DStateEdit.resize(predicted3DStateEdit.size());

    //    if(d_use2dObservations.getValue()){
    //        for (unsigned i = 0; i < predicted3DStateEdit.size(); i++){
    //            double rx = P[0][0] * predicted3DStateEdit[i][0] + P[0][1] * predicted3DStateEdit[i][1] + P[0][2] * predicted3DStateEdit[i][2] + P[0][3];
    //            double ry = P[1][0] * predicted3DStateEdit[i][0] + P[1][1] * predicted3DStateEdit[i][1] + P[1][2] * predicted3DStateEdit[i][2] + P[1][3];
    //            double rz = P[2][0] * predicted3DStateEdit[i][0] + P[2][1] * predicted3DStateEdit[i][1] + P[2][2] * predicted3DStateEdit[i][2] + P[2][3];
    //            predicted2DStateEdit[i][0]=rx* (1.0/rz);
    //            predicted2DStateEdit[i][1]=ry* (1.0/rz);
    //        }
    //        for (size_t i = 0; i < predicted3DStateEdit.size(); i++){
    //            for (size_t d = 0; d < 2; d++){
    //                _predictedObservation(2*i+d) = predicted2DStateEdit[i][d];
    //            }
    //        }

    //    }else{
    //        for (size_t i = 0; i < predicted3DStateEdit.size(); i++)
    //            for (size_t d = 0; d < 3; d++)
    //                _predictedObservation(3*i+d) = predicted3DStateEdit[i][d];

    //    }
    //    return true;


}


template <class FilterType, class DataTypes1, class DataTypes2>
bool CombinedObservationManager<FilterType,DataTypes1,DataTypes2>::getPredictedObservation(int _id, EVectorX& _predictedObservation)
{
    const Mat3x4d & P = d_projectionMatrix.getValue();
    _predictedObservation.resize(this->observationSize);

    Data<typename DataTypes1::VecCoord> predicted2DState;
    Data<typename DataTypes2::VecCoord> predicted3DState;


    typename DataTypes1::VecCoord& predicted2DStateEdit = *predicted2DState.beginEdit();
    typename DataTypes2::VecCoord& predicted3DStateEdit = *predicted3DState.beginEdit();

    predicted3DStateEdit.resize(masterState->getSize());

    stateWrapper->getActualPosition(_id, predicted3DStateEdit);
    predicted2DStateEdit.resize(predicted3DStateEdit.size());

    if(d_use2dObservations.getValue()){
        for (unsigned i = 0; i < predicted3DStateEdit.size(); i++){
            double rx = P[0][0] * predicted3DStateEdit[i][0] + P[0][1] * predicted3DStateEdit[i][1] + P[0][2] * predicted3DStateEdit[i][2] + P[0][3];
            double ry = P[1][0] * predicted3DStateEdit[i][0] + P[1][1] * predicted3DStateEdit[i][1] + P[1][2] * predicted3DStateEdit[i][2] + P[1][3];
            double rz = P[2][0] * predicted3DStateEdit[i][0] + P[2][1] * predicted3DStateEdit[i][1] + P[2][2] * predicted3DStateEdit[i][2] + P[2][3];
            predicted2DStateEdit[i][0]=rx* (1.0/rz);
            predicted2DStateEdit[i][1]=ry* (1.0/rz);
        }
        for (size_t i = 0; i < predicted3DStateEdit.size(); i++){
            for (size_t d = 0; d < 2; d++){
                _predictedObservation(2*i+d) = predicted2DStateEdit[i][d];
            }
        }
    }else{
        for (size_t i = 0; i < predicted3DStateEdit.size(); i++){
            if (hasObservation1 && !hasObservation2) {
                for (size_t d = 0; d < 3; d++)
                    _predictedObservation(3*i+d) = predicted3DStateEdit[i][d];
            }
            if (!hasObservation1 && hasObservation2) {
                for (size_t d = 0; d < 3; d++)
                    _predictedObservation(3*i+d) = predicted3DStateEdit[i][3+d];
            }
            if (hasObservation1 && hasObservation2) {
                for (size_t d = 0; d < 6; d++)
                    _predictedObservation(6*i+d) = predicted3DStateEdit[i][d];
            }
        }
    }
    return true;


}



} // stochastic
} // component
} // sofa

#endif // CombinedObservationManager
