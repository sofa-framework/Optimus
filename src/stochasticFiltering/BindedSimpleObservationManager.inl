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
#ifndef BindedSimpleObservationManager_INL_
#define BindedSimpleObservationManager_INL_

#include <sofa/simulation/Node.h>

#include "BindedSimpleObservationManager.h"


namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType, class DataTypes1, class DataTypes2>
BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::BindedSimpleObservationManager()
    : Inherit()
    , d_use2dObservations(initData(&d_use2dObservations, false, "use2dObservation", "Set to True if using 2D observations"))
    , d_projectionMatrix( initData(&d_projectionMatrix, Mat3x4d(defaulttype::Vec<4,float>(1.0,0.0,0.0,0.0),
                                                                defaulttype::Vec<4,float>(0.0,1.0,0.0,0.0),
                                                                defaulttype::Vec<4,float>(0.0,0.0,1.0,0.0)), "projectionMatrix","Projection matrix"))
    , d_proj_dist(initData(&d_proj_dist, (double) 0.0, "projDist", "Projection Distance"))
    , stateWrapperLink(initLink("stateWrapper", "link to the state wrapper needed to perform apply (perhaps to be changed)"))
    , d_mappedStatePath(initData(&d_mappedStatePath, "mappedState", "Link to Virtual Mapped Catheter "))

{
}

template <class FilterType, class DataTypes1, class DataTypes2>
void BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::init()
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

    this->gnode->get(masterState);
    if (masterState != NULL) {
        PRNS("Found master mechanical state: " << masterState->getName());
    }
    else {
        PRNE("No master mechanical state found!");
    }

    this->getContext()->get(mappedState, d_mappedStatePath.getValue());
    if (mappedState == NULL) serr << "Error: Cannot find the Mapped State Component" <<sendl;

}

template <class FilterType, class DataTypes1, class DataTypes2>
void BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::bwdInit()
{
    this->observationSize = observationSource->getStateSize() * DataTypes1::spatial_dimensions;
    Inherit::bwdInit();

}

template <class FilterType, class DataTypes1, class DataTypes2>
bool BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::hasObservation(double _time) {

    bool hasObservation = observationSource->getObservation(this->actualTime, realObservations);

    if (!hasObservation) {
        PRNE("No observation for time " << _time);
        return(false);
    }

    return(true);
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::getPredictedObservation(double _time, int _id, EVectorX& _predictedObservation)
{
    const Mat3x4d & P = d_projectionMatrix.getValue();

    _predictedObservation.resize(this->observationSize);

    Data<typename DataTypes1::VecCoord> real2DObservations;
    Data<typename DataTypes1::VecCoord> allPredicted2DState;
    Data<typename DataTypes1::VecCoord> bindedPredicted2DState;
    Data<typename DataTypes2::VecCoord> allPredicted3DState;

    typename DataTypes1::VecCoord& allPredicted2DStateEdit = *allPredicted2DState.beginEdit();
    typename DataTypes2::VecCoord& allPredicted3DStateEdit = *allPredicted3DState.beginEdit();
    typename DataTypes1::VecCoord& real2DObservationsEdit = *real2DObservations.beginEdit();
    typename DataTypes1::VecCoord& bindedPredicted2DStateEdit = *bindedPredicted2DState.beginEdit();


    allPredicted3DStateEdit.resize(mappedState->getSize());
    real2DObservationsEdit.resize((this->observationSize)*0.5);
    bindedPredicted2DStateEdit.resize((this->observationSize)*0.5);

    stateWrapper->getActualMappedPosition(_id, allPredicted3DStateEdit);
    allPredicted2DStateEdit.resize(allPredicted3DStateEdit.size());

    for (unsigned i = 0; i < allPredicted3DStateEdit.size(); i++){
        double rx = P[0][0] * allPredicted3DStateEdit[i][0] + P[0][1] * allPredicted3DStateEdit[i][1] + P[0][2] * allPredicted3DStateEdit[i][2] + P[0][3];
        double ry = P[1][0] * allPredicted3DStateEdit[i][0] + P[1][1] * allPredicted3DStateEdit[i][1] + P[1][2] * allPredicted3DStateEdit[i][2] + P[1][3];
        double rz = P[2][0] * allPredicted3DStateEdit[i][0] + P[2][1] * allPredicted3DStateEdit[i][1] + P[2][2] * allPredicted3DStateEdit[i][2] + P[2][3];
        allPredicted2DStateEdit[i][0]=rx* (1.0/rz);
        allPredicted2DStateEdit[i][1]=ry* (1.0/rz);
    }


    for (size_t i = 0; i < real2DObservationsEdit.size(); i++){
        for (size_t d = 0; d < 2; d++){
            real2DObservationsEdit[i][d] = realObservations[0](2*i+d);
        }
    }
//    PRNS("real2DObservationsEdit "  << real2DObservationsEdit);
    PRNS("allPredicted2DStateEdit "  << allPredicted2DStateEdit);

    bindId.clear();
    for (unsigned t=0;t < real2DObservationsEdit.size();t++) {
        Vector2 real(real2DObservationsEdit[t][0],real2DObservationsEdit[t][1]);

        int bind = -1;
        double minDist =  0;

        for (unsigned i=0;i < allPredicted2DStateEdit.size();i++) {
             Vector2 proj (allPredicted2DStateEdit[i][0],allPredicted2DStateEdit[i][1]);

            double dist=(real-proj).norm();

            if (dist< d_proj_dist.getValue() && ((bind == -1) || (dist< minDist))) {
                minDist = dist;
                bind = i;
            }
        }
        bindId.push_back(bind);
    }

    PRNS("BindId  " << bindId)

//    for (size_t i = 0; i < real2DObservationsEdit.size(); i++){
//        for (size_t d = 0; d < 2; d++)
//            bindedPredicted2DStateEdit[i][d]= allPredicted2DStateEdit[bindId[i]][d];
//    }

    for (size_t i = 0; i < real2DObservationsEdit.size(); i++){
        for (size_t d = 0; d < 2; d++){
            _predictedObservation(2*i+d) = allPredicted2DStateEdit[bindId[i]][d];
        PRNS("_predictedObservation " << allPredicted2DStateEdit[i][d]  );}
    }


    return true;

}
template <class FilterType, class DataTypes1, class DataTypes2>
bool BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::getInnovation(double _time, EVectorX& _state, EVectorX& _innovation)
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
        for (size_t i = 0; i < this->observationSize; i++)
            _innovation(i) = realObservations[0](i) - _state(i);
    }
//    PRNS("realObservations  " << realObservations);
//    PRNS("_predictedObs     " << _state.transpose());

    return true;


}




} // stochastic
} // component
} // sofa

#endif // BindedSimpleObservationManager
