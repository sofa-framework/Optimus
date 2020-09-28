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
    , d_bindId(initData(&d_bindId, "bindId", "Vector of 2D 3D correspondences"))
    , stateWrapperLink(initLink("stateWrapper", "link to the state wrapper needed to perform apply (perhaps to be changed)"))
    , d_mappedStatePath(initData(&d_mappedStatePath, "mappedState", "Link to Virtual Mapped Catheter "))

{
}

template <class FilterType, class DataTypes1, class DataTypes2>
void BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::init()
{

    Inherit::init();
    bindId=d_bindId.getValue();
    this->gnode->get(observationSource);
    if (observationSource) {
        PRNS("Found observation source: " << observationSource->getName());
    } else {
        PRNE("No observation source found!");
    }

    stateWrapper = stateWrapperLink.get();

    if (stateWrapper) {
        std::cout<< "[BindedSimpleObservationManager] Link to state wrapper: " << stateWrapper->getName()<< std::endl;
    } else {
        std::cout<< "[BindedSimpleObservationManager] Link to state wrapper not initialized!"<< std::endl;

    }
    if(stateWrapper->declaredMapState()==0){
        return;
        serr<<"No mapped state declared in the StochasticStateWrapper  "<<sendl;
    }

    this->gnode->get(masterState);

    if (masterState != NULL) {
        std::cout <<"Found master mechanical state: " << masterState->getName()<< std::endl;
    }
    else {
        std::cout <<"No master mechanical state found!"<< std::endl;
    }


    this->gnode->get(mappedState, d_mappedStatePath.getValue());

    if ( mappedState != NULL)  {
        std::cout<<"[BindedSimpleObservationManager]  Found mapped mechanical state: " << mappedState->getName() << " size:  "<< mappedState->getSize() <<std::endl;
    }
    else {
        std::cout<<"[BindedSimpleObservationManager]  No mapped state state found"<<std::endl;
        return;

    }



}

template <class FilterType, class DataTypes1, class DataTypes2>
void BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::bwdInit()
{
    this->observationSize = observationSource->getStateSize() * DataTypes1::spatial_dimensions;
    Inherit::bwdInit();


}

template <class FilterType, class DataTypes1, class DataTypes2>
bool BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::hasObservation(double _time) {
    bool hasObservation;
    if(this->actualTime==0){
        hasObservation=true;
    } else{

        hasObservation= observationSource->getObservation(this->actualTime, realObservations);
    }
    if (!hasObservation) {
        PRNE("No observation for time " << _time);
        return(false);
    }

    return(true);
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


    return true;


}
template <class FilterType, class DataTypes1, class DataTypes2>
bool BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::obsFunction(EVectorX& /* _state */, EVectorX& /* _predictedObservation */)
{
    return 0;
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::getRealObservation(double /* _time */, EVectorX& /* _realObs */)
{
    return 0;
}

template <class FilterType, class DataTypes1, class DataTypes2>
bool BindedSimpleObservationManager<FilterType,DataTypes1,DataTypes2>::getPredictedObservation(int _id, EVectorX& _predictedObservation)
{

    const Mat3x4d & P = d_projectionMatrix.getValue();
    _predictedObservation.resize(this->observationSize);

    Data<typename DataTypes1::VecCoord> allPred2DObs;
    Data<typename Vec3dTypes::VecCoord> mappPred3DState;


    typename Vec3dTypes::VecCoord& mappPred3DStateEdit = *mappPred3DState.beginEdit();
    mappPred3DStateEdit.resize(mappedState->getSize());

    stateWrapper->getActualMappedPosition(_id, mappPred3DStateEdit);

    typename DataTypes1::VecCoord& allPred2DObsEdit = *allPred2DObs.beginEdit();
    allPred2DObsEdit.resize(mappedState->getSize());

    for (unsigned i = 0; i < mappPred3DStateEdit.size(); i++){
        double rx = P[0][0] * mappPred3DStateEdit[i][0] + P[0][1] * mappPred3DStateEdit[i][1] + P[0][2] * mappPred3DStateEdit[i][2] + P[0][3];
        double ry = P[1][0] * mappPred3DStateEdit[i][0] + P[1][1] * mappPred3DStateEdit[i][1] + P[1][2] * mappPred3DStateEdit[i][2] + P[1][3];
        double rz = P[2][0] * mappPred3DStateEdit[i][0] + P[2][1] * mappPred3DStateEdit[i][1] + P[2][2] * mappPred3DStateEdit[i][2] + P[2][3];
        allPred2DObsEdit[i][0]=rx* (1.0/rz);
        allPred2DObsEdit[i][1]=ry* (1.0/rz);
    }
//    std::cout << "_predictedObservation " << allPred2DObsEdit << std::endl;

//    double dist;
//    Vector2 trans;
//    for (unsigned t=0;t < real2DObsEdit.size();t++) {
//        Vector2 real(real2DObsEdit[t][0],real2DObsEdit[t][1]);

//        Vector2 proj0 (allPred2DObsEdit[0][0],allPred2DObsEdit[0][1]);
//        Vector2 real0(real2DObsEdit[0][0],real2DObsEdit[0][1]);
//        trans=(real0-proj0);


//        int bind = -1;
//        double minDist =  0;

//        for (unsigned i=0;i < allPred2DObsEdit.size();i++) {
//            Vector2 proj ((allPred2DObsEdit[i][0]+trans[0]),(allPred2DObsEdit[i][1]+trans[1]));
//            dist=(real-proj).norm();

//            if (dist< d_proj_dist.getValue() && ((bind == -1) || (dist< minDist))) {
//                minDist = dist;
//                bind = i;
//            }
//        }
//        bindId.push_back(bind);
//    }

    for (size_t i = 0; i <(this->observationSize)*0.5; i++){
        for (size_t d = 0; d < 2; d++){
            _predictedObservation(2*i+d) = allPred2DObsEdit[bindId[i]][d];

        }
    }

    return true;

}


} // stochastic

} // component

} // sofa

