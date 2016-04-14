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
#ifndef MAPPEDSTATEOBSERVATIONMANAGER_INL
#define MAPPEDSTATEOBSERVATIONMANAGER_INL

#include <sofa/simulation/common/Node.h>

#include "MappedStateObservationManager.h"


namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType, class DataTypes1, class DataTypes2>
MappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::MappedStateObservationManager()
    : Inherit()
    , inputObservationData( initData (&inputObservationData, "observations", "observations read from a file") )
    , mappedObservationData( initData (&mappedObservationData, "mappedObservations", "mapped observations") )
    , noiseStdev( initData(&noiseStdev, double(0.0), "noiseStdev", "standard deviation of generated noise") )
    , abberantIndex( initData(&abberantIndex, int(-1), "abberantIndex", "index of an aberrant point") )
    , doNotMapObservations( initData(&doNotMapObservations, false, "doNotMapObservations", "if real observations are read from a file (not the mechanical object)") )
    , stateWrapperLink(initLink("stateWrapper", "link to the state wrapper needed to perform apply (perhaps to be changed)"), stateWrapper)
{    
}

template <class FilterType, class DataTypes1, class DataTypes2>
void MappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::init()
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

    if (stateWrapper) {
        PRNS("Link to state wrapper: " << stateWrapper->getName());
    } else {
        PRNE("Link to state wrapper not initialized!");
    }

    /*SofaVerdandiFilter<SofaModelWrapper<Real1>, SofaLinearObservationManager<Real1> >* sofaFilter;
    gnode->get(sofaFilter, core::objectmodel::BaseContext::SearchUp);
    if (sofaFilter) {
        sofaModel = sofaFilter->getModel();
    } else
        std::cerr << "[" << this->getName() << "]: ERROR no SOFA filter found " << std::endl;*/

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
void MappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::bwdInit()
{
    typename DataTypes1::VecCoord& inputObservation = *inputObservationData.beginEdit();

    this->observationSize = observationSource->getNParticles()*DataTypes1::spatial_dimensions;

    observationSource->getObservation(0.0, inputObservation);

    if (doNotMapObservations.getValue()) {
       sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObservation = mappedObservationData;
       mappedObservation.resize(inputObservation.size());
       PRNS("Not mapping observations, using directly the input observations provided by the source.");

       for (size_t i = 0; i < mappedObservation.size(); i++)
           mappedObservation[i] = inputObservation[i];
    } else {
       sofa::core::MechanicalParams mp;
       mapping->apply(&mp, mappedObservationData, inputObservationData);
    }

    noise.clear();
    noise.resize(this->observationSize);

    Inherit::bwdInit();
}

template <class FilterType, class DataTypes1, class DataTypes2>
typename MappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::EVectorX &
    MappedStateObservationManager<FilterType,DataTypes1,DataTypes2>::getInnovation(EVectorX& _state)
{
    typename DataTypes1::VecCoord& inputObservation = *inputObservationData.beginEdit();

    bool hasObservation = observationSource->getObservation(this->actualTime, inputObservation);

    sofa::core::MechanicalParams mp;
    EVectorX actualObs(3*mappedObservationData.getValue().size());
    if (!this->doNotMapObservations.getValue()) {
       mapping->apply(&mp, mappedObservationData, inputObservationData);
       sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObservation = mappedObservationData;

        for (size_t i = 0; i < mappedObservation.size(); i++) {
            for (size_t d = 0; d < 3; d++) {
                mappedObservation[i][d] += noise[3*i+d];
                actualObs(3*i+d) = mappedObservation[i][d];
            }
        }
    } else {
        sofa::helper::WriteAccessor< Data<typename DataTypes1::VecCoord> > mappedObservation = mappedObservationData;
        if (mappedObservation.size() != inputObservation.size()) {
            PRNE("Different mapped and input observation size!");
        }

        for (size_t i = 0; i < mappedObservation.size(); i++)
            mappedObservation[i] = inputObservation[i];

        for (size_t i = 0; i < mappedObservation.size(); i++) {
            for (size_t d = 0; d < 3; d++) {
                actualObs(3*i+d) = mappedObservation[i][d];
            }
        }
    }

    Data<typename DataTypes1::VecCoord> masterStateTempData;
    Data<typename DataTypes2::VecCoord> mappedStateTempData;

    typename DataTypes1::VecCoord& masterStateTemp = *masterStateTempData.beginEdit();
    typename DataTypes2::VecCoord& mappedStateTemp = *mappedStateTempData.beginEdit();

    masterStateTemp.resize(masterState->getSize());
    mappedStateTemp.resize(mappedState->getSize());

    //mappedState.resize(mappedStateSize);
    stateWrapper->setSofaVectorFromVerdandiVector(_state, masterStateTemp);
    /*sofaModel->SetSofaVectorFromVerdandiState(actualState, x, sofaObject);

    //std::cout << "AKDEBUG2 GI " << actualState[50] << " " << actualState[100] << std::endl;

    mapping->apply(&mp, mappedStateData, actualStateData);

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

#endif // MAPPEDSTATEOBSERVATIONMANAGER_INL
