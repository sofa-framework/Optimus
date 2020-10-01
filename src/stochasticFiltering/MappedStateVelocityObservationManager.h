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

#include "initOptimusPlugin.h"
#include "ObservationManagerBase.h"
#include "../genericComponents/SimulatedStateObservationSource.h"
#include "StochasticStateWrapper.h"

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>



namespace sofa
{

namespace component
{

namespace stochastic
{


using namespace defaulttype;

template <class FilterType, class DataTypes1, class DataTypes2>
class MappedStateVelocityObservationManager: public sofa::component::stochastic::ObservationManager<FilterType>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE3(MappedStateVelocityObservationManager, FilterType, DataTypes1, DataTypes2), SOFA_TEMPLATE(ObservationManager, FilterType));

    typedef typename sofa::component::stochastic::ObservationManager<FilterType> Inherit;
    //typedef typename Inherit::EVectorX EVectorX;
    //typedef typename Inherit::EMatrixX EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

    typedef typename DataTypes1::Real Real1;
    typedef core::behavior::MechanicalState<DataTypes1> MasterState;
    typedef core::behavior::MechanicalState<DataTypes2> MappedState;
    typedef sofa::core::Mapping<DataTypes1, DataTypes2> Mapping;
    typedef sofa::component::container::SimulatedStateObservationSource<DataTypes1> ObservationSource;
    typedef StochasticStateWrapper<DataTypes1,FilterType> StateWrapper;

    MappedStateVelocityObservationManager();
    ~MappedStateVelocityObservationManager() {}

protected:
    size_t inputVectorSize, masterVectorSize, mappedVectorSize;     /// real sizes of vectors
    size_t inputStateSize, inputLoadedStateSize, masterStateSize, mappedStateSize;        /// number of points in each vector

    Mapping* mapping;
    MappedState* mappedState;
    MasterState* masterState;
    helper::vector<ObservationSource*> observationSources;
    StateWrapper* stateWrapper;

    double actualObservationTime;
    EVectorX actualObservation;


public:
    void init() override;
    void bwdInit() override;
    void initializeObservationData();

    virtual bool hasObservation(double _time) override; /// TODO
    virtual bool getInnovation(double _time, EVectorX& _state, EVectorX& _innovation) override;
    virtual bool getRealObservation(double _time, EVectorX& _realObs) override;
    virtual bool getPredictedObservation(int _id, EVectorX& _predictedObservation) override;
    virtual bool obsFunction(EVectorX& _state, EVectorX& _predictedObservation) override;

    Data<typename DataTypes1::VecCoord> inputObservationData;
    Data<typename DataTypes1::VecDeriv> inputVelocityObservationData;
    Data<typename DataTypes2::VecCoord> mappedObservationFullData;
    Data<typename DataTypes2::VecCoord> mappedObservationData;
    Data<typename DataTypes2::VecDeriv> mappedObservationVelocityData;
    Data<double> noiseStdev;
    Data<int> abberantIndex;
    Data<bool> doNotMapObservations;
    Data<bool> d_observePositions;
    Data<bool> d_observeVelocities;
    Data<FilterType> d_velocityObservationStdev;
    Data<helper::vector<int> > d_observationIndices;
    helper::vector<int> observationIndices;


    SingleLink<MappedStateVelocityObservationManager<FilterType, DataTypes1, DataTypes2>, StateWrapper, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> stateWrapperLink;

    boost::mt19937* pRandGen; // I don't seed it on purpouse (it's not relevant)
    boost::normal_distribution<>* pNormDist;
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >* pVarNorm;
    helper::vector<double> noise;

}; /// class


} // namespace stochastic

} // namespace component

} // namespace sofa

