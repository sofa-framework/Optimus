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
#ifndef MAPPEDSTATEOBSERVATIONMANAGER_H_
#define MAPPEDSTATEOBSERVATIONMANAGER_H_

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include "initOptimusPlugin.h"
#include "ObservationManagerBase.h"
#include "../genericComponents/SimulatedStateObservationSource.h"
#include "StochasticStateWrapper.h"

namespace sofa
{
namespace component
{
namespace stochastic
{

using namespace defaulttype;

template <class FilterType, class DataTypes1, class DataTypes2>
class MappedStateObservationManager: public sofa::component::stochastic::ObservationManager<FilterType>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE3(MappedStateObservationManager, FilterType, DataTypes1, DataTypes2), SOFA_TEMPLATE(ObservationManager, FilterType));

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

    MappedStateObservationManager();
    ~MappedStateObservationManager() {}

protected:
    size_t inputVectorSize, masterVectorSize, mappedVectorSize;     /// real sizes of vectors
    size_t inputStateSize, masterStateSize, mappedStateSize;        /// number of points in each vector

    Mapping* mapping;
    MappedState* mappedState;
    MasterState* masterState;
    ObservationSource *observationSource;
    StateWrapper* stateWrapper;

    double actualObservationTime;
    EVectorX actualObservation;

public:
    void init();
    void bwdInit();

    virtual bool hasObservation(double _time); /// TODO
    virtual bool getInnovation(double _time, EVectorX& _state, EVectorX& _predictedObservation, EVectorX& _innovation);

    Data<typename DataTypes1::VecCoord> inputObservationData;
    Data<typename DataTypes2::VecCoord> mappedObservationData;
    Data<double> noiseStdev;
    Data<int> abberantIndex;
    Data<bool> doNotMapObservations;
    SingleLink<MappedStateObservationManager<FilterType, DataTypes1, DataTypes2>, StateWrapper, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> stateWrapperLink;

    boost::mt19937* pRandGen; // I don't seed it on purpouse (it's not relevant)
    boost::normal_distribution<>* pNormDist;
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >* pVarNorm;
    helper::vector<double> noise;



}; /// class


} // stochastic
} // component
} // sofa

#endif // MAPPEDSTATEOBSERVATIONMANAGER_H


