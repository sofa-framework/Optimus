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
#ifndef SOFASTATEOBSERVATIONWRAPPER_H_
#define SOFASTATEOBSERVATIONWRAPPER_H_

#include "StochasticStateWrapper.h"

namespace sofa
{
namespace component
{
namespace stochastic
{

using namespace defaulttype;

template <class DataTypes, class FilterType>
class StochasticStateObservationWrapper: public StochasticStateWrapper<DataTypes, FilterType>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(StochasticStateObservationWrapper, DataTypes, FilterType), SOFA_TEMPLATE(StochasticStateWrapperBaseT, FilterType));

    typedef sofa::component::stochastic::StochasticStateWrapper<DataTypes, FilterType> Inherit;

    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

    StochasticStateObservationWrapper();
    ~StochasticStateObservationWrapper();

protected:
    EVectorX obsState;

    helper::vector<std::pair<size_t, size_t> > observationPosPairs;

    void copyStateFilter2Sofa(const core::MechanicalParams *_mechParams);  // copy actual DA state to SOFA state and propagate to mappings
    void copyStateSofa2Filter();  // copy the actual SOFA state to DA state
    void computeSofaStep(const core::ExecParams* execParams, bool _updateTime);
    void computeSofaStepWithLM(const core::ExecParams* params, bool _updateTime);

public:
    bool estimatingPosition() {
        return this->estimatePosition.getValue();
    }

    bool estimatingVelocity() {
        return this->estimateVelocity.getValue();
    }

    bool estimatingExternalForces() {
        return this->estimateExternalForces.getValue();
    }


    void init();
    void bwdInit();

    void applyOperator(EVectorX& _vecX, const core::MechanicalParams* _mparams, bool _preserveState, bool _updateForce);
    //void setSofaTime(const core::ExecParams* _execParams);

    void setState(EVectorX& _state, const core::MechanicalParams* _mparams) {
        this->state = _state;
        copyStateFilter2Sofa(_mparams);
    }

    void setSofaVectorFromFilterVector(EVectorX& _state, typename DataTypes::VecCoord& _vec);
    void setSofaVectorFromObservationsStateVector(EVectorX& _observationsState, typename DataTypes::VecCoord& _vec);

    virtual EVectorX& getStateForObservations() {
        return obsState;
    }

}; /// class


} // simulation
} // component
} // sofa

#endif // SOFASTATEOBSERVATIONWRAPPER_H


