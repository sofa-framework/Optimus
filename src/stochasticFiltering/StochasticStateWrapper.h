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
#ifndef SOFASTATEWRAPPERBASE_H_
#define SOFASTATEWRAPPERBASE_H_

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>

#include <SofaBoundaryCondition/FixedConstraint.h>

#include <Eigen/Dense>

#include "initOptimusPlugin.h"
#include "StochasticStateWrapperBase.h"
#include "../OptimParams.h"

namespace sofa
{
namespace component
{
namespace stochastic
{

using namespace defaulttype;

template <class DataTypes, class FilterType>
class StochasticStateWrapper: public sofa::component::stochastic::StochasticStateWrapperBase
{
public:
    typedef sofa::component::stochastic::StochasticStateWrapperBase Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Coord Deriv;

    enum { Dim = Coord::spatial_dimensions };


    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef typename component::projectiveconstraintset::FixedConstraint<DataTypes> FixedConstraint;
    typedef sofa::component::container::OptimParamsBase OptimParamsBase;

    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

    StochasticStateWrapper();
    ~StochasticStateWrapper();

protected:    
    MechanicalState *mechanicalState;
    FixedConstraint* fixedConstraint;
    helper::vector<OptimParamsBase*> vecOptimParams;

    bool valid;
    helper::vector<std::pair<size_t, size_t> > positionPairs;
    helper::vector<std::pair<size_t, size_t> > velocityPairs;

    int reducedStateIndex;
    int stateSize, reducedStateSize;

    void copyStateVerdandi2Sofa();
    void copyStateSofa2Verdandi();

    EVectorX state;

public:    
    Data<bool> velocityInState;

    void applyOperator();
    void init();
    void bwdInit();
}; /// class


} // simulation
} // component
} // sofa

#endif // SOFASTATEWRAPPERBASE_H


