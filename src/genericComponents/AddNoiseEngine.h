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

//#include <sofa/component/component.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/DataEngine.h>

#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Vec.h>

#include <sofa/core/topology/Topology.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/gl/template.h>

#include <random>
#include <chrono>


namespace sofa
{

namespace component
{

namespace engine
{


using namespace defaulttype;

template <class DataTypes>
class AddNoiseEngine : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(AddNoiseEngine, DataTypes), core::DataEngine);

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real Real;
    typedef typename sofa::defaulttype::Vec3d Vector3;
    typedef sofa::core::topology::Topology::Edge Edge;
    
    AddNoiseEngine();
    ~AddNoiseEngine() {}
    
    Data<VecCoord> _inputPositions;
    Data<VecCoord> _outputPositionsWithNoise;
    Data<Coord> _noisePrincipalDirection;
    Data<std::string> _noiseType;
    Data<double>  _noiseMean;
    Data<double>  _noisePrincipalVariance;
    Data<double>  _noiseOrthogonalVariance;

    Coord dir, orth1, orth2;
    
    void init() override;

    void reinit() override {
        doUpdate();
    }

    void doUpdate();
    double getRandomGauss(const double mean, const double sigma);
    
    void handleEvent(core::objectmodel::Event *event) override;
}; /// class



} // engine

} // component

} // sofa

