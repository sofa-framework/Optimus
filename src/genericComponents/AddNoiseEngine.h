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
#ifndef ADD_NOISE_ENGINE_H_
#define ADD_NOISE_ENGINE_H_

//#include "initPLUGIN.h"
//#include <sofa/component/component.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
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
class AddNoiseEngine: public sofa::core::DataEngine
{
public:
    
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
    
    void init();

    void reinit() {
        doUpdate();
    }

    void doUpdate();
    double getRandomGauss(const double mean, const double sigma);
    
    void handleEvent(core::objectmodel::Event *event);
}; /// class


} // engine
} // component
} // sofa

#endif 


