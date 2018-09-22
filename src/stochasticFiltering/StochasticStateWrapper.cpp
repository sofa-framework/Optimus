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

//#define SOFASTATEWRAPPER_CPP

#include <sofa/core/ObjectFactory.h>
#include "StochasticStateWrapper.inl"
//#include <sofa/helper/accessor.h>

namespace sofa
{

namespace component
{

namespace stochastic
{

using namespace defaulttype;

SOFA_DECL_CLASS(StochasticStateWrapper)

// Register in the Factory
int StochasticStateWrapperClass = core::RegisterObject("StochasticStateWrapper")
        #ifndef SOFA_FLOAT
        .add< StochasticStateWrapper<Vec3dTypes, double> >(true)
        .add< StochasticStateWrapper<Rigid3dTypes, double> >()

        #endif
        #ifndef SOFA_DOUBLE
//        .add< StochasticStateWrapper<Vec3fTypes, float> >()
//        .add< StochasticStateWrapper<Rigid3fTypes, float> >()

        #endif
        ;

#ifndef SOFA_FLOAT
template class SOFA_SIMULATION_COMMON_API StochasticStateWrapper<Vec3dTypes, double>;
template class SOFA_SIMULATION_COMMON_API StochasticStateWrapper<Rigid3dTypes, double>;

#endif
#ifndef SOFA_DOUBLE
//template class SOFA_SIMULATION_COMMON_API StochasticStateWrapper<Vec3fTypes, float>;
//template class SOFA_SIMULATION_COMMON_API StochasticStateWrapper<Rigid3dTypes, float>;

#endif


} // namespace simulation
} // namespace component
} // namespace sofa




