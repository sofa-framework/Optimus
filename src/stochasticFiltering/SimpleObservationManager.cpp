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

//#define MAPPEDSTATEOBSERVATIONMANAGER_CPP

#include <sofa/core/ObjectFactory.h>
#include "SimpleObservationManager.inl"
//#include <sofa/helper/accessor.h>

namespace sofa
{

namespace component
{

namespace stochastic
{

using namespace defaulttype;


SOFA_DECL_CLASS(SimpleObservationManager)

// Register in the Factory
int SimpleObservationManagerClass = core::RegisterObject("SimpleObservationManager")
    .add< SimpleObservationManager<double, Vec3Types, Vec3Types> >()
    .add< SimpleObservationManager<double, Vec2Types, Vec3Types> >()
    .add< SimpleObservationManager<double, Vec2Types, Rigid3Types> >()
    .add< SimpleObservationManager<double, Vec3Types, Rigid3Types> >()
    ;

template class SOFA_STOCHASTIC_API SimpleObservationManager<double, Vec3Types, Vec3Types>;
template class SOFA_STOCHASTIC_API SimpleObservationManager<double, Vec2Types, Vec3Types>;
template class SOFA_STOCHASTIC_API SimpleObservationManager<double, Vec2Types, Rigid3Types>;
template class SOFA_STOCHASTIC_API SimpleObservationManager<double, Vec3Types, Rigid3Types>;



} // namespace stochastic

} // namespace component

} // namespace sofa

