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
#include "SimulatedStateObservationSource.inl"
#include <sofa/core/ObjectFactory.h>

#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>


using namespace sofa::defaulttype;

namespace sofa
{

namespace component
{

namespace container
{

SOFA_DECL_CLASS(SimulatedStateObservationSource)

int SimulatedStateObservationSourceClass = core::RegisterObject("Parameters that will be adapted by ROUKF object")
//#ifndef SOFA_FLOAT
        .add< SimulatedStateObservationSource<Vec2dTypes> >()
        .add< SimulatedStateObservationSource<Vec3dTypes> >(true)
        .add< SimulatedStateObservationSource<Rigid3dTypes> >()

//#endif
//#ifndef SOFA_DOUBLE
//        .add< SimulatedStateObservationSource<Vec3fTypes> >()
//#endif
        ;

//#ifndef SOFA_FLOAT
template class SimulatedStateObservationSource<Vec2dTypes>;
template class SimulatedStateObservationSource<Vec3dTypes>;
template class SimulatedStateObservationSource<Rigid3dTypes>;

//#endif
//#ifndef SOFA_DOUBLE
//template class SimulatedStateObservationSource<Vec3fTypes>;
//#endif



} // namespace container

} // namespace component

} // namespace sofa
