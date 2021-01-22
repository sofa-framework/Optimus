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
#include "SimulatedStateObservationSource.inl"
#include <sofa/core/ObjectFactory.h>

#include <sofa/defaulttype/VecTypes.h>
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
    .add< SimulatedStateObservationSource<Vec2Types> >()
    .add< SimulatedStateObservationSource<Vec3Types> >(true)
    .add< SimulatedStateObservationSource<Rigid3Types> >()
    ;


template class SOFA_OPTIMUSPLUGIN_API SimulatedStateObservationSource<Vec2Types>;
template class SOFA_OPTIMUSPLUGIN_API SimulatedStateObservationSource<Vec3Types>;
template class SOFA_OPTIMUSPLUGIN_API SimulatedStateObservationSource<Rigid3Types>;



} // namespace container

} // namespace component

} // namespace sofa
