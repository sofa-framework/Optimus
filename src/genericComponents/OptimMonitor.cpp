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
#define SOFA_COMPONENT_MISC_OPTIMMONITOR_CPP

#include "OptimMonitor.inl"
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>



namespace sofa
{

namespace component
{

namespace misc
{


SOFA_DECL_CLASS(OptimMonitor)

using namespace sofa::defaulttype;

// Register in the Factory
int OptimMonitorClass = core::RegisterObject("OptimMonitoring of particles")
    .add< OptimMonitor<Vec3Types> >(true)
    .add< OptimMonitor<Vec6Types> >()
    .add< OptimMonitor<Rigid3Types> >()
    ;


template class SOFA_OPTIMUSPLUGIN_API OptimMonitor<Vec3Types>;
template class SOFA_OPTIMUSPLUGIN_API OptimMonitor<Vec6Types>;
template class SOFA_OPTIMUSPLUGIN_API OptimMonitor<Rigid3Types>;



} // namespace misc

} // namespace component

} // namespace sofa

