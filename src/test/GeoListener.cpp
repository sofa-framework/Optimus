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
/******************************************************************************
  *  average von mises stress class registration
 *****************************************************************************/
#define SOFA_COMPONENT_GEO_LISTENER_CPP

/* include files */
#include "GeoListener.inl"
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/TemplatesAliases.h>



namespace sofa
{

namespace component
{

namespace behavior
{



using namespace sofa::defaulttype;

SOFA_DECL_CLASS(GeomagicDeviceListener)

int GeomagicDeviceListenerClass = core::RegisterObject("Geomagic device manipulating object")
        .add< GeoListener<Vec3Types> >(true)
        .add< GeoListener<Vec6Types> >()
        .add< GeoListener<Rigid3Types> >()
        ;


template class GeoListener<Vec3Types>;
template class GeoListener<Vec6Types>;
template class GeoListener<Rigid3Types>;



} // namespace bahavior

} // namespace component

} // namespace sofa

