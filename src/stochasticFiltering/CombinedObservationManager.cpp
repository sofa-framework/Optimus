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
#include "CombinedObservationManager.inl"
//#include <sofa/helper/accessor.h>

namespace sofa
{

namespace component
{

namespace stochastic
{

using namespace defaulttype;


SOFA_DECL_CLASS(CombinedObservationManager)

// Register in the Factory
int CombinedObservationManagerClass = core::RegisterObject("CombinedObservationManager")

//        .add< CombinedObservationManager<double, Vec2dTypes, Vec3dTypes> >()
        .add< CombinedObservationManager<double, Vec3dTypes, Vec3dTypes> >()
        .add< CombinedObservationManager<double, Vec2dTypes, Vec3dTypes> >()
        .add< CombinedObservationManager<double, Vec2dTypes, Rigid3dTypes> >()
        .add< CombinedObservationManager<double, Vec3dTypes, Rigid3dTypes> >()
;

//template class SOFA_STOCHASTIC_API CombinedObservationManager<double, Vec2dTypes, Vec3dTypes>;
template class SOFA_STOCHASTIC_API CombinedObservationManager<double, Vec3dTypes, Vec3dTypes>;
template class SOFA_STOCHASTIC_API CombinedObservationManager<double, Vec2dTypes, Vec3dTypes>;
template class SOFA_STOCHASTIC_API CombinedObservationManager<double, Vec2dTypes, Rigid3dTypes>;
template class SOFA_STOCHASTIC_API CombinedObservationManager<double, Vec3dTypes, Rigid3dTypes>;



} // namespace stochastic
} // namespace component
} // namespace sofa




