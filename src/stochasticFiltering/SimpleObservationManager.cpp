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

//#define MAPPEDSTATEOBSERVATIONMANAGER_CPP

#include "optimusConfig.h"

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

