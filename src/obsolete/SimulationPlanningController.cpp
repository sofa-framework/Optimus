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
//
// C++ Implementation: ArticulatedHierarchyBVHController
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2008
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "SimulationPlanningController.inl"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace component
{

namespace controller
{

SOFA_DECL_CLASS(SimulationPlanningController)

// Register in the Factory
int SimulationPlanningControllerClass = core::RegisterObject("Controller receving control data from an external program or file to drive the needle insertion.")
#ifndef SOFA_FLOAT
.add< SimulationPlanningController<defaulttype::Rigid3dTypes> >(true) // default template
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
.add< SimulationPlanningController<defaulttype::Rigid3fTypes> >()
#endif //SOFA_DOUBLE
;

#ifndef SOFA_FLOAT
template class SimulationPlanningController<defaulttype::Rigid3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
template class SimulationPlanningController<defaulttype::Rigid3fTypes>;
#endif //SOFA_DOUBLE


} // namespace controller

} // namespace component

} // namespace sofa
