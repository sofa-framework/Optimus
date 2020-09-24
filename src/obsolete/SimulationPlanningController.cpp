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
