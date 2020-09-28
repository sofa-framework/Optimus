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
#define SOFA_COMPONENT_FORCEFIELD_BUBBLEPACKINGFORCEFIELD_CPP

#include "BubblePackingForceField.inl"
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <fstream> // for reading the file
#include <iostream> //for debugging
#include <vector>
#include <sofa/defaulttype/Vec3Types.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;





SOFA_DECL_CLASS(BubblePackingForceField)

using namespace sofa::defaulttype;


// Register in the Factory
int BubblePackingForceFieldClass = core::RegisterObject("Mitchell-Schaeffer Model (Reaction Part)")
#ifndef SOFA_FLOAT
  .add< BubblePackingForceField<Vec1dTypes> >()
  .add< BubblePackingForceField<Vec2dTypes> >()
  .add< BubblePackingForceField<Vec3dTypes> >(true)
#endif
#ifndef SOFA_DOUBLE
  .add< BubblePackingForceField<Vec1fTypes> >()
  .add< BubblePackingForceField<Vec2fTypes> >()
  .add< BubblePackingForceField<Vec3fTypes> >()
#endif
;

#ifndef SOFA_FLOAT
  template class BubblePackingForceField<Vec1dTypes>;
  template class BubblePackingForceField<Vec2dTypes>;
  template class BubblePackingForceField<Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
  template class BubblePackingForceField<Vec1fTypes>;
  template class BubblePackingForceField<Vec2fTypes>;
  template class BubblePackingForceField<Vec3fTypes>;
#endif



} // namespace forcefield

} // namespace Components

} // namespace Sofa

