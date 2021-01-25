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
#define SOFA_COMPONENT_ENGINE_INDICES2VALUESTRANSFORMER_CPP

#include "Indices2ValuesTransformer.inl"
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/ObjectFactory.h>


namespace sofa
{

namespace component
{

namespace engine
{

using namespace sofa;
using namespace sofa::defaulttype;

SOFA_DECL_CLASS(Indices2ValuesTransformer)

int Indices2ValuesTransformerClass = core::RegisterObject("?")
    .add< Indices2ValuesTransformer<Vec3Types> >(true)
    ;


template class SOFA_OPTIMUSPLUGIN_API Indices2ValuesTransformer<Vec3Types>;


} // namespace engine

} // namespace component

} // namespace sofa
