/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2020 MGH, INRIA, USTL, UJF, CNRS                    *
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
#define SOFA_COMPONENT_ENGINE_INDICES2VALUESTRANSFORMER_CPP

#include <SofaGeneral/config.h>
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
