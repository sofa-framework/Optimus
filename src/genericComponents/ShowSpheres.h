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
#pragma once

#include "initOptimusPlugin.h"
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/helper/types/RGBAColor.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>



namespace sofa
{

namespace component
{

namespace engine
{


template <class DataTypes>
class ShowSpheres : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ShowSpheres, DataTypes), core::objectmodel::BaseObject);

    typedef typename DataTypes::VecCoord VecCoord;
    typedef defaulttype::Vec<4,float> Vec4f;

ShowSpheres();
~ShowSpheres();

protected:    
public:
    Data<VecCoord> _positions;
    Data<helper::vector<unsigned int> > _indices;
    Data<bool> _draw;
    Data<float> _radius;
    Data<float> _showIndicesSize;
    Data<helper::types::RGBAColor> _color;
    Data<helper::types::RGBAColor> _indexColor;


    void draw(const core::visual::VisualParams* vparams) override;

    void init() override { }
    void reinit() override { }

}; /// class

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SHOWSPHERES_CPP)
extern template class SOFA_OPTIMUSPLUGIN_API ShowSpheres<defaulttype::Vec3Types>;
extern template class SOFA_OPTIMUSPLUGIN_API ShowSpheres<defaulttype::Rigid3Types>;
#endif

} // engine

} // component

} // sofa

