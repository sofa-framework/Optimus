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
#ifndef OPTIMPARAMS_H_
#define OPTIMPARAMS_H_

#include <initOptimusPlugin.h>
#include <sofa/component/component.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>

namespace sofa
{
namespace component
{
namespace container
{

template <class DataTypes>
class OptimParams : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(OptimParams, DataTypes), sofa::core::objectmodel::BaseObject);
    OptimParams();
    ~OptimParams();
    void init();
    void reinit();

protected:
    typedef typename DataTypes::Real        Real;
    typedef typename DataTypes::Coord       Coord;
    typedef typename DataTypes::Deriv       Deriv;

    Data< Coord > m_val;
    Data< Coord > m_initVal;
    Data< Coord > m_min;
    Data< Coord > m_max;
};

} // container
} // component
} // sofa

#endif /*OPTIMPARAMS_H_*/


