/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2016 INRIA, USTL, UJF, CNRS, MGH                    *
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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: Igo Peterlik                                                       *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_CORE_BEHAVIOR_FORCEFIELDSENSITIVITY_H
#define SOFA_CORE_BEHAVIOR_FORCEFIELDSENSITIVITY_H

#include <sofa/core/core.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MultiMatrixAccessor.h>
#include <sofa/defaulttype/BaseMatrix.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa
{

namespace core
{

namespace behavior
{

/**
 *  \brief Component to allow for computing of the sensitivity matrix of a model (probably force field)
 *
 */
class SOFA_CORE_API ForceFieldSensitivity : public virtual objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(ForceFieldSensitivity, objectmodel::BaseObject);
    SOFA_BASE_CAST_IMPLEMENTATION(ForceFieldSensitivity)
protected:
    ForceFieldSensitivity();
    virtual ~ForceFieldSensitivity() {}

private:
    BaseForceField(const ForceFieldSensitivity& n) ;
    BaseForceField& operator=(const ForceFieldSensitivity& n) ;


};  /// class

} /// namespace behaviour

} /// namespace core

} /// namespace sofa

#endif
