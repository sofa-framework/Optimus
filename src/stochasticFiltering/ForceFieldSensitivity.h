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
 *  Component to allow for computing of the sensitivity matrix of a model (probably force field)
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
    BaseForceField(const ForceFieldSensitivity& n);
    BaseForceField& operator=(const ForceFieldSensitivity& n);
};



} // namespace behaviour

} // namespace core

} // namespace sofa

