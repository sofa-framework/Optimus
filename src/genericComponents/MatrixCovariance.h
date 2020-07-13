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
#pragma once

#include <cmath>
#include <fstream>

#include "../initOptimusPlugin.h"
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>


namespace sofa
{

namespace component
{

namespace container
{



template <class DataTypes>
class MatrixCovariance : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(MatrixCovariance, DataTypes), BaseObject);

    MatrixCovariance();
    ~MatrixCovariance();
    void init() override;
    void reinit() override;
    void bwdInit() override;

    unsigned int Rows() { return d_rows.getValue(); }
    unsigned int Columns() { return d_columns.getValue(); }
    const helper::vector<DataTypes>& GetData() { return d_val.getValue(); }

protected:
    Data<helper::vector<DataTypes>> d_val;    // real actual value of parameters
    Data<unsigned int> d_rows;                // amount of rows for covariance matrix
    Data<unsigned int> d_columns;             // amount of columns for covariance matrix

    virtual void handleEvent(core::objectmodel::Event */*event*/) override {}

};



} // container

} // component

} // sofa

