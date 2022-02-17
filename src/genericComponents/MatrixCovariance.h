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

#include <cmath>
#include <fstream>

#include "../initOptimusPlugin.h"
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/type/Vec.h>
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

protected:
    Data<type::vector<DataTypes>> d_val;    // real actual value of parameters
    Data<unsigned int> d_rows;                // amount of rows for covariance matrix
    Data<unsigned int> d_columns;             // amount of columns for covariance matrix

    virtual void handleEvent(core::objectmodel::Event */*event*/) override {}

public:
    MatrixCovariance();
    ~MatrixCovariance();
    void init() override;
    void reinit() override;
    void bwdInit() override;

    unsigned int Rows() { return d_rows.getValue(); }
    unsigned int Columns() { return d_columns.getValue(); }
    const type::vector<DataTypes>& GetData() { return d_val.getValue(); }
};



} // container

} // component

} // sofa

