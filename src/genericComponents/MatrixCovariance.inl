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
#include "MatrixCovariance.h"


#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/core/loader/BaseLoader.h>
#include <sofa/core/loader/PrimitiveGroup.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/helper/fixed_array.h>

namespace sofa
{
namespace component
{
namespace container
{

template <class DataTypes>
MatrixCovariance<DataTypes>::MatrixCovariance()
    : d_val( initData(&d_val, "values", "values for covariance matrix") )
    , d_rows( initData(&d_rows, "rows", "amount of rows in covariance matrix") )
    , d_columns( initData(&d_columns, "columns", "amount of columns in covariance matrix") )
{
}

template <class DataTypes>
MatrixCovariance<DataTypes>::~MatrixCovariance()
{
}

template <class DataTypes>
void MatrixCovariance<DataTypes>::init()
{

}

template <class DataTypes>
void MatrixCovariance<DataTypes>::reinit()
{
    init();
}



template <class DataTypes>
void MatrixCovariance<DataTypes>::bwdInit()
{

}

} // container
} // component
} // sofa

