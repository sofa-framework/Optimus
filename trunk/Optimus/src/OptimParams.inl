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
#include "OptimParams.h"


namespace sofa
{
namespace component
{
namespace container
{

template <class DataTypes>
OptimParams<DataTypes>::OptimParams()
    : OptimParamsBase()
    , m_val( initData(&m_val, "value", "parameter value") )
    , m_initVal( initData(&m_initVal, "initValue", "initial parameter value") )
    , m_min( initData(&m_min, "min", "lower bound for parameter") )
    , m_max( initData(&m_max, "max", "higher bound for parameter") )
    , m_stdev( initData(&m_stdev, "stdev", "standard variation") )
{    
}

template <class DataTypes>
OptimParams<DataTypes>::~OptimParams()
{
}

template <class DataTypes>
void OptimParams<DataTypes>::init()
{

}

template <class DataTypes>
void OptimParams<DataTypes>::reinit()
{
    init();
}

} // container
} // component
} // sofa

