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


#include "OptimParams.h"

#include <sofa/type/Vec.h>
#include <sofa/type/Mat.h>
#include <sofa/core/loader/BaseLoader.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/type/fixed_array.h>



namespace sofa
{

namespace component
{

namespace container
{



template <class DataTypes>
OptimParams<DataTypes>::OptimParams(loader_t* mm)
    : OptimParamsBase()
    //, m_paramMOLink(initLink("parametricMechanicalObject", "link to a mechanical object which is considered as parameter (only for VecCoord3D)"))
    , m_paramMOLinkrigid(initLink("parametricMechanicalObjectRigid", "link to a mechanical object which is considered as parameter (only for Rigid)"))
    , m_addedVal(initData(&m_addedVal, "addedVal", "added parameters values"))
    , m_addedMinVal(initData(&m_addedMinVal, "addedMinVal", "added minimal bound for parameters values"))
    , m_addedMaxVal(initData(&m_addedMaxVal, "addedMaxVal", "added maximal bound for parameters values"))
    , m_addedStd(initData(&m_addedStd, "addedStd", "added std values"))
    , m_val(initData(&m_val, "value", "parameter value"))
    , m_initVal(initData(&m_initVal, "initValue", "initial parameter value"))
    , m_minVal(initData(&m_minVal, "minValue", "lower bound for parameter"))
    , m_maxVal(initData(&m_maxVal, "maxValue", "higher bound for parameter"))
    , m_stdev(initData(&m_stdev, "stdev", "standard variation"))
    , m_loader(initLink("loader", "loader for mechanical state for which we approximate stiffness of ALL elements"), mm)
{ }



template <class DataTypes>
OptimParams<DataTypes>::~OptimParams()
{ }



template <class DataTypes>
void OptimParams<DataTypes>::init()
{ }



template <class DataTypes>
void OptimParams<DataTypes>::reinit()
{
    init();
}



template<>
void OptimParams<type::vector<double> >::bwdInit()
{
    loader_t* myLoader = m_loader.get();
    if (myLoader)
    {
        size_t count = myLoader->d_tetrahedra.getValue().size() + myLoader->d_hexahedra.getValue().size();
        msg_info(this) << "Optim params <" << getName() << "> found loader <" << myLoader->getName() << "> with " << count << " elements.\n";

        sofa::helper::WriteAccessor< Data<type::vector<double> > > initValues = m_initVal;
        sofa::helper::WriteAccessor< Data<type::vector<double> > > values = m_val;
        sofa::helper::WriteAccessor< Data<type::vector<double> > > min = m_minVal;
        sofa::helper::WriteAccessor< Data<type::vector<double> > > max = m_maxVal;
        sofa::helper::WriteAccessor< Data<type::vector<double> > > stdev = m_stdev;

        initValues.resize(count);
        values.resize(count);
        min.resize(count);
        max.resize(count);
        stdev.resize(count);

        m_numParams = count;

        for (size_t i = 0; i < count; i++)
        {
             initValues[i] = m_initVal.getValue()[0];
             values[i] = m_val.getValue()[0];
             min[i] = m_minVal.getValue()[0];
             max[i] = m_maxVal.getValue()[0];
             stdev[i] = m_stdev.getValue()[0];
        }
    }
}



} // namespace container

} // namespace component

} // namespace sofa

