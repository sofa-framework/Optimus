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

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/BaseObject.h>
//#include <sofa/core/DataEngine.h>

#include <sofa/type/Mat.h>
#include <sofa/type/Vec.h>

#include <sofa/simulation/Node.h>

#include "../stochasticFiltering/StochasticFilterBase.h"
#include "../stochasticFiltering/StochasticStateWrapperBase.h"
#include "OptimParams.h"

#include <random>
#include <chrono>



namespace sofa
{

namespace component
{

namespace engine
{


using namespace defaulttype;

template <class DataTypes>
class FilteringUpdateEngine : public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(FilteringUpdateEngine, core::objectmodel::BaseObject);

    typedef core::objectmodel::BaseObject Inherit;
    typedef typename sofa::type::Vec3d Vector3;
    
    FilteringUpdateEngine();
    ~FilteringUpdateEngine() {}
    
    Data<size_t> d_addData;
    Data<size_t> d_updateData;

    component::stochastic::StochasticFilterBase* filter;
    type::vector<component::stochastic::StochasticStateWrapperBaseT<DataTypes>*> stateWrappers;
    type::vector<component::container::OptimParamsBase*> vecOptimParams;
    

    // found planes
    sofa::simulation::Node* gnode;
    
    void init() override;

    void bwdInit() override;

    void reinit() override {

    }

    void handleEvent(sofa::core::objectmodel::Event* event) override;
    
    //virtual std::string getTemplateName() const override
    //{
    //    return templateName(this);
    //}

    //static std::string templateName(const FilteringUpdateEngine<DataTypes>* = NULL)
    //{
    //    return "double";
    //}
};



} // engine

} // component

} // sofa

