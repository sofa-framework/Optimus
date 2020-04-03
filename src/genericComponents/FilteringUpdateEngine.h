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
#ifndef FILTERING_UPDATE_ENGINE_H_
#define FILTERING_UPDATE_ENGINE_H_

//#include "initPLUGIN.h"
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/defaulttype.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/BaseObject.h>
//#include <sofa/core/DataEngine.h>

#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Vec.h>

#include <sofa/simulation/Node.h>

#include "../stochasticFiltering/UKFilterClassicOrig.h"
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
    typedef typename sofa::defaulttype::Vec3d Vector3;
    
    FilteringUpdateEngine();
    ~FilteringUpdateEngine() {}
    
    Data<size_t> d_updateData;

    component::stochastic::UKFilterClassicOrig<DataTypes>* filter;
    helper::vector<component::stochastic::StochasticStateWrapperBaseT<DataTypes>*> stateWrappers;
    helper::vector<component::container::OptimParamsBase*> vecOptimParams;
    

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
    
}; /// class


} // engine
} // component
} // sofa

#endif // FILTERING_UPDATE_ENGINE_H_


