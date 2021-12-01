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
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include "initOptimusPlugin.h"
#include "ObservationManagerBase.h"
#include "../genericComponents/SimulatedStateObservationSource.h"
#include "StochasticStateWrapper.h"



namespace sofa
{

namespace component
{

namespace stochastic
{



template <class FilterType, class DataTypes1, class DataTypes2>
class SimpleObservationManager : public sofa::component::stochastic::ObservationManager<FilterType>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE3(SimpleObservationManager, FilterType, DataTypes1, DataTypes2), SOFA_TEMPLATE(ObservationManager, FilterType));

    typedef typename sofa::component::stochastic::ObservationManager<FilterType> Inherit;
    //typedef typename Inherit::EVectorX EVectorX;
    //typedef typename Inherit::EMatrixX EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

    typedef typename DataTypes1::Real Real1;
    typedef typename DataTypes2::VecCoord VecCoord;
    typedef typename DataTypes2::VecDeriv VecDeriv;

    typedef core::behavior::MechanicalState<DataTypes2> MasterState;
    typedef sofa::component::container::SimulatedStateObservationSource<DataTypes1> ObservationSource;
    typedef StochasticStateWrapper<DataTypes2,FilterType> StateWrapper; ///Before it was DataTypes1
    typename DataTypes1::VecCoord realObservations;
    typename type::vector< VecCoord > modelObservations;

    Data< sofa::type::Mat3x4d > d_projectionMatrix;
    SingleLink<SimpleObservationManager<FilterType, DataTypes1, DataTypes2>, StateWrapper, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> stateWrapperLink;


protected:
    MasterState* masterState;
    ObservationSource* observationSource;
    StateWrapper* stateWrapper;
    double actualObservationTime;

public:
    SimpleObservationManager();
    virtual ~SimpleObservationManager() override {}

    void init() override;
    void bwdInit() override;

    virtual bool hasObservation(double _time) override;
    virtual bool getInnovation(double _time, EVectorX& _state, EVectorX& _innovation) override;
    virtual bool getRealObservation(double _time, EVectorX& _realObs) override;
    virtual bool obsFunction(EVectorX& _state, EVectorX& _predictedObservation) override;
    virtual bool getPredictedObservation(int _id, EVectorX& _predictedObservation) override;

    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        //        if (dynamic_cast<MState *>(context->getMechanicalState()) == NULL) return false;
        return sofa::core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const SimpleObservationManager< FilterType, DataTypes1,  DataTypes2>* = NULL)
    {
        return DataTypes1::Name()+ std::string(",") + DataTypes2::Name();
    }
};


extern template class SOFA_STOCHASTIC_API SimpleObservationManager<double, defaulttype::Vec3Types, defaulttype::Vec3Types>;
extern template class SOFA_STOCHASTIC_API SimpleObservationManager<double, defaulttype::Vec2Types, defaulttype::Vec3Types>;
extern template class SOFA_STOCHASTIC_API SimpleObservationManager<double, defaulttype::Vec2Types, defaulttype::Rigid3Types>;
extern template class SOFA_STOCHASTIC_API SimpleObservationManager<double, defaulttype::Vec3Types, defaulttype::Rigid3Types>;



} // stochastic

} // component

} // sofa

