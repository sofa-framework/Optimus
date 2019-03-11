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
#ifndef CombinedObservationManager_H_
#define CombinedObservationManager_H_

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
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

using namespace defaulttype;

template <class FilterType, class DataTypes1, class DataTypes2>
class CombinedObservationManager: public sofa::component::stochastic::ObservationManager<FilterType>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE3(CombinedObservationManager, FilterType, DataTypes1, DataTypes2), SOFA_TEMPLATE(ObservationManager, FilterType));

    typedef typename sofa::component::stochastic::ObservationManager<FilterType> Inherit;
    //typedef typename Inherit::EVectorX EVectorX;
    //typedef typename Inherit::EMatrixX EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

    typedef typename DataTypes1::Real Real1;
    typedef core::behavior::MechanicalState<DataTypes2> MasterState;
    typedef sofa::component::container::SimulatedStateObservationSource<DataTypes1> ObservationSource1;
    typedef sofa::component::container::SimulatedStateObservationSource<DataTypes1> ObservationSource2;

    typedef StochasticStateWrapper<DataTypes2,FilterType> StateWrapper; ///Before it was DataTypes1

    typedef typename DataTypes2::VecCoord VecCoord;
    typedef typename DataTypes2::VecDeriv VecDeriv;


    CombinedObservationManager();
    ~CombinedObservationManager() {}

protected:

    MasterState* masterState;
    ObservationSource1 *observationSource1;
    ObservationSource2 *observationSource2;

    StateWrapper* stateWrapper;

    double actualObservationTime;



public:
    void init();
    void bwdInit();

    virtual bool hasObservation(double _time); /// TODO
    virtual bool getInnovation(double _time, EVectorX& _state, EVectorX& _innovation);
    virtual bool getRealObservation(double _time, EVectorX& _realObs);
    virtual bool obsFunction(EVectorX& _state, EVectorX& _predictedObservation);
    virtual bool getPredictedObservation(int _id, EVectorX& _predictedObservation);

    typename DataTypes1::VecCoord realObservations1;
    typename DataTypes1::VecCoord realObservations2;
    bool hasObservation1, hasObservation2;
    size_t obsSize1, obsSize2;
    typename helper::vector< VecCoord > modelObservations;

    Data<bool> d_use2dObservations;
    Data<Mat3x4d> d_projectionMatrix;

    SingleLink<CombinedObservationManager<FilterType, DataTypes1, DataTypes2>, StateWrapper, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> stateWrapperLink;
    SingleLink<CombinedObservationManager<FilterType, DataTypes1, DataTypes2>, ObservationSource1, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> observationWrapperLink1;
    SingleLink<CombinedObservationManager<FilterType, DataTypes1, DataTypes2>, ObservationSource2, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> observationWrapperLink2;


    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        //        if (dynamic_cast<MState *>(context->getMechanicalState()) == NULL) return false;
        return sofa::core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const CombinedObservationManager< FilterType, DataTypes1,  DataTypes2>* = NULL)
    {
        return DataTypes1::Name()+ std::string(",") + DataTypes2::Name();
    }


}; /// class


} // stochastic
} // component
} // sofa

#endif // CombinedObservationManager_H


