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

#include "initOptimusPlugin.h"
#include <sofa/simulation/Node.h>
#include <Eigen/Dense>



namespace sofa
{

namespace component
{

namespace stochastic
{



class ObservationManagerBase : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(ObservationManagerBase, BaseObject);
    typedef sofa::core::objectmodel::BaseObject Inherit;

protected:    
    sofa::simulation::Node* gnode;
    size_t stepNumber;
    double actualTime;

public:
    Data<bool> verbose;

    ObservationManagerBase()
        : Inherit()
        , verbose( initData(&verbose, false, "verbose", "print tracing informations") )
    {}

    ~ObservationManagerBase() {}

    void init() override {
        Inherit::init();

        gnode = dynamic_cast<sofa::simulation::Node*>(this->getContext());
        if (!gnode) {
            PRNE("Cannot find node!");
            return;
        }
    }

    virtual void initializeStep(size_t _stepNumber) {
        stepNumber = _stepNumber;
        actualTime = double(stepNumber)*gnode->getDt();
    }
};



template <class FilterType>
class ObservationManager: public sofa::component::stochastic::ObservationManagerBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ObservationManager, FilterType), ObservationManagerBase);

    typedef sofa::component::stochastic::ObservationManagerBase Inherit;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

    Data<FilterType> observationStdev;
    Data<bool> initialiseObservationsAtFirstStep;

protected:
    size_t observationSize;               /// size of the observation vector
    FilterType errorVarianceValue;
    EMatrixX errorVariance;
    EMatrixX errorVarianceInverse;

public:
    ObservationManager()
        :Inherit()
        , observationStdev( initData(&observationStdev, FilterType(0.0), "observationStdev", "standard deviation in observations") )
        , initialiseObservationsAtFirstStep( initData(&initialiseObservationsAtFirstStep, false, "initialiseObservationsAtFirstStep", "if true initialise component during first iteration") )
    {}
    ~ObservationManager() {}

    virtual bool hasObservation(double _time) = 0;
    virtual bool getInnovation(double _time, EVectorX& _state, EVectorX& _innovation) = 0;
    virtual bool getRealObservation(double _time, EVectorX& _realObs) = 0;
    virtual bool obsFunction(EVectorX& _state, EVectorX& _predictedObservation) = 0;
    virtual bool getPredictedObservation(int _id, EVectorX& _predictedObservation) = 0;

    size_t getObservationSize()
    {
        return observationSize;
    }

    virtual EMatrixX& getErrorVariance()
    {
        return errorVariance;
    }

    virtual EMatrixX& getErrorVarianceInverse()
    {
        return errorVarianceInverse;
    }

    void init() override
    {
        Inherit::init();

        gnode = dynamic_cast<sofa::simulation::Node*>(this->getContext());
        if (!gnode) {
            PRNE("Cannot find node!");
            return;
        }
    }

    void bwdInit() override
    {
        Inherit::bwdInit();

        if (!initialiseObservationsAtFirstStep.getValue()) {
            initializeObservationData();
        }
    }

    void initializeObservationData()
    {
        if (observationSize == 0) {
            PRNE("No observations available, cannot allocate the structures!");
        }

        FilterType obsStDev = observationStdev.getValue();
        errorVarianceValue = obsStDev * obsStDev;
        errorVariance = EMatrixX::Identity(observationSize, observationSize) * errorVarianceValue;
        errorVarianceInverse = EMatrixX::Identity(observationSize, observationSize) / errorVarianceValue;
    }
};



} // stochastic

} // component

} // sofa

