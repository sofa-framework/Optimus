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
#ifndef OBSERVATIONMANAGERBASE_H_
#define OBSERVATIONMANAGERBASE_H_

#include "initOptimusPlugin.h"

#include <sofa/simulation/common/Node.h>

namespace sofa
{
namespace component
{
namespace stochastic
{

using namespace defaulttype;

class ObservationManagerBase: public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(ObservationManagerBase, BaseObject);

    typedef sofa::core::objectmodel::BaseObject Inherit;
    ObservationManagerBase()
        : Inherit()
        , verbose( initData(&verbose, false, "verbose", "print tracing informations") )
    {}

    ~ObservationManagerBase() {}

protected:    
    sofa::simulation::Node* gnode;
    size_t stepNumber;
    double actualTime;

public:
    Data<bool> verbose;

    void init() {
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


}; /// class


template <class FilterType>
class ObservationManager: public sofa::component::stochastic::ObservationManagerBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ObservationManager, FilterType), ObservationManagerBase);

    typedef sofa::component::stochastic::ObservationManagerBase Inherit;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

    ObservationManager()
        :Inherit()
        , observationStdev( initData(&observationStdev, FilterType(0.0), "observationStdev", "standard deviation in observations") )
    {}

    ~ObservationManager() {}

protected:
    size_t observationSize;     /// size of the observation vector
    size_t observationsNumber;  /// number of observations

    FilterType errorVarianceValue;
    EMatrixX errorVariance;
    EMatrixX errorVarianceInverse;

public:
    Data<FilterType> observationStdev;

    virtual bool hasObservation() = 0;
    virtual EVectorX& getInnovation(EVectorX& state) = 0;

    size_t getObservationSize() {
        return observationSize;
    }

    virtual EMatrixX& getErrorVariance() {
        return errorVariance;
    }

    virtual EMatrixX& getErrorVarianceInverse() {
        return errorVarianceInverse;
    }

    void init() {
        Inherit::init();

        gnode = dynamic_cast<sofa::simulation::Node*>(this->getContext());
        if (!gnode) {
            PRNE("Cannot find node!");
            return;
        }
    }

    void bwdInit() {
        Inherit::bwdInit();

        if (observationSize == 0) {
            PRNE("No observations available, cannot allocate the structures!");
        }

        FilterType obsStDev = observationStdev.getValue();
        errorVarianceValue = obsStDev * obsStDev;
        errorVariance = EMatrixX::Identity(observationSize, observationSize) * errorVarianceValue;
        errorVarianceInverse = EMatrixX::Identity(observationSize, observationSize) / errorVarianceValue;
    }

}; /// class


} // stochastic
} // component
} // sofa

#endif // OBSERVATIONMANAGERBASE_H


