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
#ifndef SOFASTATEWRAPPER_H_
#define SOFASTATEWRAPPER_H_

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/simulation/common/Node.h>

#include "initOptimusPlugin.h"

#ifdef Success
#undef Success // dirty workaround to cope with the (dirtier) X11 define. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=253
#endif
#include <Eigen/Dense>

namespace sofa
{
namespace component
{
namespace stochastic
{

class StochasticStateWrapperBase: public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(StochasticStateWrapperBase, BaseObject);
    typedef sofa::core::objectmodel::BaseObject Inherit;

    StochasticStateWrapperBase()
        : Inherit()
        , verbose( initData(&verbose, false, "verbose", "print tracing informations") )
        , slave( initData(&slave, false, "slave", "slave wrapper (needed only for parallelization") ) {

    }

protected:
    sofa::simulation::Node* gnode;
    size_t stepNumber;
    double actualTime;

    sofa::helper::system::thread::CTime *timer;
    double startTime, stopTime;

    const core::ExecParams* execParams;

public:
    Data<bool> verbose;
    Data<bool> slave;

    void init() {
        Inherit::init();

        gnode = dynamic_cast<sofa::simulation::Node*>(this->getContext());
        if (!gnode) {
            PRNE("Cannot find node!");
            return;
        }
    }

    virtual void initializeStep(const core::ExecParams* _execParams, size_t _stepNumber) {
        execParams = _execParams;
        stepNumber = _stepNumber;
        actualTime = double(stepNumber)*gnode->getDt();
        PRNS("========= Initialize DA step T = " << actualTime);
    }

    bool isSlave() {
        return slave.getValue();
    }
}; /// class

template <class FilterType>
class StochasticStateWrapperBaseT: public sofa::component::stochastic::StochasticStateWrapperBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(StochasticStateWrapperBaseT, FilterType), StochasticStateWrapperBase);

    typedef sofa::component::stochastic::StochasticStateWrapperBase Inherit;

    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;


    StochasticStateWrapperBaseT()
        : Inherit()        {
    }

protected:
    size_t reducedStateIndex;
    size_t stateSize, reducedStateSize;

    EVectorX state;

    EMatrixX stateErrorVariance;
    EMatrixX stateErrorVarianceReduced;
    EMatrixX stateErrorVarianceProjector;

    EVectorX stateErrorVarianceRow;


public:
    virtual void applyOperator(EVectorX& _vecX,  bool _preserveState = true, bool _updateForce = true) = 0;
    //virtual void setSofaTime(const core::ExecParams* _execParams) = 0;

    virtual void setStateErrorVarianceProjector(EMatrixX& _mat) {
        stateErrorVarianceProjector = _mat;
    }

    virtual EMatrixX& getStateErrorVariance() {
        return stateErrorVariance;
    }

    virtual EMatrixX& getStateErrorVarianceProjector()  {
        return stateErrorVarianceProjector;
    }

    virtual EMatrixX& getStateErrorVarianceReduced() {
        return stateErrorVarianceReduced;
    }

    virtual EVectorX& getStateErrorVarianceRow(int rowIndex) {
        stateErrorVarianceRow = stateErrorVariance.row(rowIndex);
        return stateErrorVarianceRow;
    }

    virtual EVectorX& getState() {
        return state;
    }

    virtual void setState(EVectorX& _state) {
        state = _state;
    }

    size_t getStateSize() {
        return state.rows();
    }

    void init() {
        Inherit::init();
    }
}; /// class


} // stochastic
} // component
} // sofa

#endif // SOFASTATEWRAPPER_H


