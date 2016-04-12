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
    typedef sofa::core::objectmodel::BaseObject Inherit;

    StochasticStateWrapperBase()
        : Inherit()
        , verbose( initData(&verbose, false, "verbose", "print tracing informations") ) {

    }

protected:
    sofa::simulation::Node* gnode;
public:
    Data<bool> verbose;

    virtual void applyOperator() = 0;
    void init() {
        Inherit::init();

        gnode = dynamic_cast<sofa::simulation::Node*>(this->getContext());
        if (!gnode) {
            PRNE("Cannot find node!");
            return;
        }

    }
}; /// class

template <class FilterType>
class StochasticStateWrapperBaseT: public sofa::component::stochastic::StochasticStateWrapperBase
{
public:
    typedef sofa::component::stochastic::StochasticStateWrapperBase Inherit;

    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;


    StochasticStateWrapperBaseT()
        : Inherit()        {
    }

protected:



public:
    virtual EMatrixX& getStateErrorVariance() = 0;
    virtual EMatrixX& getStateErrorVarianceProjector() = 0;
    virtual EMatrixX& getStateErrorVarianceReduced() = 0;
    virtual EVectorX& getStateErrorVarianceRow(int row) = 0;

    void init() {
        Inherit::init();
    }
}; /// class


} // stochastic
} // component
} // sofa

#endif // SOFASTATEWRAPPER_H


