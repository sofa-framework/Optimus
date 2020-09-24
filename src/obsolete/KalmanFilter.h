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
#ifndef SOFA_COMPONENT_ENGINE_KALMANFILTER_H
#define SOFA_COMPONENT_ENGINE_KALMANFILTER_H

// sofa
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/helper/vector.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/accessor.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include "initOptimusPlugin.h"
#include "ekfilter.hpp"

namespace sofa
{

namespace component
{

namespace engine
{

/*
 * This engine applies a Extended Kalman Filter to a data set
 */
template <class DataTypes>
class SOFA_OptimusPlugin_API KalmanFilter : public core::DataEngine, public Kalman::EKFilter<DataTypes,false,true>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(KalmanFilter, DataTypes), core::DataEngine);
    typedef typename sofa::defaulttype::Vector3 Vec3;
    typedef typename defaulttype::Vec1d Vec1d;
    typedef typename sofa::helper::vector<Vec1d> VecVec1d;
    typedef typename sofa::helper::vector<DataTypes> VecReal;
    typedef Kalman::EKFilter<DataTypes, false, true> inherit;
    typedef typename Kalman::EKFilter<DataTypes, false,true>::Vector Vector;
    typedef typename Kalman::EKFilter<DataTypes, false,true>::Matrix Matrix;

public:
    KalmanFilter();
    ~KalmanFilter();
    void init();
    void reinit();
    void update();

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const KalmanFilter<DataTypes>* = NULL)
    {
        std::string temp("Template is 'Real' (float or double)");
        return temp;
    }

protected:
    Data<VecVec1d> f_inputX; ///< input positions
    Data<VecVec1d> f_inputV; ///< input velocities
    Data<VecReal> f_outputX; ///< ouput positions

    double Period, Mass, Bfriction, Portance, Gravity;

    virtual void makeBaseA();
    virtual void makeBaseH();
    virtual void makeBaseV();
    virtual void makeBaseR();
    virtual void makeBaseW();
    virtual void makeBaseQ();

    virtual void makeA();
    virtual void makeH();
    virtual void makeProcess();
    virtual void makeMeasure();

private:
    std::ifstream _dataInput;
    std::ofstream _dataOutput;
    unsigned _NTRY;
    unsigned _nbStates;	//nb states
    unsigned _nbMeasures; //nb measures
    Vector   _F;
    Matrix   _Measure;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_ENGINE_KALMANFILTER_CPP)

#ifndef SOFA_FLOAT
extern template class SOFA_ENGINE_API KalmanFilter<defaulttype::Vec1dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
extern template class SOFA_ENGINE_API KalmanFilter<defaulttype::Vec1fTypes>;
#endif //SOFA_DOUBLE
#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif
