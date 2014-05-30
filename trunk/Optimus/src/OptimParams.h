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
#ifndef OPTIMPARAMS_H_
#define OPTIMPARAMS_H_

#include "initOptimusPlugin.h"
#include <sofa/component/component.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>

namespace sofa
{
namespace component
{
namespace container
{

using namespace defaulttype;

template <class DataTypes>
struct templateName
{
    std::string operator ()(void) { return("generic"); }
};

template<>
struct templateName<Vec3dTypes::VecCoord>
{
    std::string operator ()(void) { return("VecCoord3d"); }
};

template<>
struct templateName<Vec3fTypes::VecCoord>
{
    std::string operator ()(void) { return("VecCoord3f"); }
};


template<>
struct templateName<double>
{
    std::string operator ()(void) { return("double"); }
};


template<>
struct templateName<sofa::defaulttype::RigidCoord<3,double> >
{
    std::string operator ()(void) { return("Rigid3d"); }
};


template<>
struct templateName<sofa::defaulttype::RigidCoord<2,double> >
{
    std::string operator ()(void) { return("Rigid2d"); }
};

template<>
struct templateName<sofa::defaulttype::Vec3d>
{
    std::string operator ()(void) { return("Vec3d"); }
};


template<>
struct templateName<sofa::defaulttype::Vec2d>
{
    std::string operator ()(void) { return("Vec2d"); }
};


template<>
struct templateName<sofa::defaulttype::Vec1d>
{
    std::string operator ()(void) { return("Vec1d"); }
};

template<>
struct templateName<sofa::helper::vector<double> >
{
    std::string operator ()(void) { return("Vector"); }
};

template<>
struct templateName<sofa::helper::vector<float> >
{
    std::string operator ()(void) { return("Vector"); }
};

class OptimParamsBase : public sofa::core::objectmodel::BaseObject
{
    typedef helper::vector<double> DVec;

protected:
    DVec _stDevInVector;
    DVec _valueInVector;
    Data< bool > m_optimize;            ///

    virtual void _getStDev() = 0;   /// copy standard deviation from a structure of given type to plain helper::vector
    virtual void _getValue() = 0;   /// copy the actual value from a structure of given type to plain helper::vector
    virtual void _setValue() = 0;   /// copy a value in plain vector to a structure of given type

public:
    helper::vector<size_t> mappingIndices;  /// mapping of parameters stored in m_val to Verdandi state vector

    OptimParamsBase()
        : m_optimize( initData(&m_optimize, true, "optimize", "the parameter will be optimized by Verdandi") )
    {}

    size_t size() {
        return(_valueInVector.size());
    }

    DVec& getStDev() {
        this->_getStDev();
        return _stDevInVector;
    }

    DVec& getValue() {
        this->_getValue();
        return _valueInVector;
    }

    void setValue(DVec& _value) {
        _valueInVector = _value;
        this->_setValue();
    }

    bool optimize()  {
        return(m_optimize.getValue());
    }
};

//////////////////////////////////////////////////////////////////////////
template <class DataTypes>
class OptimParams : public OptimParamsBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(OptimParams, DataTypes), sofa::core::objectmodel::BaseObject);    
    OptimParams();
    ~OptimParams();
    void init();
    void reinit();

    static std::string templateName(const OptimParams<DataTypes>* = NULL) { std::string name = sofa::component::container::templateName<DataTypes>()(); return(name); }       

protected:
    Data< DataTypes > m_val;            /// real actual value of parameters
    Data< DataTypes > m_initVal;        /// initial value
    Data< DataTypes > m_min;
    Data< DataTypes > m_max;
    Data< DataTypes > m_stdev;          /// standard deviation
    Data< int > m_numParams;

    /// must be implemented in specializations
    virtual void _getStDev() {}
    virtual void _getValue() {}
    virtual void _setValue() {}

};

} // container
} // component
} // sofa

#endif /*OPTIMPARAMS_H_*/


