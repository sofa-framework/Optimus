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
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>

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
public:
    typedef helper::vector<double> DVec;
    typedef helper::vector<size_t> IVec;

protected:    
    size_t m_dim;
    Data< bool > m_optimize;            ///if OptimParams component are used in Verdandi optimization
    Data< size_t > m_numParams;
    Data< int > m_transformParams;
    Data< helper::vector<double> > m_prescribedParamKeys;
    Data< std::string > m_exportParamFile;
    bool saveParam;

    IVec paramIndices;  /// mapping of parameters stored in m_val to Verdandi state vector

    virtual void getStDevTempl(DVec& _stdev) = 0;   /// copy standard deviation from a structure of given type to plain helper::vector
    //virtual void getValueTempl(DVec& _value) = 0;   /// copy the actual value from a structure of given type to plain helper::vector
    //virtual void setValueTempl(const DVec& _value) = 0;   /// copy a value in plain vector to a structure of given type
    virtual void rawVectorToParams(const double* _vector) = 0;  /// copy values from a input vector into parameters at correct positions
    virtual void paramsToRawVector(double* _vector) = 0;  /// copy values from parameters to the output vector at correct positions

public:


    OptimParamsBase()
        : m_dim(1)
        , m_optimize( initData(&m_optimize, true, "optimize", "the parameters handled in the component will be optimized by Verdandi") )
        , m_numParams( initData(&m_numParams, size_t(1), "numParams", "number of params for vectorial data (input values replicated)") )
        , m_transformParams( initData(&m_transformParams, 0, "transformParams", "transform estimated params: 0: do nothing, 1: absolute value, 2: quadratic (not implemented)") )
        , m_prescribedParamKeys( initData (&m_prescribedParamKeys, "prescribedParamKeys", "prescribed params in list format: ti p1i ... pni") )
        , m_exportParamFile( initData(&m_exportParamFile, std::string(""), "exportParamFile", "store the parameter at the begining of each time step") )
    {}

    void init() {
        if (!m_prescribedParamKeys.getValue().empty()) {
            if (m_optimize.getValue()) {
                std::cout << this->getName() << ": WARNING: parameters can be either optimized or prescribed, optimization set to false" << std::endl;
                m_optimize.setValue(false);
            }

            this->f_listening.setValue(true);
        }
    }

    size_t size() {
        return(m_numParams.getValue() * m_dim);
    }

    void rawVectorToParams(const double* _rawVector, size_t /*_size*/) {
        /*if (!_size != size()) {
            std::cerr << "Vector sizes differ!" << std::endl;
            return;
        }*/
        rawVectorToParams(_rawVector);
    }

    void paramsToRawVector(double* _rawVector, size_t /*_size*/) {
        /*if (!_size != size()) {
            std::cerr << "Vector sizes differ!" << std::endl;
            return;
        }*/
        paramsToRawVector(_rawVector);
    }

    void getStDev(DVec& _stdev) {
        this->getStDevTempl(_stdev);
    }

    /*void getValue(DVec& _value) {
        this->getValueTempl(_value);
    }

    void setValue(const DVec& _value) {
        this->setValueTempl(_value);
    }*/

    IVec& getVStateParamIndices() {
        return paramIndices;
    }

    void setVStateParamIndices(IVec& _vector) {
        paramIndices = _vector;
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

    typedef OptimParamsBase Inherit;

    OptimParams();
    ~OptimParams();
    void init();
    void reinit();

    typedef core::behavior::MechanicalState<defaulttype::Vec3dTypes> MechStateVec3d;
    SingleLink<OptimParams<DataTypes>, MechStateVec3d, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> m_paramMOLink;
    MechStateVec3d* paramMO;

    static std::string templateName(const OptimParams<DataTypes>* = NULL) { std::string name = sofa::component::container::templateName<DataTypes>()(); return(name); }       

protected:
    Data< DataTypes > m_val;            /// real actual value of parameters
    Data< DataTypes > m_initVal;        /// initial value
    Data< DataTypes > m_min;
    Data< DataTypes > m_max;
    Data< DataTypes > m_stdev;          /// standard deviation
    std::vector<std::pair<double, DataTypes> > m_paramKeys;

    /// must be implemented in specializations
    virtual void getStDevTempl(DVec& /*_stdev*/) {}
    //virtual void getValueTempl(DVec& /*_value*/) {}
    //virtual void setValueTempl(const DVec& /*_value*/) {}
    virtual void rawVectorToParams(const double* /*_vector*/) {}
    virtual void paramsToRawVector(double* /*_vector*/) {}

    virtual void handleEvent(core::objectmodel::Event */*event*/) {}

};

} // container
} // component
} // sofa

#endif /*OPTIMPARAMS_H_*/


