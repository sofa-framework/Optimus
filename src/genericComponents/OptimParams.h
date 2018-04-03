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

#include <cmath>
#include <fstream>

#include "initOptimusPlugin.h"
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/core/loader/MeshLoader.h>

#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#ifdef Success
#undef Success // dirty workaround to cope with the (dirtier) X11 define. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=253
#endif
#include <Eigen/Dense>

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
struct templateName<Vec3dTypes::VecDeriv>
{
    std::string operator ()(void) { return("VecDeriv3d"); }
};

template<>
struct templateName<Rigid3dTypes::VecDeriv>
{
    std::string operator ()(void) { return("RigidDeriv3d"); }
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
    SOFA_ABSTRACT_CLASS(OptimParamsBase, BaseObject);

    typedef helper::vector<double> DVec;
    typedef helper::vector<size_t> IVec;

    typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorXf;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorXd;

protected:    
    size_t m_dim;
    size_t mstate_dim;
    Data< bool > m_optimize;            ///if OptimParams component are used in Verdandi optimization
    Data< size_t > m_numParams;
    Data< std::string > m_transformParams;
    Data< helper::vector<double> > m_prescribedParamKeys;
    Data< std::string > m_exportParamFile;
    Data< bool> m_interpolateSmooth;
    bool saveParam;
    int transParamType;

    IVec paramIndices;  /// mapping of parameters stored in m_val to Verdandi state vector

public:
    
    //virtual void rawVectorToParams(const double* _vector) = 0;  /// copy values from a input vector into parameters at correct positions
    //virtual void paramsToRawVector(double* _vector) = 0;  /// copy values from parameters to the output vector at correct positions

    virtual void vectorToParams(VectorXf& _vector) = 0;
    virtual void vectorToParams(VectorXd& _vector) = 0;
    virtual void paramsToVector(VectorXf& _vector) = 0;
    virtual void paramsToVector(VectorXd& _vector) = 0;

    virtual void getInitVariance(DVec& _variance) = 0;  /// provide variance as a vector of length given by the number of parameters    


    OptimParamsBase()
        : m_dim(1)
        , m_optimize( initData(&m_optimize, false, "optimize", "the parameters handled in the component will be optimized by Verdandi") )
        , m_numParams( initData(&m_numParams, size_t(0), "numParams", "number of params for vectorial data (input values replicated)") )
        , m_transformParams( initData(&m_transformParams, "transformParams", "transform estimated params. Choice: none, absolute, sigmoid, exponential, project") )
        , m_prescribedParamKeys( initData (&m_prescribedParamKeys, "prescribedParamKeys", "prescribed params in list format: ti p1i ... pni") )
        , m_exportParamFile( initData(&m_exportParamFile, std::string(""), "exportParamFile", "store the parameter at the begining of each time step") )
        , m_interpolateSmooth( initData(&m_interpolateSmooth, true, "interpolateSmooth", "use hyperbolic tangent to interpolate the parameters (linear interpolation if false") )        
    {}

    void init() {       
        if (!m_prescribedParamKeys.getValue().empty()) {
            if (m_optimize.getValue()) {
                std::cout << this->getName() << ": WARNING: parameters can be either optimized or prescribed, optimization set to false" << std::endl;
                m_optimize.setValue(false);
            }

            this->f_listening.setValue(true);
        }

        transParamType = -1;
        std::string transf = m_transformParams.getValue();

        if (std::strcmp(transf.c_str(), "none") == 0)
            transParamType = 0;

        if (std::strcmp(transf.c_str(), "absolute") == 0)
            transParamType = 1;

        if (std::strcmp(transf.c_str(), "sigmoid") == 0)
            transParamType = 2;

        if (std::strcmp(transf.c_str(), "exponential") == 0)
            transParamType = 3;

        if (std::strcmp(transf.c_str(), "project") == 0)
            transParamType = 4;

    }

    size_t size() {
        return(m_numParams.getValue() * m_dim);
    }

    IVec& getVStateParamIndices() {
        return paramIndices;
    }

    void setVStateParamIndices(IVec& _vector) {
        paramIndices = _vector;
    }

    bool isOptimized()  {
        return(m_optimize.getValue());
    }
    void setOptimize(bool value)
    {
        m_optimize.setValue(value);
    }
};

//////////////////////////////////////////////////////////////////////////
template <class DataTypes>
class OptimParams : public OptimParamsBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(OptimParams, DataTypes), OptimParamsBase);

    typedef typename sofa::core::loader::MeshLoader loader_t;
    typedef OptimParamsBase Inherit;

    OptimParams(loader_t* = NULL);
    ~OptimParams();
    void init();
    void reinit();
    void bwdInit(){}

    typedef core::behavior::MechanicalState<defaulttype::Vec3dTypes> MechStateVec3d;
    typedef core::behavior::MechanicalState<defaulttype::Rigid3dTypes> MechStateRigid3d;

    SingleLink<OptimParams<DataTypes>, MechStateVec3d, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> m_paramMOLink;
    SingleLink<OptimParams<DataTypes>, MechStateRigid3d, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> m_paramMOLinkrigid;
    MechStateVec3d* paramMO;
    MechStateRigid3d* paramMOrigid;
    void getInitVariance(DVec& /*_variance*/)  {}    

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
    static std::string templateName(const OptimParams<DataTypes>* = NULL) { std::string name = sofa::component::container::templateName<DataTypes>()(); return(name); }


protected:
    Data< DataTypes > m_val;            /// real actual value of parameters
    Data< DataTypes > m_initVal;        /// initial value
    Data< DataTypes > m_minVal;
    Data< DataTypes > m_maxVal;
    Data< DataTypes > m_stdev;          /// standard deviation

    SingleLink<OptimParams<DataTypes>, loader_t, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> m_loader;
    std::vector<std::pair<double, DataTypes> > m_paramKeys;

    /// must be implemented in specializations    
    virtual void vectorToParams(VectorXf& /*_vector*/) {
//        std::cerr << "[" << this->getName() << "] ERROR: vectorToParams not implemented!" << std::endl;
    }

    virtual void vectorToParams(VectorXd& /*_vector*/) {
        std::cerr << "[" << this->getName() << "] ERROR: vectorToParams not implemented!" << std::endl;
    }

    virtual void paramsToVector(VectorXf& /*_vector*/) {
//        std::cerr << "[" << this->getName() << "] ERROR: paramsToVector not implemented!" << std::endl;
    }

    virtual void paramsToVector(VectorXd& /*_vector*/) {
        std::cerr << "[" << this->getName() << "] ERROR: paramsToVector not implemented!" << std::endl;
    }   

    virtual void handleEvent(core::objectmodel::Event */*event*/) {}


    template<class X, class Y> inline X sigmoid(X arg, Y maxBound, Y minBound) {
        return 1.0 / (1.0 + std::exp(-arg / ((X)maxBound - (X)minBound))) * ((X)maxBound - (X)minBound) - (X)minBound;
    }

    template<class X, class Y> inline X logit(X arg, Y maxBound, Y minBound) {
        return -std::log(1.0 * ((X)maxBound - (X)minBound) / (arg + (X)minBound) - 1.0) * ((X)maxBound - (X)minBound);
    }

};

} // container
} // component
} // sofa

#endif /*OPTIMPARAMS_H_*/


//    void rawVectorToParams(const double* _rawVector, size_t /*_size*/) {
//        /*if (!_size != size()) {
//            std::cerr << "Vector sizes differ!" << std::endl;
//            return;
//        }*/
//        rawVectorToParams(_rawVector);
//    }



//    void paramsToRawVector(double* _rawVector, size_t /*_size*/) {

//            paramsToRawVector(_rawVector);
//    }

//    virtual void rawVectorToParamsParallel(const double* /*_vector*/){std::cout<<"failure!\n";}
//    virtual void paramsToRawVectorParallel(double* /*_vector*/){std::cout<<"failure!\n";}

    //void rawVectorToParamsParallel(const double* _rawVector);
    //void paramsToRawVectorParallel(double* __rawVector);


//    virtual void rawVectorToParams(const double* /*_vector*/) {
//        std::cerr << "[" << this->getName() << "] ERROR: rawVectorToParams not implemented!" << std::endl;
//    }

//    virtual void paramsToRawVector(double* /*_vector*/) {
//        std::cerr << "[" << this->getName() << "] ERROR: paramsToRawVector not implemented!" << std::endl;
//    }

//    virtual void rawVectorToParamsParallel(const double* /*_vector*/){}
//    virtual void paramsToRawVectorParallel(double* /*_vector*/){}
