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

//#define SOFA_COMPONENT_CONTAINER_OPTIMPARAMS_CPP

#
#include <sofa/core/ObjectFactory.h>
#include "OptimParams.inl"
//#include <sofa/helper/accessor.h>

namespace sofa
{

namespace component
{

namespace container
{

using namespace defaulttype;

/// SPECIALIZATIONS FOR vector<double>

template<>
void OptimParams<sofa::helper::vector<double> >::getStDevTempl(DVec& _stdev) {
    _stdev.resize(m_stdev.getValue().size());    
    for (size_t i = 0; i < _stdev.size(); i++)
        _stdev[i] = m_stdev.getValue()[i];
}

/*template<>
void OptimParams<sofa::helper::vector<double> >::getValueTempl(DVec& _value) {
    _value.resize(m_val.getValue().size());
    for (size_t i = 0; i < _value.size(); i++)
        _value[i] = m_val.getValue()[i];
}

template<>
void OptimParams<sofa::helper::vector<double> >::setValueTempl(const DVec& _value) {
    helper::vector<double>* val = m_val.beginEdit();
    for (size_t i = 0; i < _value.size(); i++)
        val->at(i) = _value[i];
    m_val.endEdit();
}*/

template<>
void OptimParams<sofa::helper::vector<double> >::rawVectorToParams(const double* _vector) {
    helper::WriteAccessor<Data<helper::vector<double> > > val = m_val;

    switch (this->m_transformParams.getValue()) {
    case 1:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=fabs(_vector[this->paramIndices[i]]);
        }
        break;
    default:
        for (size_t i = 0; i < this->paramIndices.size(); i++)
            val[i] = _vector[this->paramIndices[i]];
    }
}

template<>
void OptimParams<sofa::helper::vector<double> >::paramsToRawVector(double* _vector) {
    helper::ReadAccessor<Data<helper::vector<double> > > val = m_val;

    switch (this->m_transformParams.getValue()) {
    case 1:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = fabs(val[i]);
        break;
    default:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = val[i];
    }

}


template<>
void OptimParams<sofa::helper::vector<double> >::init() {
    helper::ReadAccessor<Data<sofa::helper::vector<double> > > initVal = m_initVal;
    size_t nInitVal = initVal.size();

    m_numParams.setValue(nInitVal);

    if (nInitVal == 1 && m_numParams.getValue() > 1) {
        double value = initVal[0];
        nInitVal = m_numParams.getValue();
        helper::WriteAccessor<Data<sofa::helper::vector<double> > > wInitVal = m_initVal;
        wInitVal.wref().resize(nInitVal);
        for (size_t i = 0; i < nInitVal; i++)
            wInitVal[i] = value;
    }


    if (nInitVal != 0) {
        helper::WriteAccessor<Data<sofa::helper::vector<double> > > val = m_val;
        if (val.size() == 0) {
            val.resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                val[i] = initVal[i];
        }

        helper::WriteAccessor<Data<sofa::helper::vector<double> > > minVal = m_min;
        if (minVal.size() == 0) {
            minVal.resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                minVal[i] = initVal[i];
        }

        helper::WriteAccessor<Data<sofa::helper::vector<double> > > maxVal = m_max;
        if (maxVal.size() == 0) {
            maxVal.resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                maxVal[i] = initVal[i];
        }

        helper::WriteAccessor<Data<sofa::helper::vector<double> > > stdev = m_stdev;
        //std::cout << "STDEV: " << stdev << std::endl;
        if (stdev.size() == 0) {
            stdev.resize(nInitVal, 0.0);
        } else if (stdev.size() != nInitVal) {
            std::cerr << this->getName() << ": ERROR: |stdev| != |init value|, taking the first member of stdev. " << std::endl;
            //double x=stdev[0];
            stdev.resize(nInitVal);
            for (size_t i = 1; i < nInitVal; i++)
                stdev[i] = stdev[0];
        }


    }
}


/// SPECIALIZATIONS FOR VecCoord3D

template<>
void OptimParams<Vec3dTypes::VecCoord>::init() {
    paramMO = m_paramMOLink.get();
    this->m_dim = 3;

    if (paramMO == NULL)
        std::cerr << "WARNING: cannot find the parametric mechanical state, assuming no mechanical state is associated with the parameters" << std::endl;
    else {
        std::cout << "Mechanical state associated with the parameters: " << paramMO->getName() << std::endl;

        typename MechStateVec3d::ReadVecCoord moPos = paramMO->readPositions();
        helper::WriteAccessor<Data<Vec3dTypes::VecCoord> > waInitVal = m_initVal;


        int moSize = moPos.size();
        waInitVal.resize(moSize);


        for (int i = 0; i < moSize; i++)
            waInitVal[i] = moPos[i];
    }

    helper::ReadAccessor<Data<Vec3dTypes::VecCoord> > raInitVal = m_initVal;
    helper::WriteAccessor<Data<Vec3dTypes::VecCoord> > waVal = m_val;
    int numParams = raInitVal.size();
    m_numParams.setValue(numParams);
    waVal.resize(numParams);
    for (int i = 0; i < numParams; i++)
        waVal[i] = raInitVal[i];

    helper::ReadAccessor<Data<Vec3dTypes::VecCoord> > raStdDev = m_stdev;
    if (raStdDev.size() != 1) {
        std::cout << this->getName() << ": cannot handle standard deviation, size should be 1 for VecCoord3D" << std::endl;
    } else {
        Vec3dTypes::Coord sd0 = raStdDev[0];
        helper::WriteAccessor<Data<Vec3dTypes::VecCoord> > waStdDev = m_stdev;
        waStdDev.resize(numParams);
        for (int i = 0; i < numParams; i++)
            waStdDev[i] = sd0;
    }
}


template<>
void OptimParams<Vec3dTypes::VecCoord>::rawVectorToParams(const double* _vector) {
    helper::WriteAccessor<Data<Vec3dTypes::VecCoord> > waVal = m_val;

    size_t numParams = m_numParams.getValue();
    size_t k = 0;
    for (size_t i = 0; i < numParams; i++)
        for (size_t j = 0; j < 3; j++, k++)
            waVal[i][j] = _vector[paramIndices[k]];

    if (paramMO != NULL) {
        typename MechStateVec3d::WriteVecCoord moWPos = paramMO->writePositions();
        helper::ReadAccessor<Data<Vec3dTypes::VecCoord> > raVal = m_val;

        for (size_t i = 0; i < numParams; i++)
            moWPos[i] = raVal[i];
    }

}

template<>
void OptimParams<Vec3dTypes::VecCoord>::paramsToRawVector(double* _vector) {
    size_t numParams = m_numParams.getValue();
    if (paramMO != NULL) {
        typename MechStateVec3d::ReadVecCoord moRPos = paramMO->readPositions();
        helper::WriteAccessor<Data<Vec3dTypes::VecCoord> > waVal = m_val;

        for (size_t i = 0; i < numParams; i++)
            waVal[i] = moRPos[i];
    }

    helper::ReadAccessor<Data<Vec3dTypes::VecCoord> > raVal = m_val;
    size_t k = 0;
    for (size_t i = 0; i < numParams; i++)
        for (size_t j = 0; j < 3; j++, k++)
            _vector[paramIndices[k]] = raVal[i][j];

}

template<>
void OptimParams<Vec3dTypes::VecCoord>::getStDevTempl(DVec& _stdev) {
    size_t numParams = this->m_numParams.getValue();
    _stdev.resize(numParams*m_dim);
    size_t ij = 0;
    for (size_t i = 0; i < numParams; i++)
        for (size_t j = 0; j < m_dim; j++, ij++)
            _stdev[ij] = m_stdev.getValue()[i][j];
}

/*template<>
void OptimParams<Vec3dTypes::VecCoord>::getValueTempl(DVec& _value) {
    _value.resize(m_val.getValue().size());
    for (size_t i = 0; i < _value.size(); i++)
        _value[i] = m_val.getValue()[i];
}

template<>
void OptimParams<Vec3dTypes::VecCoord>::setValueTempl(const DVec& _value) {
    helper::vector<double>* val = m_val.beginEdit();
    for (size_t i = 0; i < _value.size(); i++)
        val->at(i) = _value[i];
    m_val.endEdit();
}*/

/// DECLARATIONS



SOFA_DECL_CLASS(OptimParams)

// Register in the Factory
int OptimParamsClass = core::RegisterObject("Optimization Parameters")
        #ifndef SOFA_FLOAT
        .add< OptimParams<double> >(true)
        /*.add< OptimParams<Vec3d> >()
        .add< OptimParams<Vec2d> >()
        .add< OptimParams<Vec1d> >()
        .add< OptimParams<RigidCoord<3,double> > >()
        .add< OptimParams<RigidCoord<2,double> > >()*/
        .add< OptimParams<sofa::helper::vector<double> > >()
        .add< OptimParams<Vec3dTypes::VecCoord> >()
        #endif
        #ifndef SOFA_DOUBLE
        /*.add< OptimParams<float> >(true)
        .add< OptimParams<Vec3f> >()
        .add< OptimParams<Vec2f> >()
        .add< OptimParams<Vec1f> >()
        .add< OptimParams<RigidCoord<3,float> > >()
        .add< OptimParams<RigidCoord<2,float> > >()
        .add< OptimParams<sofa::helper::vector<float> > >()
        .add< OptimParams<Vec3fTypes::VecCoord> >()*/
        #endif
        ;

#ifndef SOFA_FLOAT
template class SOFA_OptimusPlugin_API OptimParams<double>;
/*template class SOFA_OptimusPlugin_API OptimParams<Vec3d>;
template class SOFA_OptimusPlugin_API OptimParams<Vec2d>;
template class SOFA_OptimusPlugin_API OptimParams<Vec1d>;
template class SOFA_OptimusPlugin_API OptimParams<RigidCoord<3,double> >;
template class SOFA_OptimusPlugin_API OptimParams<RigidCoord<2,double> >;*/
template class SOFA_OptimusPlugin_API OptimParams<sofa::helper::vector<double> >;
//template class SOFA_OptimusPlugin_API OptimParams<Vec3dTypes::VecCoord>;
#endif
#ifndef SOFA_DOUBLE
/*template class SOFA_OptimusPlugin_API OptimParams<float>;
template class SOFA_OptimusPlugin_API OptimParams<Vec3f>;
template class SOFA_OptimusPlugin_API OptimParams<Vec2f>;
template class SOFA_OptimusPlugin_API OptimParams<Vec1f>;
template class SOFA_OptimusPlugin_API OptimParams<RigidCoord<3,float> >;
template class SOFA_OptimusPlugin_API OptimParams<RigidCoord<2,float> >;
template class SOFA_OptimusPlugin_API OptimParams<sofa::helper::vector<float> >;
template class SOFA_OptimusPlugin_API OptimParams<Vec3fTypes::VecCoord>;*/
#endif


} // namespace container
} // namespace component
} // namespace sofa




