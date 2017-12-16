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

#include <fstream>
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
void OptimParams<sofa::helper::vector<double> >::getInitVariance(DVec& _variance) {
    _variance.resize(m_stdev.getValue().size());

    switch (transParamType) {
    case 3:
        for (size_t i = 0; i < _variance.size(); i++)
            _variance[i] = log(SQR(m_stdev.getValue()[i]));
        break;
    default:
        for (size_t i = 0; i < _variance.size(); i++)
            _variance[i] = SQR(m_stdev.getValue()[i]);

    }

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
void OptimParams<sofa::helper::vector<double> >::vectorToParams(VectorXd& _vector) {    
    helper::WriteAccessor<Data<helper::vector<double> > > val = m_val;

    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=fabs(_vector[this->paramIndices[i]]);
        }
        break;
    case 2:
        // std::cout << "backward step " << std::endl;
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=sigmoid(_vector[this->paramIndices[i]], m_max.getValue()[i], m_min.getValue()[i]);
            //std::cout << val[i] << std::endl;
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=exp(_vector[this->paramIndices[i]]);
            //std::cout << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < this->paramIndices.size(); i++)
            val[i] = _vector[this->paramIndices[i]];
    }
}

template<>
void OptimParams<sofa::helper::vector<double> >::vectorToParams(VectorXf& _vector) {    
    helper::WriteAccessor<Data<helper::vector<double> > > val = m_val;

    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=double(fabs(_vector[this->paramIndices[i]]));
        }
        break;
    case 2:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=sigmoid(_vector[this->paramIndices[i]], m_max.getValue()[i], m_min.getValue()[i]);
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=exp(_vector[this->paramIndices[i]]);
            //std::cout << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < this->paramIndices.size(); i++)
            val[i] = double(_vector[this->paramIndices[i]]);
    }
}

template<>
void OptimParams<sofa::helper::vector<double> >::rawVectorToParams(const double* _vector) {
    helper::WriteAccessor<Data<helper::vector<double> > > val = m_val;

    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=fabs(_vector[this->paramIndices[i]]);
        }
        break;
    case 2:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=sigmoid(_vector[this->paramIndices[i]], m_max.getValue()[i], m_min.getValue()[i]);
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=exp(_vector[this->paramIndices[i]]);
            //std::cout << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < this->paramIndices.size(); i++)
            val[i] = _vector[this->paramIndices[i]];
    }

    //std::cout << this->getContext()->getName() << "::" << this->getName() << " params: " << val << std::endl;
}

template<>
void OptimParams<sofa::helper::vector<double> >::rawVectorToParamsParallel(const double* _vector)
{
    //std::cout<<"INFO: rawVectorToParamsParallel\n";
    //std::cout << this->getContext()->getName() << "::" << this->getName() << " paramIdx size: " << this->paramIndices << std::endl;
    helper::WriteAccessor<Data<helper::vector<double> > > val = m_val;
    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=fabs(_vector[this->paramIndices[i]]);
        }
        break;
    case 2:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=sigmoid(_vector[this->paramIndices[i]], m_max.getValue()[i], m_min.getValue()[i]);
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i]=exp(_vector[this->paramIndices[i]]);
            //std::cout << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < this->paramIndices.size(); i++)
            val[i] = _vector[this->paramIndices[i]];
    }
    //std::cout << this->getContext()->getName() << "::" << this->getName() << " params: " << val << std::endl;
}

template<>
void OptimParams<sofa::helper::vector<double> >::paramsToVector(VectorXd& _vector) {
    helper::ReadAccessor<Data<helper::vector<double> > > val = m_val; // real values of parameters

    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = double(fabs(val[i]));
        break;
    case 2:
        // std::cout << "forward step " << std::endl;
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = double(logit(val[i], m_max.getValue()[i], m_min.getValue()[i]));
            //std::cout << val[i] << " " << logit(val[i], m_max.getValue()[i], m_min.getValue()[i]) << std::endl;
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = double(log(val[i]));
            //std::cout << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = val[i];
    }
}

template<>
void OptimParams<sofa::helper::vector<double> >::paramsToVector(VectorXf& _vector) {
    helper::ReadAccessor<Data<helper::vector<double> > > val = m_val; // real values of parameters

    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = float(fabs(val[i]));
        break;
    case 2:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = float(logit(val[i], m_max.getValue()[i], m_min.getValue()[i]));
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = float(log(val[i]));
            //std::cout << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = float(val[i]);
    }
}


template<>
// puts the params at the known indices
void OptimParams<sofa::helper::vector<double> >::paramsToRawVector(double* _vector) {
    helper::ReadAccessor<Data<helper::vector<double> > > val = m_val; // real values of parameters

    //std::cout<<"INFO: paramsToRawVectorDouble\n";
    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = fabs(val[i]);
        break;
    case 2:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = logit(val[i], m_max.getValue()[i], m_min.getValue()[i]);
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = log(val[i]);
            //std::cout << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = val[i];
    }

}

template<>
void OptimParams<sofa::helper::vector<double> >::paramsToRawVectorParallel(double* _vector)
{    
    //std::cout<<"INFO:: paramsToRawVectorParallel\n";
    helper::ReadAccessor<Data<helper::vector<double> > > val = m_val;
    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = fabs(val[i]);
        break;
    case 2:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = logit(val[i], m_max.getValue()[i], m_min.getValue()[i]);
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = log(val[i]);
            //std::cout << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = val[i];
    }
}



template<>
void OptimParams<sofa::helper::vector<double> >::init() {    
    Inherit::init();
    m_dim = 1;

    /// take the initial value and initial stdev
    if (!this->m_prescribedParamKeys.getValue().empty()) {
        helper::ReadAccessor<Data<helper::vector<double> > >keys = m_prescribedParamKeys;

        size_t numParams = this->m_dim * this->m_numParams.getValue();
        size_t numKeyValues = keys.size();

        if ((numKeyValues % (numParams+1) ) != 0) {
            serr << this->getName() << " ERROR: wrong size of keys, should be N x " << numParams+1 << sendl;
        } else {
            size_t numKeys = numKeyValues / (numParams+1);
            std::cout << this->getName() << " found " << numKeys << " keys for prescribed parameters" << std::endl;

            m_paramKeys.clear();
            m_paramKeys.resize(numKeys);

            for (size_t i = 0; i < numKeys; i++) {
                m_paramKeys[i].first = keys[i*(numParams+1)];
                helper::vector<double> v(numParams);
                for (size_t j = 0; j < numParams; j++)
                    v[j] = keys[i*(numParams+1)+j+1];
                m_paramKeys[i].second = v;
            }

            /*for (size_t i = 0; i < m_paramKeys.size(); i++) {
                std::cout << "T = " << m_paramKeys[i].first;
                for (size_t j = 0; j < m_paramKeys[i].second.size(); j++)
                    std::cout << " " << m_paramKeys[i].second[j];
                std::cout << std::endl;
            }*/

            helper::WriteAccessor<Data<sofa::helper::vector<double> > > initVal = m_initVal;
            initVal.resize(numParams);
            for (size_t i = 0; i < numParams; i++)
                initVal[i] = m_paramKeys[0].second[i];
        }
    }

    helper::ReadAccessor<Data<sofa::helper::vector<double> > > initVal = m_initVal;
    size_t nInitVal = initVal.size();

    if ( (m_numParams.getValue() == 0 && nInitVal > 0) || m_numParams.getValue() == nInitVal) {
        sout << this->getName() << ": setting parameter number according to init. value vector: " << nInitVal << sendl;
        m_numParams.setValue(nInitVal);
    } else {
        if (m_numParams.getValue() > 0 && nInitVal == 1) {
            nInitVal = m_numParams.getValue();
            sout << this->getName() << ": copying initial value to " << nInitVal << " positions" << sendl;
            double value = initVal[0];

            sout << this->getName() << ": Resizing init value vector to " << nInitVal << sendl;
            helper::WriteAccessor<Data<sofa::helper::vector<double> > > wInitVal = m_initVal;
            wInitVal.wref().resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                wInitVal[i] = value;
        } else {
            serr << this->getName() << ": Incompatible input: numParams: " << m_numParams.getValue() << " |initVal| = " << nInitVal << sendl;
            return;
        }
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
        if (stdev.size() == 0) {
            stdev.resize(nInitVal, 0.0);
            PRNW("Standard deviation not used, setting to 0!")
        } else if (stdev.size() != nInitVal) {
            PRNW("|stdev| != |init value|, resizing and taking the first member of stdev. ")
            stdev.resize(nInitVal);
            for (size_t i = 1; i < nInitVal; i++)
                stdev[i] = stdev[0];
        }
    }

    this->saveParam = false;
    if (!m_exportParamFile.getValue().empty()) {
        std::ofstream paramFile(m_exportParamFile.getValue().c_str());
        if (paramFile.is_open()) {
            this->saveParam = true;
            paramFile.close();
        }
    }
}

template<>
void OptimParams<sofa::helper::vector<double> >::handleEvent(core::objectmodel::Event *event) {
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        //if (!this->m_optimize.getValue()) {
            double actTime = this->getTime() + this->getContext()->getDt(); /// the time has not been increased yet
            std::cout << this->getName() << " begin event at time: " << actTime << std::endl;

            int timeSlot = -1;

            for (size_t i = 1; i < m_paramKeys.size() && timeSlot < 0; i++) {
                if (m_paramKeys[i-1].first <= actTime && actTime < m_paramKeys[i].first) {
                    timeSlot = i-1;
                }
            }           

            if (timeSlot == -1) {
                if (actTime >= m_paramKeys.back().first) {
                    helper::WriteAccessor<Data<sofa::helper::vector<double> > > val = m_val;
                    std::cout << "Const val: ";
                    for (size_t i = 0; i < val.size(); i++) {
                        val[i] = m_paramKeys.back().second[i];
                        std::cout << " " << val[i];
                    }
                    std::cout << std::endl;
                } else {
                    std::cerr << this->getName() << " ERROR: no slot found for time " << actTime << std::endl;
                }
            } else {
                double t1 = m_paramKeys[timeSlot].first;
                double t2 = m_paramKeys[timeSlot+1].first;
                double r1 = (actTime-t1)/(t2-t1);
                double r2 = (t2-actTime)/(t2-t1);

                //std::cout << "Time slot: " << timeSlot << " actual time: " << actTime << " ratii: " << r1 << " " << r2 << std::endl;

                helper::WriteAccessor<Data<sofa::helper::vector<double> > > val = m_val;
                std::cout << "Value: ";
                for (size_t i = 0; i < val.size(); i++) {
                    double v1=m_paramKeys[timeSlot].second[i];
                    double v2=m_paramKeys[timeSlot+1].second[i];

                    /// (y2-y1)*(tanh(3.5*(a-(t0+t1)/2))+1)/2+y1
                    if (this->m_interpolateSmooth.getValue())
                        val[i] =  (v2-v1)*(tanh(2*(actTime-(t1+t2)/2))+1)/2+v1;
                    else
                        val[i] = r2*v1 + r1*v2;
                    std::cout << " " << val[i];
                }
                std::cout << std::endl;

            //}

        }

        if (this->saveParam) {
            std::ofstream paramFile(m_exportParamFile.getValue().c_str(), std::ios::app);
            if (paramFile.is_open()) {
                helper::ReadAccessor<Data<sofa::helper::vector<double> > > val = m_val;
                for (size_t i = 0; i < val.size(); i++)
                    paramFile << val[i] << " ";
                paramFile << '\n';
                paramFile.close();
            }
        }

    }
}


/// SPECIALIZATIONS FOR VecCoord3D

template<>
void OptimParams<Vec3dTypes::VecCoord>::init() {
    Inherit1::init();
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
    //std::cout<<"INFO: paramsToRawVectorVec3d\n";
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
void OptimParams<Vec3dTypes::VecCoord>::getInitVariance(DVec& _variance) {
    size_t numParams = this->m_numParams.getValue();
    _variance.resize(numParams*m_dim);
    size_t ij = 0;
    for (size_t i = 0; i < numParams; i++)
        for (size_t j = 0; j < m_dim; j++, ij++)
            _variance[ij] = SQR(m_stdev.getValue()[i][j]);
}

template<>
void OptimParams<Vec3dTypes::VecCoord>::handleEvent(core::objectmodel::Event *event) {
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        //if (!this->m_optimize.getValue()) {
            double actTime = this->getTime() + this->getContext()->getDt(); /// the time has not been increased yet
            std::cout << "[" << this->getName() << "] begin event at time: " << actTime << std::endl;

            int timeSlot = -1;

            for (size_t i = 1; i < m_paramKeys.size() && timeSlot < 0; i++) {
                if (m_paramKeys[i-1].first <= actTime && actTime < m_paramKeys[i].first) {
                    timeSlot = i-1;
                }
            }
            std::cout << "HERE" << std::endl;
            if (timeSlot == -1) {
                if (actTime >= m_paramKeys.back().first) {
                    helper::WriteAccessor<Data<Vec3dTypes::VecCoord> > val = m_val;
                    std::cout << "["  << this->getName() << "] const val: ";
                    for (size_t i = 0; i < val.size(); i++) {
                        val[i] = m_paramKeys.back().second[i];
                        std::cout << " " << val[i];
                    }
                    std::cout << std::endl;
                } else {
                    std::cerr << this->getName() << " ERROR: no slot found for time " << actTime << std::endl;
                }
            } else {
                double t1 = m_paramKeys[timeSlot].first;
                double t2 = m_paramKeys[timeSlot+1].first;
                double r1 = (actTime-t1)/(t2-t1);
                double r2 = (t2-actTime)/(t2-t1);

                //std::cout << "Time slot: " << timeSlot << " actual time: " << actTime << " ratii: " << r1 << " " << r2 << std::endl;

                helper::WriteAccessor<Data<Vec3dTypes::VecCoord > > val = m_val;
                std::cout << "[" << this->getName() << "] Value: ";
                for (size_t i = 0; i < val.size(); i++) {
                    Vec3dTypes::Coord v1=m_paramKeys[timeSlot].second[i];
                    Vec3dTypes::Coord v2=m_paramKeys[timeSlot+1].second[i];

                    /// (y2-y1)*(tanh(3.5*(a-(t0+t1)/2))+1)/2+y1
                    if (this->m_interpolateSmooth.getValue())
                        val[i] =  (v2-v1)*(tanh(2*(actTime-(t1+t2)/2))+1)/2+v1;
                    else
                        val[i] = r2*v1 + r1*v2;
                    std::cout << " " << val[i];
                }
                std::cout << std::endl;

            }

        //}

        if (this->saveParam) {
            std::ofstream paramFile(m_exportParamFile.getValue().c_str(), std::ios::app);
            if (paramFile.is_open()) {
                helper::ReadAccessor<Data<Vec3dTypes::VecCoord> > val = m_val;
                for (size_t i = 0; i < val.size(); i++)
                    paramFile << val[i] << " ";
                paramFile << '\n';
                paramFile.close();
            }
        }

    }
}


/// Specialization for RigidDeriv3d

template<>
void OptimParams<Rigid3dTypes::VecDeriv>::init() {
    Inherit1::init();
    paramMOrigid = m_paramMOLinkrigid.get();

    this->m_dim = 6;

    if (paramMOrigid == NULL)
        std::cerr << "WARNING: cannot find the parametric mechanical state, assuming no mechanical state is associated with the parameters" << std::endl;
    else {
        std::cout << "Mechanical state associated with the parameters: " << paramMOrigid->getName() << std::endl;

        int moSize = 6 ; /// TODO retrieve catheter nodes
        helper::WriteAccessor<Data<Rigid3dTypes::VecDeriv> > waInitVal = m_initVal;
        Rigid3dTypes::Deriv initForces = Rigid3dTypes::Deriv() ;
        waInitVal.resize(moSize);

        for (int i = 0; i < moSize; i++)
            waInitVal[i] = initForces;
    }
    helper::ReadAccessor<Data<Rigid3dTypes::VecDeriv> > raInitVal = m_initVal;
    helper::WriteAccessor<Data<Rigid3dTypes::VecDeriv> > waVal = m_val;
    int numParams = raInitVal.size();
    m_numParams.setValue(numParams);
    waVal.resize(numParams);
    for (int i = 0; i < numParams; i++)
        waVal[i] = raInitVal[i];

}


template<>
void OptimParams<Rigid3dTypes::VecDeriv>::vectorToParams(VectorXd& _vector) {
    helper::WriteAccessor<Data<Rigid3dTypes::VecDeriv> > waVal = m_val;

    size_t numParams = m_numParams.getValue();
    size_t k = 0;
    for (size_t i = 0; i < numParams; i++)
        for (size_t j = 0; j < 6; j++, k++)
            waVal[i][j] = _vector[paramIndices[k]];

//    if (paramMOrigid != NULL) {
//        typename MechStateRigid3d::ReadVecDeriv moWForces = paramMOrigid->writeForces();
//        helper::ReadAccessor<Data<Rigid3dTypes::VecDeriv> > raVal = m_val;

//        for (size_t i = 0; i < numParams; i++)
//            moWForces[i] = raVal[i];
//    }

}

template<>
void OptimParams<Rigid3dTypes::VecDeriv>::paramsToVector(VectorXd& _vector) {
    size_t numParams = m_numParams.getValue();
//    if (paramMOrigid != NULL) {
//        typename MechStateRigid3d::ReadVecDeriv moRForces = paramMOrigid->readForces();
//        helper::WriteAccessor<Data<Rigid3dTypes::VecDeriv> > waVal = m_val;


//        for (size_t i = 0; i < numParams; i++)
//            waVal[i] = moRForces[i];
//    }

    helper::ReadAccessor<Data<Rigid3dTypes::VecDeriv> > raVal = m_val;
    size_t k = 0;
    for (size_t i = 0; i < numParams; i++)
        for (size_t j = 0; j < 6; j++, k++)
            _vector[paramIndices[k]] = raVal[i][j];

}


template<>
void OptimParams<Rigid3dTypes::VecDeriv>::handleEvent(core::objectmodel::Event *event) {
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        //if (!this->m_optimize.getValue()) {
            double actTime = this->getTime() + this->getContext()->getDt(); /// the time has not been increased yet
            std::cout << "[" << this->getName() << "] begin event at time: " << actTime << std::endl;

            int timeSlot = -1;

            for (size_t i = 1; i < m_paramKeys.size() && timeSlot < 0; i++) {
                if (m_paramKeys[i-1].first <= actTime && actTime < m_paramKeys[i].first) {
                    timeSlot = i-1;
                }
            }
            std::cout << "HERE" << std::endl;
            if (timeSlot == -1) {
                if (actTime >= m_paramKeys.back().first) {
                    helper::WriteAccessor<Data<Rigid3dTypes::VecDeriv> > val = m_val;
                    std::cout << "["  << this->getName() << "] const val: ";
                    for (size_t i = 0; i < val.size(); i++) {
                        val[i] = m_paramKeys.back().second[i];
                        std::cout << " " << val[i];
                    }
                    std::cout << std::endl;
                } else {
                    std::cerr << this->getName() << " ERROR: no slot found for time " << actTime << std::endl;
                }
            } else {
                double t1 = m_paramKeys[timeSlot].first;
                double t2 = m_paramKeys[timeSlot+1].first;
                double r1 = (actTime-t1)/(t2-t1);
                double r2 = (t2-actTime)/(t2-t1);

                //std::cout << "Time slot: " << timeSlot << " actual time: " << actTime << " ratii: " << r1 << " " << r2 << std::endl;

                helper::WriteAccessor<Data<Rigid3dTypes::VecDeriv> > val = m_val;
                std::cout << "[" << this->getName() << "] Value: ";
                for (size_t i = 0; i < val.size(); i++) {
                    Rigid3dTypes::Deriv v1=m_paramKeys[timeSlot].second[i];
                    Rigid3dTypes::Deriv v2=m_paramKeys[timeSlot+1].second[i];

                    /// (y2-y1)*(tanh(3.5*(a-(t0+t1)/2))+1)/2+y1
                    if (this->m_interpolateSmooth.getValue())
                        val[i] =  (v2-v1)*(tanh(2*(actTime-(t1+t2)/2))+1)/2+v1;
//                    else
//                        val[i] = r2*v1 + r1*v2;
//                    std::cout << " " << val[i];
                }
                std::cout << std::endl;

            }

        //}

        if (this->saveParam) {
            std::ofstream paramFile(m_exportParamFile.getValue().c_str(), std::ios::app);
            if (paramFile.is_open()) {
                helper::ReadAccessor<Data<Rigid3dTypes::VecDeriv> > val = m_val;
                for (size_t i = 0; i < val.size(); i++)
                    paramFile << val[i] << " ";
                paramFile << '\n';
                paramFile.close();
            }
        }

    }
}


/// Specialization for double

template<>
void OptimParams<double>::init() {
    Inherit::init();
    m_dim = 1;

    /// take the initial value and initial stdev
    if (!this->m_prescribedParamKeys.getValue().empty()) {
        helper::ReadAccessor<Data<helper::vector<double> > >keys = m_prescribedParamKeys;

        size_t numKeyValues = keys.size();

        if ((numKeyValues % 2 ) != 0) {
            serr << this->getName() << " ERROR: wrong size of keys, should be N x " << 2 << sendl;
        } else {
            size_t numKeys = numKeyValues / 2;
            std::cout << this->getName() << " found " << numKeys << " keys for prescribed parameters" << std::endl;

            m_paramKeys.clear();
            m_paramKeys.resize(numKeys);

            for (size_t i = 0; i < numKeys; i++) {
                m_paramKeys[i].first = keys[2*i];
                m_paramKeys[i].second = keys[2*i+1];
            }

            /*for (size_t i = 0; i < m_paramKeys.size(); i++) {
                std::cout << "T = " << m_paramKeys[i].first;
                for (size_t j = 0; j < m_paramKeys[i].second.size(); j++)
                    std::cout << " " << m_paramKeys[i].second[j];
                std::cout << std::endl;
            }*/

            m_initVal.setValue(m_paramKeys[0].second);
        }
    }
}


template<>
void OptimParams<double>::handleEvent(core::objectmodel::Event *event) {
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        //if (!this->m_optimize.getValue()) {
            double actTime = this->getTime() + this->getContext()->getDt(); /// the time has not been increased yet
            std::cout << "[" << this->getName() << "] begin event at time: " << actTime << std::endl;

            int timeSlot = -1;

            std::cout << "Keys size: " << m_paramKeys.size() << std::endl;
            for (size_t i = 1; i < m_paramKeys.size() && timeSlot < 0; i++) {
                if (m_paramKeys[i-1].first <= actTime && actTime < m_paramKeys[i].first) {
                    timeSlot = i-1;
                }
            }
            if (timeSlot == -1) {
                if (actTime >= m_paramKeys.back().first) {
                    //helper::WriteAccessor<Data<double> > val = m_val;
                    double val = m_paramKeys.back().second;
                    std::cout << "["  << this->getName() << "] const val: " << val << std::endl;
                    m_val.setValue(val);
                } else {
                    std::cerr << this->getName() << " ERROR: no slot found for time " << actTime << std::endl;
                }
            } else {
                double t1 = m_paramKeys[timeSlot].first;
                double t2 = m_paramKeys[timeSlot+1].first;
                double r1 = (actTime-t1)/(t2-t1);
                double r2 = (t2-actTime)/(t2-t1);

                //std::cout << "Time slot: " << timeSlot << " actual time: " << actTime << " ratii: " << r1 << " " << r2 << std::endl;

                //helper::WriteAccessor<Data<double > > val = m_val;
                double val;
                double v1=m_paramKeys[timeSlot].second;
                double v2=m_paramKeys[timeSlot+1].second;

                /// (y2-y1)*(tanh(3.5*(a-(t0+t1)/2))+1)/2+y1
                if (this->m_interpolateSmooth.getValue())
                    val =  (v2-v1)*(tanh(2*(actTime-(t1+t2)/2))+1)/2+v1;
                else
                    val = r2*v1 + r1*v2;
                std::cout << "[" << this->getName() << "] Value: " << val << std::endl;
                m_val.setValue(val);
            }
    }
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
        .add< OptimParams<double> >()
        .add< OptimParams<Vec3d> >()
                                        /*.add< OptimParams<Vec2d> >()
                                        .add< OptimParams<Vec1d> >()
                                        .add< OptimParams<RigidCoord<3,double> > >()
                                        .add< OptimParams<RigidCoord<2,double> > >()*/
        .add< OptimParams<sofa::helper::vector<double> > >()
        .add< OptimParams<Vec3dTypes::VecCoord> >()
        .add< OptimParams<Rigid3dTypes::VecDeriv> >()

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
template class SOFA_OptimusPlugin_API OptimParams<Vec3d>;
/*template class SOFA_OptimusPlugin_API OptimParams<Vec2d>;
template class SOFA_OptimusPlugin_API OptimParams<Vec1d>;
template class SOFA_OptimusPlugin_API OptimParams<RigidCoord<3,double> >;
template class SOFA_OptimusPlugin_API OptimParams<RigidCoord<2,double> >;*/
template class SOFA_OptimusPlugin_API OptimParams<sofa::helper::vector<double> >;
//template class SOFA_OptimusPlugin_API OptimParams<Vec3dTypes::VecCoord>;
template class SOFA_OptimusPlugin_API OptimParams<Rigid3dTypes::VecDeriv>;
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




