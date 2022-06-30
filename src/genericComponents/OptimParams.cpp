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

//#define SOFA_COMPONENT_CONTAINER_OPTIMPARAMS_CPP
#include "../optimusConfig.h"

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
void OptimParams<sofa::type::vector<double> >::getInitVariance(DVec& _variance)
{
    _variance.resize(m_stdev.getValue().size());

    switch (transParamType) {
    case 3:
        for (size_t i = 0; i < _variance.size(); i++)
            _variance[i] = helper::SQR(log(m_stdev.getValue()[i]));
        break;
    default:
        for (size_t i = 0; i < _variance.size(); i++)
            _variance[i] = helper::SQR(m_stdev.getValue()[i]);
    }
}



template<>
void OptimParams<sofa::type::vector<double> >::vectorToParams(VectorXd& _vector)
{
    helper::WriteAccessor< Data< type::vector<double> > > val = m_val;
    helper::ReadAccessor< Data< type::vector<double> > > minVal = m_minVal;
    helper::ReadAccessor< Data< type::vector<double> > > maxVal = m_maxVal;

    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i] = fabs(_vector[this->paramIndices[i]]);
        }
        break;
    case 2:
        // std::cout << "backward step " << std::endl;
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i] = sigmoid(_vector[this->paramIndices[i]], maxVal[i], minVal[i]);
            // std::cout << val[i] << std::endl;
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i] = exp(_vector[this->paramIndices[i]]);
            // std::cout << val[i] << std::endl;
        }
        break;
    case 4:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            double inVal = _vector[this->paramIndices[i]];
            // std::cout << "HERE: " << inVal << " vs " << minVal[i] << " " <<(inVal < minVal[i]) << std::endl;
            val[i] = (inVal < minVal[i]) ? minVal[i] : inVal;
            val[i] = (val[i] > maxVal[i]) ? maxVal[i] : val[i];
            // std::cout <<  "  val: " << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < this->paramIndices.size(); i++)
            val[i] = _vector[this->paramIndices[i]];
    }

    //  std::cout <<"Values used in SOFA:" << std::endl;
    //  for (size_t i = 0; i < this->paramIndices.size(); i++) {
    //      std::cout <<  " " << val[i];
    //  }
    //  std::cout << std::endl;
}



template<>
void OptimParams<sofa::type::vector<double> >::vectorToParams(VectorXf& _vector)
{
    helper::WriteAccessor< Data< type::vector<double> > > val = m_val;
    helper::ReadAccessor< Data< type::vector<double> > > minVal = m_minVal;
    helper::ReadAccessor< Data< type::vector<double> > > maxVal = m_maxVal;

    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i] = double(fabs(_vector[this->paramIndices[i]]));
        }
        break;
    case 2:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i] = sigmoid(_vector[this->paramIndices[i]], maxVal[i], minVal[i]);
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            val[i] = exp(_vector[this->paramIndices[i]]);
            // std::cout << val[i] << std::endl;
        }
        break;
    case 4:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            double inVal = _vector[this->paramIndices[i]];
            val[i] = (inVal < minVal[i]) ? minVal[i] : inVal;
            val[i] = (val[i] > maxVal[i]) ? maxVal[i] : val[i];
        }
        break;
    default:
        for (size_t i = 0; i < this->paramIndices.size(); i++)
            val[i] = double(_vector[this->paramIndices[i]]);
    }
}



template<>
void OptimParams<sofa::type::vector<double> >::paramsToVector(VectorXd& _vector)
{
    helper::ReadAccessor< Data< type::vector<double> > > val = m_val; // real values of parameters

    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = double(fabs(val[i]));
        break;
    case 2:
        // std::cout << "forward step " << std::endl;
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = double(logit(val[i], m_maxVal.getValue()[i], m_minVal.getValue()[i]));
            // std::cout << val[i] << " " << logit(val[i], m_maxVal.getValue()[i], m_minVal.getValue()[i]) << std::endl;
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = double(log(val[i]));
            // std::cout << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = val[i];
    }
    // std::cout << "paramsToVector raVal\n" << _vector << std::endl;
}



template<>
void OptimParams<sofa::type::vector<double> >::paramsToVector(VectorXf& _vector)
{
    helper::ReadAccessor< Data< type::vector<double> > > val = m_val; // real values of parameters

    switch (transParamType) {
    case 1:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = float(fabs(val[i]));
        break;
    case 2:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = float(logit(val[i], m_maxVal.getValue()[i], m_minVal.getValue()[i]));
        }
        break;
    case 3:
        for (size_t i = 0; i < this->paramIndices.size(); i++) {
            _vector[paramIndices[i]] = float(log(val[i]));
            // std::cout << val[i] << std::endl;
        }
        break;
    default:
        for (size_t i = 0; i < paramIndices.size(); i++)
            _vector[paramIndices[i]] = float(val[i]);
    }
}



template<>
void OptimParams<sofa::type::vector<double> >::init()
{
    Inherit::init();
    m_dim = 1;

    /// take the initial value and initial stdev
    if ( !this->m_prescribedParamKeys.getValue().empty() )
    {
        helper::ReadAccessor< Data< type::vector<double> > >keys = m_prescribedParamKeys;

        size_t numParams = this->m_dim * this->m_numParams.getValue();
        size_t numKeyValues = keys.size();

        if ((numKeyValues % (numParams+1) ) != 0) {
            PRNE(this->getName() << " ERROR: wrong size of keys, should be N x " << numParams+1);
        } else {
            size_t numKeys = numKeyValues / (numParams+1);
            PRNS(this->getName() << " found " << numKeys << " keys for prescribed parameters");

            m_paramKeys.clear();
            m_paramKeys.resize(numKeys);

            for (size_t i = 0; i < numKeys; i++) {
                m_paramKeys[i].first = keys[i*(numParams+1)];
                type::vector<double> v(numParams);
                for (size_t j = 0; j < numParams; j++)
                    v[j] = keys[i * (numParams + 1) + j + 1];
                m_paramKeys[i].second = v;
            }

            /*for (size_t i = 0; i < m_paramKeys.size(); i++) {
                std::cout << "T = " << m_paramKeys[i].first;
                for (size_t j = 0; j < m_paramKeys[i].second.size(); j++)
                    std::cout << " " << m_paramKeys[i].second[j];
                std::cout << std::endl;
            }*/

            helper::WriteAccessor< Data< sofa::type::vector<double> > > initVal = m_initVal;
            initVal.resize(numParams);
            for (size_t i = 0; i < numParams; i++)
                initVal[i] = m_paramKeys[0].second[i];
        }
    }

    helper::ReadAccessor<Data<sofa::type::vector<double> > > initVal = m_initVal;
    size_t nInitVal = initVal.size();

    if ( (m_numParams.getValue() == 0 && nInitVal > 0) || m_numParams.getValue() == nInitVal) {
        PRNS(this->getName() << ": setting parameter number according to init. value vector: " << nInitVal);
        m_numParams.setValue(nInitVal);
    } else {
        if (m_numParams.getValue() > 0 && nInitVal == 1) {
            nInitVal = m_numParams.getValue();
            PRNS(this->getName() << ": copying initial value to " << nInitVal << " positions");
            double value = initVal[0];

            PRNS(this->getName() << ": Resizing init value vector to " << nInitVal);
            helper::WriteAccessor<Data<sofa::type::vector<double> > > wInitVal = m_initVal;
            wInitVal.wref().resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                wInitVal[i] = value;
        } else {
            PRNE(this->getName() << ": Incompatible input: numParams: " << m_numParams.getValue() << " |initVal| = " << nInitVal);
            return;
        }
    }

    size_t numParams = m_numParams.getValue();
    if (numParams > 1 && m_maxVal.getValue().size() == 1) {
        const double val = m_maxVal.getValue()[0];
        helper::WriteAccessor<Data<sofa::type::vector<double> > > wMaxVal = m_maxVal;
        wMaxVal.resize(numParams, val);
        for (size_t i = 0; i < numParams; i++)
            wMaxVal[i] = val;
    }

    if (numParams > 1 && m_minVal.getValue().size() == 1) {
        const double val = m_minVal.getValue()[0];
        helper::WriteAccessor<Data<sofa::type::vector<double> > > wMinVal = m_minVal;
        wMinVal.resize(numParams, val);
        for (size_t i = 0; i < numParams; i++)
            wMinVal[i] = val;
    }

    if (nInitVal != 0) {
        helper::WriteAccessor<Data<sofa::type::vector<double> > > val = m_val;
        if (val.size() == 0) {
            val.resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                val[i] = initVal[i];
        }

        helper::WriteAccessor<Data<sofa::type::vector<double> > > minVal = m_minVal;
        if (minVal.size() == 0) {
            minVal.resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                minVal[i] = initVal[i];
        }

        helper::WriteAccessor<Data<sofa::type::vector<double> > > maxVal = m_maxVal;
        if (maxVal.size() == 0) {
            maxVal.resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                maxVal[i] = initVal[i];
        }

        helper::WriteAccessor<Data<sofa::type::vector<double> > > stdev = m_stdev;
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
void OptimParams<type::vector<double> >::appendParameters()
{
    size_t numParams = this->m_dim * this->m_numParams.getValue();
    std::cout << "Old number of parameters: " << numParams << std::endl;

    // read added parameters
    helper::ReadAccessor< Data< sofa::type::vector<double> > > addedVal = m_addedVal;
    helper::ReadAccessor< Data< sofa::type::vector<double> > > addedMinVal = m_addedMinVal;
    helper::ReadAccessor< Data< sofa::type::vector<double> > > addedMaxVal = m_addedMaxVal;
    helper::ReadAccessor< Data< sofa::type::vector<double> > > addedStd = m_addedStd;

    size_t newNumParams = addedVal.size() + this->m_dim * this->m_numParams.getValue();
    std::cout << "Added values size: " << addedVal.size() << std::endl;
    std::cout << "New number of parameters: " << newNumParams << std::endl;

    helper::WriteAccessor<Data<sofa::type::vector<double> > > val = m_val;
    val.resize(newNumParams);
    for (size_t i = numParams; i < newNumParams; i++)
        val[i] = addedVal[i - numParams];

    helper::WriteAccessor<Data<sofa::type::vector<double> > > initVal = m_initVal;
    initVal.resize(newNumParams);
    for (size_t i = numParams; i < newNumParams; i++)
        initVal[i] = addedVal[i - numParams];

    helper::WriteAccessor<Data<sofa::type::vector<double> > > minVal = m_minVal;
    minVal.resize(newNumParams);
    for (size_t i = numParams; i < newNumParams; i++)
        minVal[i] = addedMinVal[i - numParams];

    helper::WriteAccessor<Data<sofa::type::vector<double> > > maxVal = m_maxVal;
    maxVal.resize(newNumParams);
    for (size_t i = numParams; i < newNumParams; i++)
        maxVal[i] = addedMaxVal[i - numParams];

    helper::WriteAccessor<Data<sofa::type::vector<double> > > stdev = m_stdev;
    stdev.resize(newNumParams);
    for (size_t i = numParams; i < newNumParams; i++)
        stdev[i] = addedStd[i - numParams];

    m_numParams.setValue(newNumParams);
}



template<>
void OptimParams<double>::appendParameters() { }



template<>
void OptimParams<type::Vec3d>::appendParameters() { }



template<>
void OptimParams<Vec3dTypes::VecDeriv>::appendParameters() { }



template<>
void OptimParams<Rigid3dTypes::VecDeriv>::appendParameters() { }



template<>
void OptimParams<sofa::type::vector<double> >::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        //if (!this->m_optimize.getValue()) {
            double actTime = this->getTime() + this->getContext()->getDt(); /// the time has not been increased yet
            std::cout << this->getName() << " begin event at time: " << actTime << std::endl;

            int timeSlot = -1;

            for (size_t i = 1; i < m_paramKeys.size() && timeSlot < 0; i++) {
                if (m_paramKeys[i - 1].first <= actTime && actTime < m_paramKeys[i].first) {
                    timeSlot = i - 1;
                }
            }           

            if (timeSlot == -1) {
                if (actTime >= m_paramKeys.back().first) {
                    helper::WriteAccessor<Data<sofa::type::vector<double> > > val = m_val;
                    std::cout << "Const val: ";
                    for (size_t i = 0; i < val.size(); i++) {
                        val[i] = m_paramKeys.back().second[i];
                        std::cout << " " << val[i];
                    }
                    std::cout << std::endl;
                } else {
                    PRNE(this->getName() << " ERROR: no slot found for time " << actTime);
                }
            } else {
                double t1 = m_paramKeys[timeSlot].first;
                double t2 = m_paramKeys[timeSlot+1].first;
                double r1 = (actTime - t1) / (t2 - t1);
                double r2 = (t2 - actTime) / (t2 - t1);

                // std::cout << "Time slot: " << timeSlot << " actual time: " << actTime << " ratii: " << r1 << " " << r2 << std::endl;

                helper::WriteAccessor<Data<sofa::type::vector<double> > > val = m_val;
                std::cout << "Value: ";
                for (size_t i = 0; i < val.size(); i++) {
                    double v1 = m_paramKeys[timeSlot].second[i];
                    double v2 = m_paramKeys[timeSlot + 1].second[i];

                    // (y2 - y1) * (tanh(3.5 * (a - (t0 + t1) / 2)) + 1) / 2 + y1
                    if (this->m_interpolateSmooth.getValue()) {
                        val[i] = (v2 - v1) * (tanh(2 * (actTime - (t1 + t2) / 2)) + 1) / 2 + v1;
                    } else {
                        val[i] = r2 * v1 + r1 * v2;
                    }
                    std::cout << " " << val[i];
                }
                std::cout << std::endl;
            }
        // }

        if (this->saveParam) {
            std::ofstream paramFile(m_exportParamFile.getValue().c_str(), std::ios::app);
            if (paramFile.is_open()) {
                helper::ReadAccessor< Data< sofa::type::vector<double> > > val = m_val;
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
void OptimParams<Vec3dTypes::VecDeriv>::init()
{
    Inherit::init();
    this->m_dim = 3;

    size_t numParams = m_numParams.getValue();
    if ( numParams == 0 ) {
        PRNE("No parameter to optimize! NumParams must be non-zero!");
    }
    helper::WriteAccessor<Data<Vec3dTypes::VecDeriv> > val = m_val;
    val.resize(numParams);

    helper::ReadAccessor<Data<Vec3dTypes::VecDeriv> >initVal = m_initVal;
    size_t nInitVal = initVal.size();
    if (nInitVal == 1) {
        for (size_t i = 0; i < numParams; i++)
            val[i] = initVal[0];
    } else {
        if (nInitVal != numParams) {
            PRNE("Size of initial value vector different from number of parameters!");
        }

        for (size_t i = 0; i < numParams; i++)
            val[i] = initVal[i];
    }

    helper::ReadAccessor<Data<Vec3dTypes::VecDeriv> > stdev = m_stdev;
    if (stdev.size() == 1) {
        Vec3dTypes::Deriv stdev0 = stdev[0];
        helper::WriteAccessor<Data<Vec3dTypes::VecDeriv> > stdev_wa = m_stdev;
        stdev_wa.resize(numParams);
        for (size_t i = 0; i < numParams; i++) {
            stdev_wa[i] = stdev0;
        }
    } else {
        if (stdev.size() != numParams) {
            PRNE("Size of standard deviation vector different from number of parameters!");
        }
    }    
}



template<>
void OptimParams<Vec3dTypes::VecDeriv>::vectorToParams(VectorXd& _vector)
{
    helper::WriteAccessor<Data<Vec3dTypes::VecDeriv> > waVal = m_val;

    size_t numParams = m_numParams.getValue();
    size_t k = 0;
    for (size_t i = 0; i < numParams; i++)
        for (size_t j = 0; j < this->m_dim ; j++, k++)
            waVal[i][j] = _vector[paramIndices[i*m_dim+j]];
}



template<>
void OptimParams<Vec3dTypes::VecDeriv>::paramsToVector(VectorXd& _vector)
{
    helper::ReadAccessor<Data<Vec3dTypes::VecCoord> > raVal = m_val;
    size_t k = 0;
    size_t numParams = this->m_numParams.getValue();

    for (size_t i = 0; i <  numParams ; i++)
        for (size_t j = 0; j < m_dim ; j++, k++)
            _vector[paramIndices[i*m_dim+j]]=raVal[i][j];
}



template<>
void OptimParams<Vec3dTypes::VecDeriv>::getInitVariance(DVec& _variance)
{
    size_t numParams = this->m_numParams.getValue();
    _variance.resize(numParams*m_dim);
    size_t ij = 0;
    for (size_t i = 0; i <  numParams ; i++)
        for (size_t j = 0; j < m_dim; j++, ij++) {
            _variance[ij] = helper::SQR(m_stdev.getValue()[i][j]);            
        }

}



template<>
void OptimParams<Vec3dTypes::VecDeriv>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        double actTime = this->getTime() + this->getContext()->getDt(); /// the time has not been increased yet
        std::cout << "[" << this->getName() << "] begin event at time: " << actTime << std::endl;

        int timeSlot = -1;

        for (size_t i = 1; i < m_paramKeys.size() && timeSlot < 0; i++) {
            if (m_paramKeys[i-1].first <= actTime && actTime < m_paramKeys[i].first) {
                timeSlot = i - 1;
            }
        }
        if (timeSlot == -1) {
            if (actTime >= m_paramKeys.back().first) {
                helper::WriteAccessor<Data<Vec3dTypes::VecDeriv> > val = m_val;
                std::cout << "["  << this->getName() << "] const val: ";
                for (size_t i = 0; i < val.size(); i++) {
                    val[i] = m_paramKeys.back().second[i];
                    std::cout << " " << val[i];
                }
                std::cout << std::endl;
            } else {
                PRNE(this->getName() << " ERROR: no slot found for time " << actTime);
            }
        } else {
            double t1 = m_paramKeys[timeSlot].first;
            double t2 = m_paramKeys[timeSlot + 1].first;
            double r1 = (actTime - t1) / (t2 - t1);
            double r2 = (t2 - actTime) / (t2 - t1);

            helper::WriteAccessor< Data< Vec3dTypes::VecDeriv > > val = m_val;
            std::cout << "[" << this->getName() << "] Value: ";
            for (size_t i = 0; i < val.size(); i++) {
                Vec3dTypes::Coord v1 = m_paramKeys[timeSlot].second[i];
                Vec3dTypes::Coord v2 = m_paramKeys[timeSlot+1].second[i];

                // (y2 - y1) * (tanh(3.5 * (a - (t0 + t1) / 2)) + 1) / 2 + y1
                if (this->m_interpolateSmooth.getValue()) {
                    val[i] = (v2 - v1) * (tanh(2 * (actTime - (t1 + t2) / 2)) + 1) / 2 + v1;
                } else {
                    val[i] = r2 * v1 + r1 * v2;
                }
                std::cout << " " << val[i];
            }
            std::cout << std::endl;
        }

        if (this->saveParam) {
            std::ofstream paramFile(m_exportParamFile.getValue().c_str(), std::ios::app);
            if (paramFile.is_open()) {
                helper::ReadAccessor<Data<Vec3dTypes::VecDeriv> > val = m_val;
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
void OptimParams<Rigid3dTypes::VecDeriv>::init()
{
    Inherit1::init();
    paramMOrigid = m_paramMOLinkrigid.get();

    if (paramMOrigid == NULL) {
        PRNE("WARNING: cannot find the parametric mechanical state, assuming no mechanical state is associated with the parameters");
    } else {
        PRNS("Mechanical state associated with the parameters: " << paramMOrigid->getName());
        typename MechStateRigid3d::ReadVecCoord moPos = paramMOrigid->readPositions();
        mstate_dim = moPos.size();
    }
    helper::ReadAccessor<Data<Rigid3dTypes::VecDeriv> >initVal = m_initVal;
    size_t nInitVal = initVal.size();

    if (nInitVal != mstate_dim ){
        PRNE("Wrong values of parameter \"initValue\" in OptimParams. \nRESET it consistent with dimension of "<< paramMOrigid->getName())
        return;
    }

    if ( (m_numParams.getValue() == 0 && nInitVal > 0) || m_numParams.getValue() == nInitVal*6) {
         PRNS(this->getName() << ": setting parameter number according to init. value vector: " << nInitVal*6);
         m_numParams.setValue(nInitVal*6);
    } else {
        if (m_numParams.getValue() > 0 && nInitVal == mstate_dim) {

        } else {
            PRNE(this->getName() << ": Incompatible input: numParams: " << m_numParams.getValue() << " wth value = " << nInitVal*3);
            return;
        }
    }
    if (nInitVal != 0 || nInitVal == mstate_dim) {
        helper::WriteAccessor<Data<Rigid3dTypes::VecDeriv> > val = m_val;

        if (val.size() == 0) {
            val.resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                val[i] = initVal[i];
        }

        helper::WriteAccessor<Data<Rigid3dTypes::VecDeriv> > stdev = m_stdev;
        if (stdev.size() == 0) {
            stdev.resize(nInitVal);
            PRNW("Standard deviation not used, setting to 0!")
        } else if (stdev.size() != nInitVal) {
            PRNW("|stdev| != |init value|, resizing and taking the first member of stdev. ")
                    stdev.resize(nInitVal);
            for (size_t i = 1; i < nInitVal; i++)
                stdev[i] = stdev[0];
        }
    } else {
         PRNE("Wrong initValue. Must be consistent with "<< paramMOrigid->getName() << "dimensions. ")
    }
}



template<>
void OptimParams<Rigid3dTypes::VecDeriv>::vectorToParams(VectorXd& _vector)
{
    helper::WriteAccessor<Data<Rigid3dTypes::VecDeriv> > waVal = m_val;

    size_t numParams = mstate_dim;
    size_t k = 0;
    for (size_t i = 0; i < numParams; i++)
        for (size_t j = 0;  k < this->paramIndices.size(); j++, k++)
            waVal[i][j] = _vector[paramIndices[k]];
}



template<>
void OptimParams<Rigid3dTypes::VecDeriv>::paramsToVector(VectorXd& _vector)
{
    helper::ReadAccessor<Data<Rigid3dTypes::VecDeriv> > raVal = m_val;
    size_t k = 0;

    for (size_t i = 0; i < mstate_dim; i++)
        for (size_t j = 0; k < this->paramIndices.size() ; j++, k++)
            _vector[paramIndices[k]]=raVal[i][j];
}



template<>
void OptimParams<Rigid3dTypes::VecDeriv>::getInitVariance(DVec& _variance)
{
    size_t numParams = this->m_numParams.getValue();
    _variance.resize(numParams);
    size_t ij = 0;
    for (size_t i = 0; i < mstate_dim; i++)
        for (size_t j = 0; j < 6; j++, ij++)
            _variance[ij] = helper::SQR(m_stdev.getValue()[i][j]);
}



template<>
void OptimParams<Rigid3dTypes::VecDeriv>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        double actTime = this->getTime() + this->getContext()->getDt(); /// the time has not been increased yet
        std::cout << "[" << this->getName() << "] begin event at time: " << actTime << std::endl;

        int timeSlot = -1;

        for (size_t i = 1; i < m_paramKeys.size() && timeSlot < 0; i++) {
            if (m_paramKeys[i - 1].first <= actTime && actTime < m_paramKeys[i].first) {
                timeSlot = i - 1;
            }
        }
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
                PRNE(this->getName() << " ERROR: no slot found for time " << actTime);
            }
        } else {
            double t1 = m_paramKeys[timeSlot].first;
            double t2 = m_paramKeys[timeSlot + 1].first;
            //  double r1 = (actTime - t1) / (t2 - t1);
            //  double r2 = (t2 - actTime) / (t2 - t1);

            helper::WriteAccessor<Data<Rigid3dTypes::VecDeriv > > val = m_val;
            std::cout << "[" << this->getName() << "] Value: ";
            for (size_t i = 0; i < val.size(); i++) {
                Rigid3dTypes::Deriv v1 = m_paramKeys[timeSlot].second[i];
                Rigid3dTypes::Deriv v2 = m_paramKeys[timeSlot + 1].second[i];

                // (y2 - y1) * (tanh(3.5 * (a - (t0 + t1) / 2)) + 1) / 2 + y1
                if (this->m_interpolateSmooth.getValue())
                    val[i] = (v2 - v1) * (tanh(2 * (actTime - (t1 + t2) / 2)) + 1) / 2 + v1;
                //  else
                //      val[i] = r2 * v1 + r1 * v2;
                //  std::cout << " " << val[i];
            }
            std::cout << std::endl;
        }

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
void OptimParams<double>::init()
{
    Inherit::init();
    m_dim = 1;

    /// take the initial value and initial stdev
    if (!this->m_prescribedParamKeys.getValue().empty()) {
        helper::ReadAccessor< Data< type::vector<double> > >keys = m_prescribedParamKeys;

        size_t numKeyValues = keys.size();

        if ((numKeyValues % 2 ) != 0) {
            PRNE(this->getName() << " ERROR: wrong size of keys, should be N x " << 2);
        } else {
            size_t numKeys = numKeyValues / 2;
            std::cout << this->getName() << " found " << numKeys << " keys for prescribed parameters" << std::endl;

            m_paramKeys.clear();
            m_paramKeys.resize(numKeys);

            for (size_t i = 0; i < numKeys; i++) {
                m_paramKeys[i].first = keys[2 * i];
                m_paramKeys[i].second = keys[2 * i + 1];
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
void OptimParams<double>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        // if (!this->m_optimize.getValue()) {
        double actTime = this->getTime() + this->getContext()->getDt(); /// the time has not been increased yet
        std::cout << "[" << this->getName() << "] begin event at time: " << actTime << std::endl;

        int timeSlot = -1;

        std::cout << "Keys size: " << m_paramKeys.size() << std::endl;
        for (size_t i = 1; i < m_paramKeys.size() && timeSlot < 0; i++) {
            if (m_paramKeys[i - 1].first <= actTime && actTime < m_paramKeys[i].first) {
                timeSlot = i - 1;
            }
        }
        if (timeSlot == -1) {
            if (actTime >= m_paramKeys.back().first) {
                // helper::WriteAccessor<Data<double> > val = m_val;
                double val = m_paramKeys.back().second;
                std::cout << "["  << this->getName() << "] const val: " << val << std::endl;
                m_val.setValue(val);
            } else {
                PRNE(this->getName() << " ERROR: no slot found for time " << actTime);
            }
        } else {
            double t1 = m_paramKeys[timeSlot].first;
            double t2 = m_paramKeys[timeSlot + 1].first;
            double r1 = (actTime - t1) / (t2 - t1);
            double r2 = (t2 - actTime) / (t2 - t1);

            // std::cout << "Time slot: " << timeSlot << " actual time: " << actTime << " ratii: " << r1 << " " << r2 << std::endl;

            // helper::WriteAccessor<Data<double > > val = m_val;
            double val;
            double v1 = m_paramKeys[timeSlot].second;
            double v2 = m_paramKeys[timeSlot + 1].second;

            // (y2 - y1) * (tanh(3.5 * (a - (t0 + t1) / 2)) + 1) / 2 + y1
            if (this->m_interpolateSmooth.getValue()) {
                val = (v2 - v1) * (tanh(2 * (actTime - (t1 + t2) / 2)) + 1) / 2 + v1;
            } else {
                val = r2 * v1 + r1 * v2;
            }
            std::cout << "[" << this->getName() << "] Value: " << val << std::endl;
            m_val.setValue(val);
        }
    }
}



/// DECLARATIONS
SOFA_DECL_CLASS(OptimParams)

// Register in the Factory
int OptimParamsClass = core::RegisterObject("Optimization Parameters")
    .add< OptimParams<double> >()
    .add< OptimParams<type::Vec3> >()
    .add< OptimParams<sofa::type::vector<double> > >()
    .add< OptimParams<Vec3Types::VecDeriv> >()
    .add< OptimParams<Rigid3Types::VecDeriv> >()
    ;


template class SOFA_OPTIMUSPLUGIN_API OptimParams<double>;
template class SOFA_OPTIMUSPLUGIN_API OptimParams<type::Vec3>;
template class SOFA_OPTIMUSPLUGIN_API OptimParams<sofa::type::vector<double> >;
template class SOFA_OPTIMUSPLUGIN_API OptimParams<Vec3Types::VecDeriv>;
template class SOFA_OPTIMUSPLUGIN_API OptimParams<Rigid3Types::VecDeriv>;



} // namespace container

} // namespace component

} // namespace sofa



//template<>
/// puts the params at the known indices
//void OptimParams<sofa::helper::vector<double> >::paramsToRawVector(double* _vector) {
//    helper::ReadAccessor<Data<helper::vector<double> > > val = m_val; // real values of parameters

//    //std::cout<<"INFO: paramsToRawVectorDouble\n";
//    switch (transParamType) {
//    case 1:
//        for (size_t i = 0; i < paramIndices.size(); i++)
//            _vector[paramIndices[i]] = fabs(val[i]);
//        break;
//    case 2:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            _vector[paramIndices[i]] = logit(val[i], m_maxVal.getValue()[i], m_minVal.getValue()[i]);
//        }
//        break;
//    case 3:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            _vector[paramIndices[i]] = log(val[i]);
//            //std::cout << val[i] << std::endl;
//        }
//        break;
//    default:
//        for (size_t i = 0; i < paramIndices.size(); i++)
//            _vector[paramIndices[i]] = val[i];
//    }

//}

//template<>
//void OptimParams<sofa::helper::vector<double> >::paramsToRawVectorParallel(double* _vector)
//{
//    //std::cout<<"INFO:: paramsToRawVectorParallel\n";
//    helper::ReadAccessor<Data<helper::vector<double> > > val = m_val;
//    switch (transParamType) {
//    case 1:
//        for (size_t i = 0; i < paramIndices.size(); i++)
//            _vector[paramIndices[i]] = fabs(val[i]);
//        break;
//    case 2:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            _vector[paramIndices[i]] = logit(val[i], m_maxVal.getValue()[i], m_minVal.getValue()[i]);
//        }
//        break;
//    case 3:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            _vector[paramIndices[i]] = log(val[i]);
//            //std::cout << val[i] << std::endl;
//        }
//        break;
//    default:
//        for (size_t i = 0; i < paramIndices.size(); i++)
//            _vector[paramIndices[i]] = val[i];
//    }
//}

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

//template<>
//void OptimParams<sofa::helper::vector<double> >::rawVectorToParams(const double* _vector) {
//    helper::WriteAccessor<Data<helper::vector<double> > > val = m_val;

//    switch (transParamType) {
//    case 1:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            val[i]=fabs(_vector[this->paramIndices[i]]);
//        }
//        break;
//    case 2:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            val[i]=sigmoid(_vector[this->paramIndices[i]], m_maxVal.getValue()[i], m_minVal.getValue()[i]);
//        }
//        break;
//    case 3:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            val[i]=exp(_vector[this->paramIndices[i]]);
//            //std::cout << val[i] << std::endl;
//        }
//        break;
//    case 4:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            val[i]=std::max(_vector[this->paramIndices[i]], m_minVal.getValue()[i]);
//            val[i]=std::min(_vector[this->paramIndices[i]], m_maxVal.getValue()[i]);
//            //std::cout << val[i] << std::endl;
//        }
//        break;
//    default:
//        for (size_t i = 0; i < this->paramIndices.size(); i++)
//            val[i] = _vector[this->paramIndices[i]];
//    }

//    //std::cout << this->getContext()->getName() << "::" << this->getName() << " params: " << val << std::endl;
//}

//template<>
//void OptimParams<sofa::helper::vector<double> >::rawVectorToParamsParallel(const double* _vector)
//{
//    //std::cout<<"INFO: rawVectorToParamsParallel\n";
//    //std::cout << this->getContext()->getName() << "::" << this->getName() << " paramIdx size: " << this->paramIndices << std::endl;
//    helper::WriteAccessor<Data<helper::vector<double> > > val = m_val;
//    switch (transParamType) {
//    case 1:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            val[i]=fabs(_vector[this->paramIndices[i]]);
//        }
//        break;
//    case 2:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            val[i]=sigmoid(_vector[this->paramIndices[i]], m_maxVal.getValue()[i], m_minVal.getValue()[i]);
//        }
//        break;
//    case 3:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            val[i]=exp(_vector[this->paramIndices[i]]);
//            //std::cout << val[i] << std::endl;
//        }
//        break;
//    case 4:
//        for (size_t i = 0; i < this->paramIndices.size(); i++) {
//            val[i]=std::max(_vector[this->paramIndices[i]], m_minVal.getValue()[i]);
//            val[i]=std::min(_vector[this->paramIndices[i]], m_maxVal.getValue()[i]);
//            //std::cout << val[i] << std::endl;
//        }
//        break;
//    default:
//        for (size_t i = 0; i < this->paramIndices.size(); i++)
//            val[i] = _vector[this->paramIndices[i]];
//    }
//    //std::cout << this->getContext()->getName() << "::" << this->getName() << " params: " << val << std::endl;
//}
//template<>
//void OptimParams<Vec3dTypes::VecDeriv>::rawVectorToParams(const double* _vector) {
//    helper::WriteAccessor<Data<Vec3dTypes::VecDeriv> > waVal = m_val;

//    size_t numParams = mstate_dim;
//    size_t k = 0;

//    switch (transParamType) {
//    case 1:
//        for (size_t i = 0; i < numParams; i++)
//            for (size_t j = 0; j < 3, k < this->paramIndices.size() ; j++, k++)
//                waVal[i][j] = _vector[paramIndices[k]];
//        break;
//    case 2:
//        // std::cout << "backward step " << std::endl;
//        for (size_t i = 0; i < numParams; i++)
//            for (size_t j = 0; j < 3, k < this->paramIndices.size() ; j++, k++)
//                waVal[i][j] = _vector[paramIndices[k]];
//        break;
//    case 3:
//        for (size_t i = 0; i < numParams; i++)
//            for (size_t j = 0; j < 3, k < this->paramIndices.size() ; j++, k++)
//                waVal[i][j] = _vector[paramIndices[k]];
//        break;
//    default:
//        for (size_t i = 0; i < numParams; i++)
//            for (size_t j = 0; j < 3, k < this->paramIndices.size() ; j++, k++)
//                waVal[i][j] = _vector[paramIndices[k]];
//    }
//}
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

