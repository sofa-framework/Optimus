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
#ifndef UKFilter_H_
#define UKFilter_H_

#include "initOptimusPlugin.h"
#include "StochasticFilterBase.h"
#include "StochasticStateWrapper.h"
#include "ObservationManagerBase.h"

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#ifdef Success
#undef Success // dirty workaround to cope with the (dirtier) X11 define. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=253
#endif
#include <Eigen/Dense>

//#include <Accelerate/Accelerate.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType>
struct WorkerThreadData
{
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;

    StochasticStateWrapperBaseT<FilterType>* wrapper;
    size_t threadID;
    helper::vector<size_t>* sigmaIDs;
    EMatrixX* stateMatrix;
    const core::ExecParams* execParams;
    bool saveLog;

    void set(size_t _threadID,StochasticStateWrapperBaseT<FilterType>* _wrapper,  helper::vector<size_t> *_sigID,
             EMatrixX* _stateMat, const core::ExecParams* _execParams, bool _saveLog) {
        threadID=_threadID;
        sigmaIDs=_sigID;
        stateMatrix=_stateMat;
        saveLog = _saveLog;
        wrapper = _wrapper;
        execParams = _execParams;
    }
};

template <class FilterType>
void* threadFunction(void* inArgs) {
    char name[100];
    std::ofstream fd;
    WorkerThreadData<FilterType>* threadData = reinterpret_cast<WorkerThreadData<FilterType>* >(inArgs);
    helper::vector<size_t>& sigIDs = *(threadData->sigmaIDs);
    size_t id = threadData->threadID;
    StochasticStateWrapperBaseT<FilterType>* wrapper = threadData->wrapper;

    //const core::ExecParams* execParams = threadData->execParams;
    //core::MechanicalParams* mechParams;
    //if (wrapper->isSlave())
        //mechParams = new core::MechanicalParams;
        //mechParams->setThreadID(id);
    //else
    core::MechanicalParams* mechParams = new core::MechanicalParams;
    //mechParams->setThreadID(0);

    bool saveLog = threadData->saveLog;

    if (saveLog) {
        sprintf(name, "thread%02d.out", id);
        fd.open(name);
        fd << "Thread " << id << std::endl;
        //fd << "Sigma points to process: " << sigIDs << std::endl;
    }

    Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic>& xMat = *(threadData->stateMatrix);
    Eigen::Matrix<FilterType, Eigen::Dynamic, 1> xCol(xMat.rows());
    for (size_t i = 0; i < sigIDs.size(); i++) {
        xCol = xMat.col(sigIDs[i]);
        wrapper->applyOperator(xCol, mechParams);
        xMat.col(sigIDs[i]) = xCol;
    }
    fd.close();
    delete mechParams;
}

extern "C"{
    // product C= alphaA.B + betaC
   void dgemm_(char* TRANSA, char* TRANSB, const int* M,
               const int* N, const int* K, double* alpha, double* A,
               const int* LDA, double* B, const int* LDB, double* beta,
               double* C, const int* LDC);
    // product Y= alphaA.X + betaY
   void dgemv_(char* TRANS, const int* M, const int* N,
               double* alpha, double* A, const int* LDA, double* X,
               const int* INCX, double* beta, double* C, const int* INCY);
   }

using namespace defaulttype;

template <class FilterType>
class UKFilter: public sofa::component::stochastic::StochasticFilterBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(UKFilter, FilterType), StochasticFilterBase);

    typedef sofa::component::stochastic::StochasticFilterBase Inherit;
    typedef FilterType Type;

    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

UKFilter();
~UKFilter();

protected:
    StochasticStateWrapperBaseT<FilterType>* masterStateWrapper;
    helper::vector<StochasticStateWrapperBaseT<FilterType>*> stateWrappers;
    ObservationManager<FilterType>* observationManager;

    /// vector sizes
    size_t observationSize, stateSize, reducedStateSize;

    /// number of sigma points (according to the filter type)
    size_t sigmaPointsNum;
    bool alphaConstant;

    EVectorX vecAlpha;
    EMatrixX matU, matUinv;
    EMatrixX matItrans, matI;
    EMatrixX matDv;
    EMatrixX matXi;
    EMatrixX matV, matVinv;

    Type alpha;

    /// structures for parallel computing:
    helper::vector<size_t> sigmaPoints2WrapperIDs;
    helper::vector<helper::vector<size_t> > wrapper2SigmaPointsIDs;
    size_t numThreads;

    /// functions
    void computeSimplexSigmaPoints(EMatrixX& sigmaMat);
    void blasMultAdd(EMatrixX& _a, EMatrixX& _b, EMatrixX& _c, Type _alpha, Type _beta);

public:
    Data<std::string> observationErrorVarianceType;
    Data<bool> useBlasToMultiply;
    Data<helper::vector<FilterType> > reducedState;
    Data<helper::vector<FilterType> > reducedVariance;
    Data<helper::vector<FilterType> > reducedCovariance;


    void init();
    void bwdInit();

    virtual void computePrediction();
    virtual void computePerturbedStates(EVectorX &_meanState);

    virtual void computeCorrection();

    virtual void initializeStep(const core::ExecParams* _params, const size_t _step);

}; /// class

} // stochastic
} // component
} // sofa

#endif // UKFilter_H


