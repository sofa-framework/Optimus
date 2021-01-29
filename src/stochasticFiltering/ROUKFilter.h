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
#pragma once

#include "initOptimusPlugin.h"
#include "StochasticFilterBase.h"
#include "StochasticStateWrapper.h"
#include "ObservationManagerBase.h"

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <sofa/helper/AdvancedTimer.h>

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


/// to speed up, wrappers for BLAS matrix multiplications created, much faster that Eigen by default
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


/**
 * Class implementing reduced-order unscented Kalman filter as it's described in
 * Moireau, Philippe, and Dominique Chapelle. "Reduced-order Unscented Kalman Filtering with application to parameter identification in large-dimensional systems."
 * ESAIM: Control, Optimisation and Calculus of Variations 17.2 (2011): 380-405.
 * Naming conventions inspired by Verdandi library, corresponds to symbols used in the paper.
 * Filter requires StochasticStateWrapper which provides the interface with SOFA.
 */


template <class FilterType>
class ROUKFilter: public sofa::component::stochastic::StochasticUnscentedFilterBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ROUKFilter, FilterType), StochasticUnscentedFilterBase);

    typedef sofa::component::stochastic::StochasticUnscentedFilterBase Inherit;
    typedef FilterType Type;

    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

ROUKFilter();
~ROUKFilter();

protected:
    StochasticStateWrapperBaseT<FilterType>* masterStateWrapper;
    helper::vector<StochasticStateWrapperBaseT<FilterType>*> stateWrappers;
    ObservationManager<FilterType>* observationManager;

    /// vector sizes
    size_t observationSize, stateSize, reducedStateSize;

    /// number of sigma points (according to the filter type)
    size_t sigmaPointsNum;
    bool alphaConstant;

    EVectorX vecAlpha, vecAlphaVar;
    EMatrixX matU, matUinv;
    EMatrixX matItrans, matI;
    EMatrixX matDv;
    EMatrixX matXi;

    Type alpha, alphaVar;

    /// structures for parallel computing:
    helper::vector<size_t> sigmaPoints2WrapperIDs;
    helper::vector<helper::vector<size_t> > wrapper2SigmaPointsIDs;
    size_t numThreads;

    /// functions
    void computeSimplexSigmaPoints(EMatrixX& sigmaMat);
    void computeStarSigmaPoints(EMatrixX& sigmaMat);
    void blasMultAdd(EMatrixX& _a, EMatrixX& _b, EMatrixX& _c, Type _alpha, Type _beta);
    void blasMultAdd(char _trans1, char _trans2, EMatrixX& _a, EMatrixX& _b, EMatrixX& _c, Type _alpha, Type _beta);

    void computeSimplexPrediction();
    void computeStarPrediction();
    void computeSimplexCorrection();
    void computeStarCorrection();

public:
    Data<std::string> observationErrorVarianceType;
    Data<bool> useBlasToMultiply;
    Data<helper::vector<FilterType> > reducedState;
    Data<helper::vector<FilterType> > reducedVariance;
    Data<helper::vector<FilterType> > reducedCovariance;
    Data<helper::vector<FilterType> > d_reducedInnovation;
    Data<helper::vector<FilterType> > d_state;
    Data<helper::vector<FilterType> > d_variance;
    Data<helper::vector<FilterType> > d_covariance;
    Data<bool> d_executeSimulationForCorrectedData;


    void init() override;
    void bwdInit() override;

    virtual void initializeStep(const core::ExecParams* _params, const size_t _step) override;

    virtual void computePrediction() override;
    virtual void computePerturbedStates(EVectorX &_meanState);
    virtual void computeCorrection() override;

    virtual void updateState() override { }

}; /// class

/**
 * Code necessary for parallelization. Not tested since longer time.
 */

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
        sprintf(name, "thread%02lu.out", id);
        fd.open(name);
        fd << "Thread " << id << std::endl;
        //fd << "Sigma points to process: " << sigIDs << std::endl;
    }

    Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic>& xMat = *(threadData->stateMatrix);
    Eigen::Matrix<FilterType, Eigen::Dynamic, 1> xCol(xMat.rows());
    for (size_t i = 0; i < sigIDs.size(); i++) {
        xCol = xMat.col(sigIDs[i]);
        wrapper->transformState(xCol, mechParams);
        xMat.col(sigIDs[i]) = xCol;
    }
    fd.close();
    delete mechParams;
    return nullptr;
}

} // stochastic

} // component

} // sofa

