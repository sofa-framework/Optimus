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
#ifndef UKFilter_INL
#define UKFilter_INL

#include "UKFilter.h"

namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType>
UKFilter<FilterType>::UKFilter()
    : Inherit()
    , observationErrorVarianceType( initData(&observationErrorVarianceType, std::string("inverse"), "observationErrorVarianceType", "if set to inverse, work directly with the inverse of the matrix" ) )
{

}

template <class FilterType>
UKFilter<FilterType>::~UKFilter() {}

template <class FilterType>
void UKFilter<FilterType>::propagatePerturbedStates(EVectorX & _meanState) {
//    if (numThreads == 1) {
        EVectorX xCol(stateSize);
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            xCol = matXi.col(i);
            stateWrappers[sigmaPoints2WrapperIDs[i]]->applyOperator(xCol, mechParams);
            matXi.col(i) = xCol;
        }
        _meanState = alpha * matXi.rowwise().sum();
  /*  } else {
        pthread_t* tid = new pthread_t[numThreads];
        WorkerThreadData<FilterType>* threadData = new WorkerThreadData<FilterType>[numThreads];
        for (size_t i = 0; i < numThreads; i++) {
            threadData[i].set(i, stateWrappers[i], &wrapper2SigmaPointsIDs[i], &matXi, this->execParams, 0);
            PRNS("Creating thread " << i);
            pthread_create(&tid[i], NULL, threadFunction<FilterType>, (void*)&threadData[i]);
        }

        for (size_t i = 0; i < numThreads; i++) {
            int s = pthread_join(tid[i], NULL);
            if (s != 0)
                PRNE("Thread " << i << " problem: " << s);
        }
        _meanState = alpha * matXi.rowwise().sum();
    }*/
}
template <class FilterType>
void UKFilter<FilterType>::computePrediction()
{
    PRNS("Computing prediction, T= " << this->actualTime);

    /// Computes background error variance Cholesky factorization.
    Eigen::LLT<EMatrixX> lltU(matP);
    matPsqrt = lltU.matrixL();

    /// Computes X_{n + 1}^{(i)-} sigma points
    EVectorX vecX = masterStateWrapper->getState();
    for (size_t i = 0; i < sigmaPointsNum; i++) {
        matXi.col(i) = vecX + matPsqrt * matI.col(i);  ///maybe it is matI.row
    }

    /// Propagate sigma points and compute X_{n+1}- predicted mean
    propagatePerturbedStates(vecX);

    /// Computes stateCovariance P_ = cov(X_{n + 1}^*, X_{n + 1}^*).
    EMatrixX C = matXi;
    for (size_t i = 0; i < sigmaPointsNum; i++){
        C.col(i) = C.col(i) - vecX;
    }
    for (size_t i = 0; i < sigmaPointsNum; i++){
        for (size_t j = 0; j < sigmaPointsNum; j++){
            EVectorX Ci = C.col(i);
            EVectorX Cj = C.col(j);
            matP(i,j)=Ci.dot(Cj);
        }
    }


    masterStateWrapper->setState(vecX, this->mechParams);
    masterStateWrapper->writeState(this->getTime());
}


template <class FilterType>
void UKFilter<FilterType>::computeCorrection()
{

    PRNS("Computing correction, T= " << this->actualTime);   

    /// Computes background error variance Cholesky factorization.
    Eigen::LLT<EMatrixX> lltU(matP);
    matPsqrt = lltU.matrixL();

    /// Computes X_{n + 1}^{(i)-}
    EVectorX vecX = masterStateWrapper->getState();
    for (size_t i = 0; i < sigmaPointsNum; i++) {
        matXi.col(i) = vecX + matPsqrt * matI.col(i);  ///maybe it is matI.row
    }

    if (observationManager->hasObservation(this->actualTime)) {

        /// Computes Z_{n + 1}^(i).
        EVectorX vecXCol;
        EVectorX vecZCol(observationSize), vecZ(observationSize);
        EMatrixX matZItrans(sigmaPointsNum, observationSize);
        vecZ.setZero();
        //asumEMat("correction input mat",matXi);
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            vecXCol = matXi.col(i);
            vecZCol.setZero();
            observationManager->getInnovation(this->actualTime, vecXCol, vecZCol);
            vecZ = vecZ + alpha * vecZCol;
            matZItrans.row(i) = vecZCol;
        }

        /// Computes the predicted measurement Z_{n + 1}.
        EVectorX Z_mean;
        matZi = matZItrans.transpose();
        Z_mean = alpha * matZi.rowwise().sum();

        /// Computes X_{n+1}-.
        EVectorX X_mean;
        X_mean = alpha * matXi.rowwise().sum();

        /// Computes crosscovariance P_XZ = cov(X_{n + 1}^*, Z_{n + 1}^*).
        EMatrixX Cz = matZi;
        for (size_t i = 0; i < sigmaPointsNum; i++){
            Cz.col(i) = Cz.col(i) - Z_mean;
        }
        EMatrixX Cx = matXi;
        for (size_t i = 0; i < sigmaPointsNum; i++){
            Cx.col(i) = Cx.col(i) -X_mean;
        }
        for (size_t i = 0; i < sigmaPointsNum; i++){
            for (size_t j = 0; j < sigmaPointsNum; j++){
                EVectorX Ci = Cx.col(i);
                EVectorX Cj = Cz.col(j);
                matPxz(i,j) = Ci.dot(Cj);
            }
        }

        /// Computes P_Z = cov(Z_{n + 1}^*, Z_{n + 1}^*).
        EMatrixX C = matZi;
        for (size_t i = 0; i < sigmaPointsNum; i++){
            C.col(i) = C.col(i) - Z_mean;
        }
        for (size_t i = 0; i < sigmaPointsNum; i++){
            for (size_t j = 0; j < sigmaPointsNum; j++){
                EVectorX Ci = C.col(i);
                EVectorX Cj = C.col(j);
                matPzz(i,j) = Ci.dot(Cj);
            }
            matPzz.col(i) = matW.col(i) + matPzz.col(i);
        }

        ///  Computes the Kalman gain K_{n + 1}.
        EMatrixX matK(stateSize, observationSize);
        matK= matPxz * matPzz.inverse();

        /// Computes X_{n + 1}^+.
        EVectorX state = masterStateWrapper->getState();
        EVectorX Innovation(stateSize);
        Innovation = Type(-1.0) *vecZ;

        state = state + matK*Innovation;

        ///  Computes P_{n + 1}^+.
        matP = matP - matK*matPxz.transpose();

        masterStateWrapper->setState(state, this->mechParams);
    }
}

template <class FilterType>
void UKFilter<FilterType>::init() {
    Inherit::init();
    assert(this->gnode);
    this->gnode->template get<StochasticStateWrapperBaseT<FilterType> >(&stateWrappers, this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);
    PRNS("found " << stateWrappers.size() << " state wrappers");
    masterStateWrapper=NULL;
    size_t numSlaveWrappers = 0;
    size_t numMasterWrappers = 0;
    numThreads = 0;
    for (size_t i = 0; i < stateWrappers.size(); i++) {
        if (stateWrappers[i]->isSlave()) {
            numSlaveWrappers++;
            PRNS("found stochastic state slave wrapper: " << stateWrappers[i]->getName());
        } else {
            masterStateWrapper = stateWrappers[i];
            numMasterWrappers++;
            PRNS("found stochastic state master wrapper: " << masterStateWrapper->getName());
        }
    }

    if (numMasterWrappers != 1) {
        PRNE("Wrong number of master wrappers: " << numMasterWrappers);
        return;
    }

    if (numThreads > 0) {
        PRNS("number of slave wrappers: " << numThreads-1);
           /// slaves + master
    }
    numThreads=numSlaveWrappers+numMasterWrappers;

    this->gnode->get(observationManager, core::objectmodel::BaseContext::SearchDown);
    if (observationManager) {
        PRNS("found observation manager: " << observationManager->getName());
    } else
        PRNE("no observation manager found!");
}

template <class FilterType>
void UKFilter<FilterType>::bwdInit() {
    PRNS("bwdInit");
    assert(masterStateWrapper);

    observationSize = this->observationManager->getObservationSize();
    PRNS("DEBUG Observation size" << this->observationManager->getObservationSize());     /// DEBUG observations not found
    stateSize = masterStateWrapper->getStateSize();
    matP = masterStateWrapper->getStateErrorVarianceUKF();
    matW = observationManager->getErrorVariance();

    /// compute sigma points

    computeSimplexSigmaPoints(matI);
    sigmaPointsNum = matI.rows();
    matXi.resize(stateSize, sigmaPointsNum);

    masterStateWrapper->writeState(double(0.0));

}

template <class FilterType>
void UKFilter<FilterType>::initializeStep(const core::ExecParams* _params, const size_t _step) {
    Inherit::initializeStep(_params, _step);

    for (size_t i = 0; i < stateWrappers.size(); i++)
        stateWrappers[i]->initializeStep(stepNumber);

    observationManager->initializeStep(stepNumber);
}

template <class FilterType>
void UKFilter<FilterType>::computeSimplexSigmaPoints(EMatrixX& sigmaMat) {

    size_t p = stateSize;
    size_t r = stateSize + 1;


    EMatrixX workingMatrix = EMatrixX::Zero(p, r);

    Type scal, beta, sqrt_p;
    beta = Type(p) / Type(p+1);
    sqrt_p = sqrt(Type(p));
    scal = Type(1.0)/sqrt(Type(2) * beta);
    workingMatrix(0,0) = -scal;
    workingMatrix(0,1) = scal;

    for (size_t i = 1; i < p; i++) {
        scal = Type(1.0) / sqrt(beta * Type(i+1) * Type(i+2));

        for (size_t j = 0; j < i+1; j++)
            workingMatrix(i,j) = scal;
        workingMatrix(i,i+1) = -Type(i+1) * scal;
    }

    sigmaMat.resize(r,p);
    for (size_t i = 0; i < r; i++)
        sigmaMat.row(i) = workingMatrix.col(i) * sqrt_p;

    vecAlpha.resize(r);
    vecAlpha.fill(Type(1.0)/Type(r));
    alphaConstant = true;
}


} // stochastic
} // component
} // sofa

#endif // UKFilter_INL
