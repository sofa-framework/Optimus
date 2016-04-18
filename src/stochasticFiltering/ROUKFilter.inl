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
#ifndef ROUKFILTER_INL
#define ROUKFILTER_INL

#include "ROUKFilter.h"

namespace sofa
{
namespace component
{
namespace stochastic
{

/// C = alpha*A*B + beta*C;
template <class FilterType>
void ROUKFilter<FilterType>::blasMultAdd(EMatrixX& _a, EMatrixX& _b, EMatrixX& _c, Type _alpha, Type _beta) {
    char trans = 'N';
    int m = _a.rows();
    int n = _b.cols();
    int k = _b.rows();
    Type* _adata = _a.data();
    Type* _bdata = _b.data();
    Type* _cdata = _c.data();
    dgemm_(&trans,&trans, &m, &n, &k, &_alpha, _adata, &m, _bdata, &k, &_beta, _cdata, &m);
}

template <class FilterType>
ROUKFilter<FilterType>::ROUKFilter()
    : Inherit()
    , observationErrorVarianceType( initData(&observationErrorVarianceType, std::string("inverse"), "observationErrorVarianceType", "if set to inverse, work directly with the inverse of the matrix" ) )
    , useBlasToMultiply( initData(&useBlasToMultiply, true, "useBlasToMultiply", "use BLAS to multiply the dense matrices instead of Eigen" ) )
{    
}

template <class FilterType>
ROUKFilter<FilterType>::~ROUKFilter() {}

template <class FilterType>
void ROUKFilter<FilterType>::computePerturbedStates(EVectorX& _meanState) {
    if (numSlaveWrappers == 0) {
        EVectorX xCol(stateSize);
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            xCol = matXi.col(i);
            stateWrappers[sigmaPoints2WrapperIDs[i]]->applyOperator(xCol, mechParams);
            matXi.col(i) = xCol;
        }
        _meanState = alpha * matXi.rowwise().sum();
    } else {
        pthread_t* tid = new pthread_t[numThreads];
        WorkerThreadData<FilterType>* threadData = new WorkerThreadData<FilterType>[numThreads];
        for (size_t i = 0; i < numThreads; i++) {
            threadData[i].set(i, stateWrappers[i], &wrapper2SigmaPointsIDs[i], &matXi, this->execParams, 1);
            PRNS("Creating thread " << i);
            pthread_create(&tid[i], NULL, threadFunction<FilterType>, (void*)&threadData[i]);
        }

        for (size_t i = 0; i < numThreads; i++) {
            int s = pthread_join(tid[i], NULL);
            if (s != 0)
                PRNE("Thread " << i << " problem: " << s);
        }
        _meanState = alpha * matXi.rowwise().sum();
    }
}

template <class FilterType>
void ROUKFilter<FilterType>::computePrediction()
{
    PRNS("Computing prediction, T= " << this->actualTime);

    EMatrixX tmpStateVarProj(stateSize, reducedStateSize);
    EMatrixX tmpStateVarProj2(stateSize, reducedStateSize);

    TIC
    EVectorX vecX = stateWrapper->getState();
    Eigen::LLT<EMatrixX> lltU(matUinv);
    matUinv = lltU.matrixL();
    TOC("== prediction: Cholesky ");

    tmpStateVarProj = stateWrapper->getStateErrorVarianceProjector();

    TIC
    if (useBlasToMultiply.getValue())
        blasMultAdd(tmpStateVarProj, matUinv, tmpStateVarProj2, 1.0, 0.0);
    else
        tmpStateVarProj2.noalias() = tmpStateVarProj * matUinv;
    TOC("== prediction multiplication1 == ");

    stateWrapper->setStateErrorVarianceProjector(tmpStateVarProj2);
    for (size_t i = 0; i < sigmaPointsNum; i++)
        matXi.col(i) = vecX;

    TIC
    if (useBlasToMultiply.getValue())
        blasMultAdd(tmpStateVarProj2, matI, matXi, 1.0, 1.0);
    else
        matXi = matXi + tmpStateVarProj2 * matI;
    TOC("== prediction multiplication2 == ");
    computePerturbedStates(vecX);
    TIC;

    TOCTIC("== prediction applyOperator ==");

    /// TODO: add resampling!!!

    if (useBlasToMultiply.getValue())
        blasMultAdd(matXi, matItrans, tmpStateVarProj2, alpha, 0.0);
    else
        tmpStateVarProj2 = alpha*matXi * matItrans;
    TOC("== prediction multiplication3 ==");

    stateWrapper->setStateErrorVarianceProjector(tmpStateVarProj2);
    stateWrapper->setState(vecX);
}


template <class FilterType>
void ROUKFilter<FilterType>::computeCorrection()
{
    PRNS("Computing correction, T= " << this->actualTime);

    if (!alphaConstant) {
        PRNE("Version for non-constant alpha not implemented!");
        return;
    }

    if (observationManager->hasObservation(this->actualTime)) {
        TIC
        EVectorX vecXCol;
        EVectorX vecZCol(observationSize), vecZ(observationSize);
        EMatrixX matZItrans(sigmaPointsNum, observationSize);
        vecZ.setZero();
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            vecXCol = matXi.col(i);
            vecZCol.setZero();
            observationManager->getInnovation(this->actualTime, vecXCol, vecZCol);
            vecZ = vecZ + alpha * vecZCol;
            matZItrans.row(i) = vecZCol;
        }
        TOCTIC("== an1sx == ");

        EMatrixX matHLtrans(reducedStateSize, observationSize);
        matHLtrans = alpha*matItrans.transpose()*matZItrans;

        EMatrixX matWorkingPO(reducedStateSize, observationSize), matTemp;
        if (observationErrorVarianceType.getValue() == "inverse")
            matWorkingPO = matHLtrans * observationManager->getErrorVarianceInverse();
        else {
            EMatrixX matR;
            matR = observationManager->getErrorVariance();
            matWorkingPO = matHLtrans * matR.inverse();
        }
        matTemp = EMatrixX::Identity(matUinv.rows(), matUinv.cols()) + matWorkingPO * matHLtrans.transpose();
        TOCTIC("== an3sx == ");
        matUinv = matTemp.inverse();
        TOCTIC("== an4sx == (inv) ");

        EVectorX reducedInnovation(reducedStateSize);
        reducedInnovation = Type(-1.0) * matUinv*matWorkingPO*vecZ;

        EVectorX state = stateWrapper->getState();
        EMatrixX errorVarProj = stateWrapper->getStateErrorVarianceProjector();
        state = state + errorVarProj*reducedInnovation;
        stateWrapper->setState(state);
        TOC("== an5sx == ");

        std::cout << "New state = " << std::endl;
        for (size_t i = stateSize-20; i < stateSize; i++)
            std::cout << state(i) << " ";
        std::cout << std::endl;
    }
}

template <class FilterType>
void ROUKFilter<FilterType>::init() {
    Inherit::init();
    assert(this->gnode);
    this->gnode->template get<StochasticStateWrapperBaseT<FilterType> >(&stateWrappers, this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);
    PRNS("found " << stateWrappers.size() << " stochastic filters");
    stateWrapper=NULL;
    numSlaveWrappers = 0;
    numThreads = 0;
    for (size_t i = 0; i < stateWrappers.size(); i++) {
        if (stateWrappers[i]->isSlave()) {
            numSlaveWrappers++;
            PRNS("found stochastic state slave wrapper: " << stateWrappers[i]->getName());
        } else {
            if (!stateWrapper) {
                stateWrapper = stateWrappers[i];
                PRNS("found stochastic state master wrapper: " << stateWrapper->getName());
            } else {
                PRNE("Master wrapper already assigned, cannot have two of them!");
                return;
            }
        }
    }
    if (!stateWrapper) {
        PRNE("no master state wrapper found!");
        return;
    }

    if (numSlaveWrappers > 0) {
        PRNS("number of slave wrappers: " << numSlaveWrappers);
        numThreads=numSlaveWrappers+1;   /// slaves + master
    }


    this->gnode->get(observationManager, core::objectmodel::BaseContext::SearchDown);
    if (observationManager) {
        PRNS("found observation manager: " << observationManager->getName());
    } else
        PRNE("no observation manager found!");
}

template <class FilterType>
void ROUKFilter<FilterType>::bwdInit() {
    PRNS("bwdInit");
    assert(stateWrapper);
    assert(this->observationManagerBase);

    observationSize = this->observationManager->getObservationSize();
    stateSize = stateWrapper->getStateSize();
    matU = stateWrapper->getStateErrorVarianceReduced();

    reducedStateSize = matU.cols();
    matUinv = matU.inverse();

    //PRNW("size: " << matU.rows() << " X " << matU.cols());

    /// compute sigma points
    EMatrixX matVtrans;
    computeSimplexSigmaPoints(matVtrans);
    sigmaPointsNum = matVtrans.rows();

    PRNS("State size: " << stateSize);
    PRNS("Reduced state size: " << reducedStateSize);
    PRNS("Number of sigma points: " << sigmaPointsNum);
    PRNS("Observation size: " << observationSize);

    EMatrixX matPalphaV(reducedStateSize, reducedStateSize);
    matItrans.resize(sigmaPointsNum, reducedStateSize);

    if (alphaConstant) {
        alpha=vecAlpha(0);
        matPalphaV = alpha*matVtrans.transpose()*matVtrans;
        matPalphaV = matPalphaV.inverse();
        matItrans = matVtrans*matPalphaV.transpose();
    } else {
        PRNE(" simplex method implemented only for constant alpha");
        return;
    }
    matI = matItrans.transpose();

    matDv.resize(sigmaPointsNum, sigmaPointsNum);
    matDv = alpha*alpha*matItrans*matI;

    matXi.resize(stateSize, sigmaPointsNum);

    /// prepare structures for parallel computing
    sigmaPoints2WrapperIDs.resize(sigmaPointsNum, 0);
    wrapper2SigmaPointsIDs.resize(numThreads);
    if (numSlaveWrappers > 0) {
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            size_t threadID = i%(numThreads);
            sigmaPoints2WrapperIDs[i] = threadID;
            wrapper2SigmaPointsIDs[threadID].push_back(i);
        }

        //std::cout << "SigmaWrapper: " << sigmaPoints2WrapperIDs << std::endl;
        //std::cout << "WrapperSigma: " << wrapper2SigmaPointsIDs << std::endl;
    }

}

template <class FilterType>
void ROUKFilter<FilterType>::initializeStep(const core::ExecParams* _params) {
    Inherit::initializeStep(_params);

    for (size_t i = 0; i < stateWrappers.size(); i++)
        stateWrappers[i]->initializeStep(stepNumber);

    observationManager->initializeStep(stepNumber);
}

template <class FilterType>
void ROUKFilter<FilterType>::computeSimplexSigmaPoints(EMatrixX& sigmaMat) {
    size_t p = reducedStateSize;
    size_t r = reducedStateSize + 1;

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


//template <class FilterType>
//void ROUKFilter<FilterType>::init()
//{
//
//}
//
//template <class FilterType>
//void ROUKFilter<FilterType>::reinit()
//{
//}

} // stochastic
} // component
} // sofa

#endif // ROUKFILTER_INL
