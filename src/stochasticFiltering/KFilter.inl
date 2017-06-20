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
#ifndef KFILTER_INL
#define KFILTER_INL

#include "KFilter.h"

namespace sofa
{
namespace component
{
namespace stochastic
{

/// C = alpha*A*B + beta*C;
template <class FilterType>
void KFilter<FilterType>::blasMultAdd(EMatrixX& _a, EMatrixX& _b, EMatrixX& _c, Type _alpha, Type _beta) {
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
KFilter<FilterType>::KFilter()
    : Inherit()    
    , observationManager(NULL)
    , sofaStateWrapper(NULL)
    , useBlasToMultiply( initData(&useBlasToMultiply, true, "useBlasToMultiply", "use BLAS to multiply the dense matrices instead of Eigen" ) )
{    
}

template <class FilterType>
KFilter<FilterType>::~KFilter() {}


template <class FilterType>
void KFilter<FilterType>::computePrediction()
{
    PRNS("Computing prediction, T= " << this->actualTime);

    /// apply the operator (perform 1 step with the actual state theta)
    vecState = sofaStateWrapper->getState();
    sofaStateWrapper->applyOperator(vecState, mechParams);

    /// update the covariance matrix  P = P + Q;
    matStateErrorCovar += sofaStateWrapper->getStateErrorVariance();

    /// compute the FE sensitivities: since h(theta) = J*u(theta)
    /// dh/dtheta = du(theta)/dtheta = -J * inv(dK/du) * dK/dtheta
    /// dK/du is the tangent stiffness matrix, dK/dtheta is the FE sensitivity matrix, J is the mapping between observations and u

    /*EMatrixX tmpStateVarProj(stateSize, reducedStateSize);
    EMatrixX tmpStateVarProj2(stateSize, reducedStateSize);

    TIC
    EVectorX vecX = masterStateWrapper->getState();
    Eigen::LLT<EMatrixX> lltU(matUinv);
    matUinv = lltU.matrixL();
    TOC("== prediction: Cholesky ");

    tmpStateVarProj = masterStateWrapper->getStateErrorVarianceProjector();

    TIC
    if (useBlasToMultiply.getValue())
        blasMultAdd(tmpStateVarProj, matUinv, tmpStateVarProj2, 1.0, 0.0);
    else
        tmpStateVarProj2.noalias() = tmpStateVarProj * matUinv;
    TOC("== prediction multiplication1 == ");

    masterStateWrapper->setStateErrorVarianceProjector(tmpStateVarProj2);
    for (size_t i = 0; i < sigmaPointsNum; i++)
        matXi.col(i) = vecX;

    TIC
    if (useBlasToMultiply.getValue())
        blasMultAdd(tmpStateVarProj2, matI, matXi, 1.0, 1.0);
    else
        matXi = matXi + tmpStateVarProj2 * matI;
    TOC("== prediction multiplication2 == ");

    TIC;
    computePerturbedStates(vecX);   
    //asumEVec("summPred",vecX);
    TOCTIC("== prediction compute perturbations ==");

    /// TODO: add resampling!!!

    if (useBlasToMultiply.getValue())
        blasMultAdd(matXi, matItrans, tmpStateVarProj2, alpha, 0.0);
    else
        tmpStateVarProj2 = alpha*matXi * matItrans;
    TOC("== prediction multiplication3 ==");

    masterStateWrapper->setStateErrorVarianceProjector(tmpStateVarProj2);
    masterStateWrapper->setState(vecX, this->mechParams);*/
}


template <class FilterType>
void KFilter<FilterType>::computeCorrection()
{
    PRNS("Computing correction, T= " << this->actualTime);

    /*if (!alphaConstant) {
        PRNE("Version for non-constant alpha not implemented!");
        return;
    }

    if (observationManager->hasObservation(this->actualTime)) {
        TIC
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
        TOCTIC("== an1sx == ");
        //asumEVec("correction accumInnov",vecZ);

        EMatrixX matHLtrans(reducedStateSize, observationSize);
        matHLtrans = alpha*matItrans.transpose()*matZItrans;
        //asumEMat("HL_trans", matHLtrans);

        EMatrixX matWorkingPO(reducedStateSize, observationSize), matTemp;
        if (observationErrorVarianceType.getValue() == "inverse") {
            matWorkingPO = matHLtrans * observationManager->getErrorVarianceInverse();
        } else {
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
        //asumEMat("matUinv", matUinv);
        //asumEMat("matWorkingPO", matWorkingPO);
        //asumEVec("reduced innovation", reducedInnovation);

        EVectorX state = masterStateWrapper->getState();
        EMatrixX errorVarProj = masterStateWrapper->getStateErrorVarianceProjector();
        state = state + errorVarProj*reducedInnovation;
        masterStateWrapper->setState(state,mechParams);
        TOC("== an5sx == ");

        Type maxState = 0.0, minState = 1e10;
        for (size_t i = stateSize - reducedStateSize; i < stateSize; i++) {
            maxState = (state(i) > maxState) ? state(i) : maxState;
            minState = (state(i) < minState) ? state(i) : minState;
        }

        asumEVec("############# final state", state);
        //std::cout << "Max = " << maxState << " min = " << minState << std::endl;

    }*/
}

template <class FilterType>
void KFilter<FilterType>::init() {
    Inherit::init();

    assert(this->gnode);
    helper::vector<StochasticStateWrapperBaseT<FilterType>*> stateWrappers;
    this->gnode->template get<StochasticStateWrapperBaseT<FilterType> >(&stateWrappers, this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);

    if (stateWrappers.size() != 1) {
        PRNE("only one state wrapper is currently supported with EKF");
        return;
    }

    sofaStateWrapper = stateWrappers[0];

    this->gnode->get(observationManager, core::objectmodel::BaseContext::SearchDown);
    if (observationManager) {
        PRNS("found observation manager: " << observationManager->getName());
    } else
        PRNE("no observation manager found!");
}

template <class FilterType>
void KFilter<FilterType>::bwdInit() {
    PRNS("bwdInit");
    assert(sofaStateWrapper);

    observationSize = this->observationManager->getObservationSize();
    stateSize = sofaStateWrapper->getStateSize();

    PRNS("|stoch.state| = " << stateSize);
    PRNS("|observation| = " << observationSize);

    vecState.resize(stateSize);
    matStateErrorCovar.resize(stateSize, stateSize);
    matStateErrorCovar.setZero();

    /*matU = masterStateWrapper->getStateErrorVarianceReduced();

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
    for (size_t i = 0; i < sigmaPointsNum; i++) {
        size_t threadID = i%(numThreads);
        sigmaPoints2WrapperIDs[i] = threadID;
        wrapper2SigmaPointsIDs[threadID].push_back(i);
    }

    //std::cout << "SigmaWrapper: " << sigmaPoints2WrapperIDs << std::endl;
    //std::cout << "WrapperSigma: " << wrapper2SigmaPointsIDs << std::endl;*/
}

template <class FilterType>
void KFilter<FilterType>::initializeStep(const core::ExecParams* _params, const size_t _step) {
    Inherit::initializeStep(_params, _step);

    sofaStateWrapper->initializeStep(stepNumber);
    observationManager->initializeStep(stepNumber);
}


} // stochastic
} // component
} // sofa

#endif // KFILTER_INL
