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
void UKFilter<FilterType>::computePrediction()
{
    PRNS("Computing prediction, T= " << this->actualTime);


    EVectorX vecX = masterStateWrapper->getState();

    for (size_t i = 0; i < sigmaPointsNum; i++)
        matXi.col(i) = vecX;
        matXi = matXi + matVinv * matI;     //Compute perturbed states

    EVectorX xCol(stateSize);
    for (size_t i = 0; i < sigmaPointsNum; i++) {
        xCol = matXi.col(i);
        stateWrappers[sigmaPoints2WrapperIDs[i]]->applyOperator(xCol, mechParams);
        matXi.col(i) = xCol;
    }
    vecX = alpha * matXi.rowwise().sum();

    masterStateWrapper->setState(vecX, this->mechParams);
    masterStateWrapper->writeState(this->getTime());
}


template <class FilterType>
void UKFilter<FilterType>::computeCorrection()
{

    PRNS("Computing correction, T= " << this->actualTime);   

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

        EMatrixX matHLtrans(stateSize, observationSize);
        matHLtrans = alpha*matItrans.transpose()*matZItrans;

        EMatrixX matWorkingPO(stateSize, observationSize), matTemp;
        if (observationErrorVarianceType.getValue() == "inverse") {
            matWorkingPO = matHLtrans * observationManager->getErrorVarianceInverse();
        } else {
            EMatrixX matR;
            matR = observationManager->getErrorVariance();
            matWorkingPO = matHLtrans * matR.inverse();
        }        
        matTemp = EMatrixX::Identity(matVinv.rows(), matVinv.cols()) + matWorkingPO * matHLtrans.transpose();

        matVinv = matTemp.inverse();

        EVectorX Innovation(stateSize);
        Innovation = Type(-1.0) * matVinv*matWorkingPO*vecZ;

        EVectorX state = masterStateWrapper->getState();
        EMatrixX errorVar = masterStateWrapper->getStateErrorVariance();
        state = state + errorVar*Innovation;

    }
}

template <class FilterType>
void UKFilter<FilterType>::init() {
    Inherit::init();
    assert(this->gnode);
    this->gnode->template get<StochasticStateWrapperBaseT<FilterType> >(&stateWrappers, this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);
    PRNS("found " << stateWrappers.size() << " state wrappers");
    masterStateWrapper=NULL;

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
    stateSize = masterStateWrapper->getStateSize();
    matV = masterStateWrapper->getStateErrorVariance();
    matVinv = matV.inverse();

    /// compute sigma points
    EMatrixX matVtrans;
    computeSimplexSigmaPoints(matVtrans);
    sigmaPointsNum = matVtrans.rows();

    PRNS("State size: " << stateSize);
    PRNS("Number of sigma points: " << sigmaPointsNum);
    PRNS("Observation size: " << observationSize);

    EMatrixX matPalphaV(stateSize, stateSize);
    matItrans.resize(sigmaPointsNum, stateSize);

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
