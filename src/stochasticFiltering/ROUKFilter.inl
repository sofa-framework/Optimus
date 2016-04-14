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

template <class FilterType>
ROUKFilter<FilterType>::ROUKFilter()
{    
}

template <class FilterType>
ROUKFilter<FilterType>::~ROUKFilter()
{
}

template <class FilterType>
void ROUKFilter<FilterType>::computePrediction()
{
    PRNS("Computing prediction");
    EVectorX vecX = stateWrapper->getState();
    Eigen::LLT<EMatrixX> lltU(matUinv);
    matUinv = lltU.matrixL();
    EMatrixX tmpStateVarProj = stateWrapper->getStateErrorVarianceProjector() * matUinv;
    stateWrapper->setStateErrorVarianceProjector(tmpStateVarProj);

    matXi.resize(stateSize, sigmaPointsNum);
    for (size_t i = 0; i < sigmaPointsNum; i++)
        matXi.col(i) = vecX;
    matXi = matXi + tmpStateVarProj * matI;

    vecX.setZero();
    EVectorX xCol(stateSize);

    for (size_t i = 0; i < sigmaPointsNum; i++) {
        xCol = matXi.col(i);
        stateWrapper->applyOperator(xCol);
        vecX = vecX + alpha*xCol;
        matXi.col(i) = xCol;
    }

    /// TODO: add resampling!!!
    tmpStateVarProj = alpha*matXi * matItrans;
    stateWrapper->setStateErrorVarianceProjector(tmpStateVarProj);
    stateWrapper->setState(vecX);
}


template <class FilterType>
void ROUKFilter<FilterType>::computeCorrection()
{
    PRNS("computing correction");
    EMatrixX matZitrans(sigmaPointsNum, observationsNum);

    /*TIC
    sigma_point_matrix Z_i_trans(Nsigma_point_, Nobservation_);
    sigma_point x_col;
    observation z(Nobservation_);
    z.Fill(To(0));
    if (!alpha_constant_)
        throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                             "Analyse()", "Calculation not "
                             "implemented for non constant alpha_i.");

    for (int i = 0; i < Nsigma_point_; i++)
    {
        GetRowPointer(X_i_trans_, i, x_col);
        observation& z_col =
            observation_manager_->GetInnovation(x_col);
        Add(To(alpha_), z_col, z);
        //std::cout << "Innovation: " << z_col << std::endl;
        SetRow(z_col, i, Z_i_trans);
        x_col.Nullify();
    }*/
}

template <class FilterType>
void ROUKFilter<FilterType>::init() {
    Inherit::init();
    assert(this->gnode);
    this->gnode->get(stateWrapper, core::objectmodel::BaseContext::SearchDown);
    this->gnode->get(observationManager, core::objectmodel::BaseContext::SearchDown);

    if (stateWrapper) {
        PRNS("found stochastic state wrapper: " << stateWrapper->getName());
    } else
        PRNE("no state wrapper found!");

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

    observationsNum = this->observationManager->getObservationSize();
    stateSize = stateWrapper->getStateSize();
    EMatrixX temp = stateWrapper->getStateErrorVarianceReduced();
    matU = temp;

    reducedStateSize = matU.cols();
    matUinv = matU.inverse();

    //PRNW("size: " << matU.rows() << " X " << matU.cols());

    /// compute sigma points
    EMatrixX matVtrans;
    computeSimplexSigmaPoints(matVtrans);
    sigmaPointsNum = matVtrans.rows();

    PRNS("Reduced state size: " << reducedStateSize);
    PRNS("Number of sigma points: " << sigmaPointsNum);
    PRNS("Observation size: " << observationsNum);

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
