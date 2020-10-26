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

#include "LETKFilter.h"
#include <iostream>
#include <fstream>


namespace sofa
{

namespace component
{

namespace stochastic
{


template <class FilterType>
LETKFilter<FilterType>::LETKFilter()
    : Inherit()
    , d_ensembleMembersNumber(initData(&d_ensembleMembersNumber, "ensembleMemebersNumber", "number of ensemble memebrs for localized ensemble filter" ) )
    , d_state( initData(&d_state, "state", "actual expected value of reduced state (parameters) estimated by the filter" ) )
    , d_variance( initData(&d_variance, "variance", "actual variance  of reduced state (parameters) estimated by the filter" ) )
    , d_covariance( initData(&d_covariance, "covariance", "actual co-variance  of reduced state (parameters) estimated by the filter" ) )
    , d_innovation( initData(&d_innovation, "innovation", "innovation value computed by the filter" ) )
{

}



template <class FilterType>
void LETKFilter<FilterType>::computePerturbedStates()
{
    EVectorX xCol(stateSize);
    int currentPointID = 0;

    for (size_t i = 0; i < ensembleMembersNum; i++) {
        xCol = matXi.col(i);
        stateWrappers[0]->transformState(xCol, mechParams, &currentPointID);
        m_sigmaPointObservationIndexes[i] = currentPointID;
        matXi.col(i) = xCol;
    }
}



template <class FilterType>
void LETKFilter<FilterType>::computePrediction()
{
    // std::cout<<"\n HAS OBS: =" << hasObs << " COMPUTE PREDICTION" << std::endl;
    PRNS("Computing prediction, T= " << this->actualTime  << " ======");
    //std::cout << "Computing prediction, T= " << this->actualTime  << " ======" << std::endl;
    /// Computes background error variance Cholesky factorization.
    Eigen::LLT<EMatrixX> lltU(stateCovar);
    EMatrixX matPsqrt = lltU.matrixL();
    //std::cout << "matPsqrt: " << matPsqrt << std::endl;

    stateExp.fill(FilterType(0.0));
    stateExp = masterStateWrapper->getState();
    //std::cout << "INITIAL STATE: " << stateExp.transpose() << std::endl;

    /// Computes X_{n}^{(i)-} sigma points
    for (size_t i = 0; i < ensembleMembersNum; i++) {
        matXi.col(i) = stateExp + matPsqrt * matI.row(i).transpose();
    }

    //std::cout << "matXi: " << matXi << std::endl;
    /// Propagate sigma points
    genMatXi=matXi.transpose();
    computePerturbedStates();
    //std::cout << "matXi: " << matXi << std::endl;

    /// Compute Predicted Mean
    stateExp.fill(FilterType(0.0));
    for (size_t i = 0; i < ensembleMembersNum; i++) {
        stateExp += matXi.col(i) * vecAlpha(i);
    }

    /// Compute Predicted Covariance
    stateCovar.setZero();
    EMatrixX matXiTrans= matXi.transpose();
    EMatrixX centeredPxx = matXiTrans.rowwise() - matXiTrans.colwise().mean();
    EMatrixX weightedCenteredPxx = centeredPxx.array().colwise() * vecAlphaVar.array();
    EMatrixX covPxx = (centeredPxx.adjoint() * weightedCenteredPxx);
    //EMatrixX covPxx = (centeredPxx.adjoint() * centeredPxx) / double(centeredPxx.rows() )
    stateCovar = covPxx + modelCovar;
    for (size_t i = 0; i < (size_t)stateCovar.rows(); i++) {
        diagStateCov(i)=stateCovar(i,i);
    }

    masterStateWrapper->setState(stateExp, mechParams);
    //std::cout << "PREDICTED STATE: " << stateExp.transpose() << std::endl;
}



template <class FilterType>
void LETKFilter<FilterType>::computeCorrection()
{
    if (hasObs) {
        // std::cout<<"\n HAS OBS: =" << hasObs << " COMPUTE CORRECTION" << std::endl;

        PRNS("======= Computing correction, T= " << this->actualTime << " ======");
        //std::cout << "======= Computing correction, T= " << this->actualTime << " ======" << std::endl;

        EVectorX zCol(observationSize);
        matZmodel.resize(observationSize, ensembleMembersNum);
        predObsExp.resize(observationSize);
        predObsExp.fill(FilterType(0.0));

        /// Compute Predicted Observations
        for (size_t i = 0; i < ensembleMembersNum; i++) {
            observationManager->getPredictedObservation(m_sigmaPointObservationIndexes[i],  zCol);
            //std::cout << "zCol: " << zCol << std::endl;
            matZmodel.col(i) = zCol;
            predObsExp = predObsExp + zCol * vecAlpha(i);
        }

        /// Compute State-Observation Cross-Covariance
        EMatrixX matPxz(stateSize, observationSize);
        EMatrixX matPz(observationSize, observationSize);
        EMatrixX pinvmatPz(observationSize, observationSize);


        EMatrixX matXiTrans= matXi.transpose();
        EMatrixX matZItrans = matZmodel.transpose();
        EMatrixX centeredCx = matXiTrans.rowwise() - matXiTrans.colwise().mean();
        EMatrixX centeredCz = matZItrans.rowwise() - matZItrans.colwise().mean();
        //EMatrixX covPxz = (centeredCx.adjoint() * centeredCz) / double(centeredCx.rows() );
        EMatrixX weightedCenteredCz = centeredCz.array().colwise() * vecAlphaVar.array();
        EMatrixX covPxz = (centeredCx.adjoint() * weightedCenteredCz);
        //std::cout << "covPxz: " << covPxz << std::endl;

        /// Compute Observation Covariance
        matPxz=covPxz;
        //EMatrixX covPzz = (centeredCz.adjoint() * centeredCz) / double(centeredCz.rows() );
        EMatrixX covPzz = (centeredCz.adjoint() * weightedCenteredCz);
        matPz = obsCovar + covPzz;
        //std::cout << "matPz: " << matPz << std::endl;

        /// Compute Kalman Gain
        EMatrixX matK(stateSize, observationSize);
        pseudoInverse(matPz, pinvmatPz);
        matK = matPxz*pinvmatPz;

        /// Compute Innovation
        EVectorX innovation(observationSize);
        observationManager->getInnovation(this->actualTime, predObsExp, innovation);
        //std::cout << "Innovation: " << innovation << std::endl;

        /// Compute Final State and Final Covariance
        stateExp = stateExp + matK * innovation;
        //std::cout << "matK: " << matK << std::endl;
        stateCovar = stateCovar - matK*matPxz.transpose();

        for (size_t i = 0; i < (size_t)stateCovar.rows(); i++) {
            diagStateCov(i)=stateCovar(i,i);
        }

        //std::cout << "FINAL STATE X(n+1)+n: " << stateExp.transpose() << std::endl;
        //std::cout << "FINAL COVARIANCE DIAGONAL P(n+1)+n: " << diagStateCov.transpose() << std::endl;
        PRNS("FINAL STATE X(n+1)+n: \n" << stateExp.transpose());
        PRNS("FINAL COVARIANCE DIAGONAL P(n+1)+n:  \n" << diagStateCov.transpose());

        masterStateWrapper->setState(stateExp, mechParams);

        /// Write Some Data for Validation
        helper::WriteAccessor<Data <helper::vector<FilterType> > > stat = d_state;
        helper::WriteAccessor<Data <helper::vector<FilterType> > > var = d_variance;
        helper::WriteAccessor<Data <helper::vector<FilterType> > > covar = d_covariance;
        helper::WriteAccessor<Data <helper::vector<FilterType> > > innov = d_innovation;

        stat.resize(stateSize);
        var.resize(stateSize);
        size_t numCovariances = (stateSize*(stateSize-1))/2;
        covar.resize(numCovariances);
        innov.resize(observationSize);

        size_t gli = 0;
        for (size_t i = 0; i < stateSize; i++) {
            stat[i] = stateExp[i];
            var[i] = stateCovar(i,i);
            for (size_t j = i+1; j < stateSize; j++) {
                covar[gli++] = stateCovar(i,j);
            }
        }
        for (size_t index = 0; index < observationSize; index++) {
            innov[index] = innovation[index];
        }

        //char nstepc[100];
        //sprintf(nstepc, "%04lu", stepNumber);
        //if (! exportPrefix.empty()) {
        //    std::string fileName = exportPrefix + "/covar_" + nstepc + ".txt";
        //    std::ofstream ofs;
        //    ofs.open(fileName.c_str(), std::ofstream::out);
        //    ofs << stateCovar << std::endl;
        //}

        writeValidationPlot(d_filenameInn.getValue(),innovation);
    }
    writeValidationPlot(d_filenameCov.getValue(), diagStateCov);
    writeValidationPlot(d_filenameFinalState.getValue(), stateExp);
}



template <class FilterType>
void LETKFilter<FilterType>::init() {
    Inherit::init();
    assert(this->gnode);

    this->gnode->template get<StochasticStateWrapperBaseT<FilterType> >(&stateWrappers, this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);
    PRNS("found " << stateWrappers.size() << " state wrappers");
    masterStateWrapper=NULL;
    size_t numSlaveWrappers = 0;
    size_t numMasterWrappers = 0;
    numThreads = 0;
    for (size_t i = 0; i < stateWrappers.size(); i++) {
        stateWrappers[i]->setFilterKind(CLASSIC);
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
void LETKFilter<FilterType>::bwdInit() {
    assert(masterStateWrapper);

    stateSize = masterStateWrapper->getStateSize();
    std::cout<< "[LETKF] stateSize " << stateSize << std::endl;
    PRNS("StateSize " << stateSize);

    /// Initialize Observation's data
    if (!initialiseObservationsAtFirstStep.getValue()) {
        observationSize = this->observationManager->getObservationSize();
        PRNS("Observation size: " << observationSize);
        if (observationSize == 0) {
            PRNE("No observations available, cannot allocate the structures!");
        }
        obsCovar = observationManager->getErrorVariance();
    }


    /// Initialize Model's Error Covariance
    stateCovar = masterStateWrapper->getStateErrorVariance();

    diagStateCov.resize(stateCovar.rows());
    for (size_t i = 0; i < (size_t)stateCovar.rows(); i++) {
        diagStateCov(i)=stateCovar(i,i);
    }
    PRNS(" INIT COVARIANCE DIAGONAL P(n+1)+n:  \n" << diagStateCov.transpose());

    modelCovar = masterStateWrapper->getModelErrorVariance();
    PRNS(" INIT COVARIANCE DIAGONAL P(n+1)+n:  \n" << modelCovar);

    stateExp = masterStateWrapper->getState();

    helper::ReadAccessor<Data <size_t> > membersNum = d_ensembleMembersNumber;
    ensembleMembersNum = membersNum;

    matXi.resize(stateSize, ensembleMembersNum);
    matXi.setZero();
    genMatXi.resize(ensembleMembersNum, stateSize);
    genMatXi.setZero();

    /// Initialise State Observation Mapping for Sigma Points
    m_sigmaPointObservationIndexes.resize(ensembleMembersNum);
}

template <class FilterType>
void LETKFilter<FilterType>::initializeStep(const core::ExecParams* _params, const size_t _step) {
    Inherit::initializeStep(_params, _step);

    if (initialiseObservationsAtFirstStep.getValue()) {
        observationSize = this->observationManager->getObservationSize();
        PRNS("Observation size: " << observationSize);
        if (observationSize == 0) {
            PRNE("No observations available, cannot allocate the structures!");
        }
        obsCovar = observationManager->getErrorVariance();
        initialiseObservationsAtFirstStep.setValue(false);
    }

    for (size_t i = 0; i < stateWrappers.size(); i++)
        stateWrappers[i]->initializeStep(stepNumber);

    observationManager->initializeStep(stepNumber);
}



template <class FilterType>
void LETKFilter<FilterType>::updateState() {

    stateSize = masterStateWrapper->getStateSize();
    //std::cout<< "new [UKF] stateSize " << stateSize << std::endl;
    //PRNS("StateSize " << stateSize);

    /// Initialize Model's Error Covariance
    stateCovar = masterStateWrapper->getStateErrorVariance();

    diagStateCov.resize(stateCovar.rows());
    for (size_t i = 0; i < (size_t)stateCovar.rows(); i++) {
        diagStateCov(i)=stateCovar(i,i);
    }
    //std::cout<< "INIT COVARIANCE DIAGONAL P(n+1)+n: " << diagStateCov.transpose() << std::endl;
    PRNS(" INIT COVARIANCE DIAGONAL P(n+1)+n:  \n" << diagStateCov.transpose());

    modelCovar = masterStateWrapper->getModelErrorVariance();
    //std::cout<< "Model covariance: " << modelCovar << std::endl;
    PRNS(" INIT COVARIANCE DIAGONAL P(n+1)+n:  \n" << modelCovar);

    stateExp = masterStateWrapper->getState();

    helper::ReadAccessor<Data <size_t> > membersNum = d_ensembleMembersNumber;
    ensembleMembersNum = membersNum;

    matXi.resize(stateSize, ensembleMembersNum);
    matXi.setZero();
    genMatXi.resize(ensembleMembersNum, stateSize);
    genMatXi.setZero();

    /// Initialise State Observation Mapping for Sigma Points
    m_sigmaPointObservationIndexes.resize(ensembleMembersNum);
}



template <class FilterType>
void LETKFilter<FilterType>::stabilizeMatrix (EMatrixX& _initial, EMatrixX& _stabilized) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(_initial, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singVals = svd.singularValues();
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singValsStab = singVals;
    for (int i=0; i < singVals.rows(); i++ ){
        if ((singValsStab(i)*singValsStab(i))*(1.0/(singVals(0)*singVals(0)))< 1.0e-6) singValsStab(i)=0;
    }
    _stabilized= svd.matrixU()*singValsStab*svd.matrixV().transpose();

}

template <class FilterType>
void LETKFilter<FilterType>::pseudoInverse( EMatrixX& M,EMatrixX& pinvM) {
    double epsilon= 1e-15;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singVals = svd.singularValues();
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType invSingVals = singVals;
    for(int i=0; i<singVals.rows(); i++) {
        if(singVals(i)*singVals(i) <= epsilon*epsilon) invSingVals(i) = 0.0;
        else invSingVals(i) = 1.0 / invSingVals(i);
    }
    Eigen::MatrixXd S_inv = invSingVals.asDiagonal();
    pinvM = svd.matrixV()*S_inv* svd.matrixU().transpose();
}

template <class FilterType>
void LETKFilter<FilterType>::sqrtMat(EMatrixX& A, EMatrixX& sqrtA){
    double epsilon= 1e-15;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singVals = svd.singularValues();
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sqrtSingVals = singVals;
    for(int i=0; i<singVals.rows(); i++) {
        if(singVals(i)*singVals(i) <= epsilon*epsilon) sqrtSingVals(i) = 0.0;
        else sqrtSingVals(i) = sqrt(sqrtSingVals(i));
    }
    Eigen::MatrixXd S_inv = sqrtSingVals.asDiagonal();
    sqrtA = svd.matrixV()*S_inv* svd.matrixU().transpose();

}

template <class FilterType>
void LETKFilter<FilterType>::writeValidationPlot (std::string filename ,EVectorX& state ){
    if (this->saveParam) {
        std::ofstream paramFile(filename.c_str(), std::ios::app);
        if (paramFile.is_open()) {
            paramFile << state.transpose() << "";
            paramFile << '\n';
            paramFile.close();
        }
    }
}


} // stochastic

} // component

} // sofa

