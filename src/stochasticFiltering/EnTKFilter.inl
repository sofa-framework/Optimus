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

#include "EnTKFilter.h"
#include <iostream>
#include <fstream>
#include <random>


namespace sofa
{

namespace component
{

namespace stochastic
{


template <class FilterType>
EnTKFilter<FilterType>::EnTKFilter()
    : Inherit()
    , d_ensembleMembersNumber(initData(&d_ensembleMembersNumber, "ensembleMembersNumber", "number of ensemble memebrs for localized ensemble filter" ) )
    , d_additiveNoiseType( initData(&d_additiveNoiseType, size_t(0), "additiveNoiseType", "add noise for ensemble members: 0 - no noise, 1 - after prediction, 2 -- after correction" ) )
    , d_inverseOptionType( initData(&d_inverseOptionType, "inverseOption", "inverse option type") )
    , d_state( initData(&d_state, "state", "actual expected value of reduced state (parameters) estimated by the filter" ) )
    , d_variance( initData(&d_variance, "variance", "actual variance  of reduced state (parameters) estimated by the filter" ) )
    , d_covariance( initData(&d_covariance, "covariance", "actual co-variance  of reduced state (parameters) estimated by the filter" ) )
    , d_innovation( initData(&d_innovation, "innovation", "innovation value computed by the filter" ) )
{

}



template <class FilterType>
void EnTKFilter<FilterType>::computePerturbedStates()
{
    EVectorX xCol(stateSize);
    int currentPointID = 0;

    for (size_t index = 0; index < ensembleMembersNum; index++) {
        xCol = matXi.col(index);
        stateWrappers[0]->transformState(xCol, mechParams, &currentPointID);
        m_sigmaPointObservationIndexes[index] = currentPointID;
        matXi.col(index) = xCol;
    }
}



template <class FilterType>
void EnTKFilter<FilterType>::computePrediction()
{
    // std::cout<<"\n COMPUTE PREDICTION" << std::endl;
    PRNS("Computing prediction, T= " << this->actualTime  << " ======");
    //std::cout << "Computing prediction, T= " << this->actualTime  << " ======" << std::endl;

    //std::cout << "INITIAL STATE: " << stateExp.transpose() << std::endl;

    /// Propagate L ensemble memebers
    //std::cout << "matXi: " << matXi << std::endl;
    computePerturbedStates();
    //std::cout << "matXi: " << matXi << std::endl;

    ///// add inflation noise if needed
    if (d_additiveNoiseType.getValue() == 1) {
        for (size_t index = 0; index < ensembleMembersNum; index++) {
            matXi.col(index) = matXi.col(index) + modelNoise;
        }
        //std::cout << "We are in prediction" << std::endl;
    }

    /// Compute Predicted Mean
    stateExp.fill(FilterType(0.0));
    stateExp = matXi.rowwise().mean();

    /// Compute Predicted Covariance
    stateCovar.setZero();
    matXiPerturb = matXi;
    for (size_t index = 0; index < ensembleMembersNum; index++) {
        matXiPerturb.col(index) = matXiPerturb.col(index) - stateExp;
    }
    stateCovar = matXiPerturb * matXiPerturb.transpose() / Type(ensembleMembersNum - 1);
    for (size_t index = 0; index < (size_t)stateCovar.rows(); index++) {
        diagStateCov(index) = stateCovar(index, index);
    }

    masterStateWrapper->setState(stateExp, mechParams);
    //std::cout << "PREDICTED STATE: " << stateExp.transpose() << std::endl;
}



template <class FilterType>
void EnTKFilter<FilterType>::computeCorrection()
{
    if (observationManager->hasObservation(this->actualTime)) {
        // std::cout<<"\n COMPUTE CORRECTION" << std::endl;

        PRNS("======= Computing correction, T= " << this->actualTime << " ======");
        //std::cout << "======= Computing correction, T= " << this->actualTime << " ======" << std::endl;

        EVectorX zCol(observationSize);
        matZmodel.resize(observationSize, ensembleMembersNum);
        predObsExp.resize(observationSize);

        /// Compute Predicted Observations
        for (size_t index = 0; index < ensembleMembersNum; index++) {
            observationManager->getPredictedObservation(m_sigmaPointObservationIndexes[index],  zCol);
            //std::cout << "zCol: " << zCol << std::endl;
            matZmodel.col(index) = zCol;
        }

        /// get observations perturbation
        predObsExp.fill(FilterType(0.0));
        predObsExp = matZmodel.rowwise().mean();
        for (size_t index = 0; index < ensembleMembersNum; index++) {
            matZmodel.col(index) = matZmodel.col(index) - predObsExp;
        }

        /// Compute ensemble covariance
        EMatrixX matA(ensembleMembersNum, ensembleMembersNum);
        EMatrixX matYR(ensembleMembersNum, ensembleMembersNum);
        if (m_matrixInverse == ENSEMBLE_DIMENSION) {
            matYR = Type(ensembleMembersNum - 1) * EMatrixX::Identity(matYR.rows(), matYR.cols()) + matZmodel.transpose() * obsCovarInv * matZmodel;
            pseudoInverse(matYR, matA);
        } else {
            EMatrixX matRY(observationSize, observationSize);
            EMatrixX matRYInv(observationSize, observationSize);
            matRY = obsCovar + matZmodel * matZmodel.transpose() * Type(ensembleMembersNum - 1);
            pseudoInverse(matRY, matRYInv);
            matYR = matZmodel.transpose() * matRYInv * matZmodel / Type(ensembleMembersNum - 1);
            matA = (EMatrixX::Identity(matYR.rows(), matYR.cols()) - matYR) / Type(ensembleMembersNum - 1);
        }

        /// Compute Innovation
        EVectorX innovation(observationSize);
        observationManager->getInnovation(this->actualTime, predObsExp, innovation);
        //std::cout << "Innovation: " << innovation << std::endl;

        /// Compute minimization of cost function
        EVectorX vecW(ensembleMembersNum);
        vecW = matA * matZmodel.transpose() * obsCovarInv * innovation;

        /// Compute Final State and Final Covariance
        stateExp = stateExp + matXiPerturb * vecW;

        EMatrixX matZ(ensembleMembersNum, ensembleMembersNum);
        EMatrixX matZsqrt(ensembleMembersNum, ensembleMembersNum);
        matZ = matA * Type(ensembleMembersNum - 1);

        /// Compute square root using Cholesky factorization.
        Eigen::LLT<EMatrixX> lltU(matZ);
        matZsqrt = lltU.matrixL();

        stateAnalysisCovar = matZsqrt * matZsqrt.transpose() / Type(ensembleMembersNum - 1);
        for (size_t index = 0; index < (size_t)stateAnalysisCovar.rows(); index++) {
            diagAnalysisStateCov(index) = stateAnalysisCovar(index, index);
        }
        stateCovar = matXi * stateAnalysisCovar * matXi.transpose();
        for (size_t index = 0; index < (size_t)stateCovar.rows(); index++) {
            diagStateCov(index) = stateCovar(index, index);
        }

        //std::cout << "FINAL STATE X(n+1)+n: " << stateExp.transpose() << std::endl;
        //std::cout << "FINAL COVARIANCE DIAGONAL P(n+1)+n: " << diagStateCov.transpose() << std::endl;
        PRNS("FINAL STATE X(n+1)+n: \n" << stateExp.transpose());
        PRNS("FINAL COVARIANCE DIAGONAL P(n+1)+n:  \n" << diagAnalysisStateCov.transpose());

        /// Correct ensemble members
        for (size_t index = 0; index < ensembleMembersNum; index++) {
            matXi.col(index) = stateExp;
        }
        matXi = matXi + matXiPerturb * matZsqrt;

        /// add inflation noise if needed
        if (d_additiveNoiseType.getValue() == 2) {
            for (size_t index = 0; index < ensembleMembersNum; index++) {
                matXi.col(index) = matXi.col(index) + modelNoise;
            }
            //std::cout << "We are in correction" << std::endl;
        }

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
        for (size_t index = 0; index < stateSize; index++) {
            stat[index] = stateExp[index];
            var[index] = stateCovar(index, index);
            for (size_t j = index + 1; j < stateSize; j++) {
                covar[gli++] = stateCovar(index, j);
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
    }
}



template <class FilterType>
void EnTKFilter<FilterType>::init() {
    Inherit::init();
    assert(this->gnode);

    this->gnode->template get<StochasticStateWrapperBaseT<FilterType> >(&stateWrappers, this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);
    PRNS("found " << stateWrappers.size() << " state wrappers");
    masterStateWrapper=NULL;
    size_t numSlaveWrappers = 0;
    size_t numMasterWrappers = 0;
    numThreads = 0;
    for (size_t i = 0; i < stateWrappers.size(); i++) {
        stateWrappers[i]->setFilterKind(LOCENSEMBLE);
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
void EnTKFilter<FilterType>::bwdInit() {
    assert(masterStateWrapper);

    stateSize = masterStateWrapper->getStateSize();
    std::cout<< "[LETKF] stateSize " << stateSize << std::endl;
    PRNS("StateSize " << stateSize);

    /// Initialize inversion type
    m_matrixInverse = ENSEMBLE_DIMENSION;
    std::string inverseOption = d_inverseOptionType.getValue();
    if (std::strcmp(inverseOption.c_str(), "ObservationsDimension") == 0)
        m_matrixInverse = OBSERVATIONS_DIMENSION;


    /// Initialize Observation's data
    if (!initialiseObservationsAtFirstStep.getValue()) {
        observationSize = this->observationManager->getObservationSize();
        PRNS("Observation size: " << observationSize);
        if (observationSize == 0) {
            PRNE("No observations available, cannot allocate the structures!");
        }
        obsCovarInv = observationManager->getErrorVarianceInverse();
        obsCovar = observationManager->getErrorVariance();
    }

    /// Initialize Model's Error Covariance
    stateCovar = masterStateWrapper->getStateErrorVariance();

    diagStateCov.resize(stateCovar.rows());
    for (size_t i = 0; i < (size_t)stateCovar.rows(); i++) {
        diagStateCov(i) = stateCovar(i,i);
    }
    PRNS(" INIT COVARIANCE DIAGONAL P(n+1)+n:  \n" << diagStateCov.transpose());

    modelNoise = masterStateWrapper->getModelElementNoise();
    PRNS(" INIT COVARIANCE DIAGONAL P(n+1)+n:  \n" << modelNoise);

    stateExp = masterStateWrapper->getState();

    helper::ReadAccessor<Data <size_t> > membersNum = d_ensembleMembersNumber;
    ensembleMembersNum = membersNum;

    matXi.resize(stateSize, ensembleMembersNum);
    matXi.setZero();
    stateAnalysisCovar.resize(ensembleMembersNum, ensembleMembersNum);
    diagAnalysisStateCov.resize(stateAnalysisCovar.rows());

    /// cretate gaussian random distribution
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 1.0);

    /// Generate ensemble members
    for (size_t index = 0; index < ensembleMembersNum; index++) {
        for (size_t stateIndex = 0; stateIndex < stateSize; stateIndex++) {
            matXi(stateIndex, index) = stateExp(stateIndex) + std::sqrt(diagStateCov(stateIndex)) * distribution(generator);
        }
    }

    /// Initialise State Observation Mapping for Sigma Points
    m_sigmaPointObservationIndexes.resize(ensembleMembersNum);
}

template <class FilterType>
void EnTKFilter<FilterType>::initializeStep(const core::ExecParams* _params, const size_t _step) {
    Inherit::initializeStep(_params, _step);

    if (initialiseObservationsAtFirstStep.getValue()) {
        observationSize = this->observationManager->getObservationSize();
        PRNS("Observation size: " << observationSize);
        if (observationSize == 0) {
            PRNE("No observations available, cannot allocate the structures!");
        }
        obsCovarInv = observationManager->getErrorVarianceInverse();
        obsCovar = observationManager->getErrorVariance();
        initialiseObservationsAtFirstStep.setValue(false);
    }

    for (size_t i = 0; i < stateWrappers.size(); i++)
        stateWrappers[i]->initializeStep(stepNumber);

    observationManager->initializeStep(stepNumber);
}



template <class FilterType>
void EnTKFilter<FilterType>::updateState() {

    stateSize = masterStateWrapper->getStateSize();
    //std::cout<< "new [UKF] stateSize " << stateSize << std::endl;
    //PRNS("StateSize " << stateSize);

    /// Initialize Model's Error Covariance
    stateCovar = masterStateWrapper->getStateErrorVariance();

    diagStateCov.resize(stateCovar.rows());
    for (size_t i = 0; i < (size_t)stateCovar.rows(); i++) {
        diagStateCov(i) = stateCovar(i,i);
    }
    //std::cout<< "INIT COVARIANCE DIAGONAL P(n+1)+n: " << diagStateCov.transpose() << std::endl;
    PRNS(" INIT COVARIANCE DIAGONAL P(n+1)+n:  \n" << diagStateCov.transpose());

    modelNoise = masterStateWrapper->getModelElementNoise();
    //std::cout<< "Model covariance: " << modelNoise << std::endl;
    PRNS(" INIT COVARIANCE DIAGONAL P(n+1)+n:  \n" << modelNoise);

    stateExp = masterStateWrapper->getState();

    helper::ReadAccessor<Data <size_t> > membersNum = d_ensembleMembersNumber;
    ensembleMembersNum = membersNum;

    matXi.resize(stateSize, ensembleMembersNum);
    matXi.setZero();

    /// Initialise State Observation Mapping for Sigma Points
    m_sigmaPointObservationIndexes.resize(ensembleMembersNum);
}



template <class FilterType>
void EnTKFilter<FilterType>::stabilizeMatrix (EMatrixX& _initial, EMatrixX& _stabilized) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(_initial, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singVals = svd.singularValues();
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singValsStab = singVals;
    for (int i=0; i < singVals.rows(); i++ ){
        if ((singValsStab(i)*singValsStab(i))*(1.0/(singVals(0)*singVals(0)))< 1.0e-6) singValsStab(i)=0;
    }
    _stabilized= svd.matrixU()*singValsStab*svd.matrixV().transpose();

}

template <class FilterType>
void EnTKFilter<FilterType>::pseudoInverse( EMatrixX& M,EMatrixX& pinvM) {
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
void EnTKFilter<FilterType>::sqrtMat(EMatrixX& A, EMatrixX& sqrtA){
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
void EnTKFilter<FilterType>::writeValidationPlot (std::string filename ,EVectorX& state ){
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

