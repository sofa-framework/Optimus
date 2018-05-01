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
    , reducedState( initData(&reducedState, "reducedState", "actual expected value of reduced state (parameters) estimated by the filter" ) )
    , reducedVariance( initData(&reducedVariance, "reducedVariance", "actual variance  of reduced state (parameters) estimated by the filter" ) )
    , reducedCovariance( initData(&reducedCovariance, "reducedCovariance", "actual co-variance  of reduced state (parameters) estimated by the filter" ) )
    , d_reducedInnovation( initData(&d_reducedInnovation, "reducedInnovation", "innovation value computed by the filter" ) )    
{    
    this->reducedOrder.setValue(true);
}

template <class FilterType>
ROUKFilter<FilterType>::~ROUKFilter() {}

template <class FilterType>
void ROUKFilter<FilterType>::computePerturbedStates(EVectorX& _meanState) {
    if (numThreads == 1) {
        EVectorX xCol(stateSize);
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            xCol = matXi.col(i);
            //std::cout << "Before simul:" << xCol(stateSize-3) << " " << xCol(stateSize-2) << " " << xCol(stateSize-1) << std::endl;
            stateWrappers[sigmaPoints2WrapperIDs[i]]->transformState(xCol, mechParams);
            //std::cout << "After simul:" << xCol(stateSize-3) << " " << xCol(stateSize-2) << " " << xCol(stateSize-1) << std::endl;
            matXi.col(i) = xCol;

            /*char fileName[100];
            sprintf(fileName, "sigma_%03d_%03d", this->stepNumber, i);
            std::ofstream cvmFile;
            cvmFile.open(fileName);
            cvmFile << xCol << std::endl;
            cvmFile.close();*/
        }
        _meanState = alpha * matXi.rowwise().sum();
    } else {
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
    }
}

template <class FilterType>
void ROUKFilter<FilterType>::computePrediction()
{
    //PRNS("Computing prediction, T= " << this->actualTime);
    sofa::helper::AdvancedTimer::stepBegin("ROUKFPrediction");

    EMatrixX tmpStateVarProj(stateSize, reducedStateSize);
    EMatrixX tmpStateVarProj2(stateSize, reducedStateSize);

    //TIC
    sofa::helper::AdvancedTimer::stepBegin("Cholseky_prediction");

    EVectorX vecX = masterStateWrapper->getState();
    //std::cout << "State: " << vecX.transpose() << std::endl;
    Eigen::LLT<EMatrixX> lltU(matUinv);
    matUinv = lltU.matrixL();
    //TOC("== prediction: Cholesky ");
    sofa::helper::AdvancedTimer::stepEnd("Cholseky_prediction");

    tmpStateVarProj = masterStateWrapper->getStateErrorVarianceProjector();

    //TIC
    sofa::helper::AdvancedTimer::stepBegin("prediction_multiplication");
    if (useBlasToMultiply.getValue())
        blasMultAdd(tmpStateVarProj, matUinv, tmpStateVarProj2, 1.0, 0.0);
    else
        tmpStateVarProj2.noalias() = tmpStateVarProj * matUinv;
    //TOC("== prediction multiplication1 == ");

    masterStateWrapper->setStateErrorVarianceProjector(tmpStateVarProj2);
    //PRNS("\n vecX\n " << vecX.transpose());
    for (size_t i = 0; i < sigmaPointsNum; i++)
        matXi.col(i) = vecX;

    //TIC
    if (useBlasToMultiply.getValue())
        blasMultAdd(tmpStateVarProj2, matI, matXi, 1.0, 1.0);
    else
        matXi = matXi + tmpStateVarProj2 * matI;
    //TOC("== prediction multiplication2 == ");
    sofa::helper::AdvancedTimer::stepEnd("prediction_multiplication");

    //TIC;
    sofa::helper::AdvancedTimer::stepBegin("transform_state");
    computePerturbedStates(vecX);   
    //asumEVec("summPred",vecX);
    //TOCTIC("== prediction compute perturbations ==");
    sofa::helper::AdvancedTimer::stepEnd("transform_state");

    /// TODO: add resampling!!!
    if (useBlasToMultiply.getValue())
        blasMultAdd(matXi, matItrans, tmpStateVarProj2, alpha, 0.0);
    else
        tmpStateVarProj2 = alpha*matXi * matItrans;
    //TOC("== prediction multiplication3 ==");
    //PRNS("\n errorVarProj \n " << matItrans.transpose());

    masterStateWrapper->setStateErrorVarianceProjector(tmpStateVarProj2);
    masterStateWrapper->setState(vecX, this->mechParams);
    masterStateWrapper->writeState(this->getTime());

    sofa::helper::AdvancedTimer::stepEnd("ROUKFPrediction");
}


template <class FilterType>
void ROUKFilter<FilterType>::computeCorrection()
{
    //PRNS("Computing correction, T= " << this->actualTime);
    sofa::helper::AdvancedTimer::stepBegin("ROUKFCorrection");

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
        //asumEMat("correction input mat",matXi);

        sofa::helper::AdvancedTimer::stepBegin("Innovation");

        for (size_t i = 0; i < sigmaPointsNum; i++) {
            vecXCol = matXi.col(i);
            vecZCol.setZero();
            observationManager->getInnovation(this->actualTime, vecXCol, vecZCol);
            vecZ = vecZ + alpha * vecZCol;
            matZItrans.row(i) = vecZCol;
        }
        //TOCTIC("== an1sx == ");
        sofa::helper::AdvancedTimer::stepEnd("Innovation");
        //asumEVec("correction accumInnov",vecZ);

        EMatrixX matHLtrans(reducedStateSize, observationSize);
        matHLtrans = alphaVar*matItrans.transpose()*matZItrans;
        //PRNS("\n alphaVar\n " << alphaVar);
        //PRNS("\n matItrans\n " << matItrans);
        //PRNS("\n matZItrans\n " << matZItrans);
        //PRNS("\n mult\n " << matItrans.transpose()*matZItrans);
        //asumEMat("HL_trans", matHLtrans);
        //PRNS("\n matHLtrans\n " << matHLtrans);

        EMatrixX matWorkingPO(reducedStateSize, observationSize), matTemp;
        if (observationErrorVarianceType.getValue() == "inverse") {
            matWorkingPO = matHLtrans * observationManager->getErrorVarianceInverse();
        } else {
            EMatrixX matR;
            matR = observationManager->getErrorVariance();
            matWorkingPO = matHLtrans * matR.inverse();
        }        
        matTemp = EMatrixX::Identity(matUinv.rows(), matUinv.cols()) + matWorkingPO * matHLtrans.transpose();
        //TOCTIC("== an3sx == ");
        matUinv = matTemp.inverse();
        //TOCTIC("== an4sx == (inv) ");

        EVectorX reducedInnovation(reducedStateSize);
        reducedInnovation = Type(-1.0) * matUinv*matWorkingPO*vecZ;
        //asumEMat("matUinv", matUinv);
        //PRNS("\n matUinv\n " << matUinv);
        //asumEMat("matWorkingPO", matWorkingPO);
        //PRNS("\n matWorkingPO\n " << matWorkingPO);
        //asumEVec("reduced innovation", reducedInnovation);
        //PRNS("\n vecZ \n " << vecZ);

        EVectorX state = masterStateWrapper->getState();
        //PRNS("state\n " << state.transpose());
        EMatrixX errorVarProj = masterStateWrapper->getStateErrorVarianceProjector();
        state = state + errorVarProj*reducedInnovation;
        //PRNS("\n errorVarProj \n " << errorVarProj.transpose());
        //PRNS("\n reducedInnovation \n " << reducedInnovation);
        masterStateWrapper->setState(state,mechParams);
        //PRNS("\n state+ \n" << state.transpose());


        //std::cout << "FST " << this->stepNumber << " " << this->actualTime << std::endl;

        size_t reducedStateIndex = stateSize - reducedStateSize;
        EMatrixX parErrorVarProj(reducedStateSize,reducedStateSize);

        for (size_t i = 0; i < reducedStateSize; i++)
            for (size_t j = 0; j < reducedStateSize; j++)
                parErrorVarProj(i,j) = errorVarProj(reducedStateIndex+i,j);

        EMatrixX covarianceMatrix(reducedStateSize, reducedStateSize);
        covarianceMatrix = parErrorVarProj * matUinv * parErrorVarProj.transpose();

        helper::WriteAccessor<Data <helper::vector<FilterType> > > redState = reducedState;
        helper::WriteAccessor<Data <helper::vector<FilterType> > > redVar = reducedVariance;
        helper::WriteAccessor<Data <helper::vector<FilterType> > > redCovar = reducedCovariance;
        helper::WriteAccessor<Data <helper::vector<FilterType> > > innov = d_reducedInnovation;

        redState.resize(reducedStateSize);
        redVar.resize(reducedStateSize);
        size_t numCovariances = (reducedStateSize*(reducedStateSize-1))/2;
        redCovar.resize(numCovariances);
        innov.resize(observationSize);

        size_t gli = 0;
        for (size_t i = 0; i < reducedStateSize; i++) {
            redState[i] = state[reducedStateIndex+i];
            redVar[i] = covarianceMatrix(i,i);
            for (size_t j = i+1; j < reducedStateSize; j++) {
                redCovar[gli++] = covarianceMatrix(i,j);
            }
        }
        for (size_t index = 0; index < observationSize; index++) {
            innov[index] = vecZ[index];
        }

        /*char fileName[100];
        sprintf(fileName, "outVar/parL_%03d.txt", this->stepNumber);
        std::ofstream cvmFile;
        cvmFile.open(fileName);
        cvmFile << parErrorVarProj << std::endl;
        cvmFile.close();

        sprintf(fileName, "outVar/invU_%03d.txt", this->stepNumber);
        cvmFile.open(fileName);
        cvmFile << matUinv << std::endl;
        cvmFile.close();

        sprintf(fileName, "outVar/covar_%03d.txt", this->stepNumber);
        cvmFile.open(fileName);
        cvmFile << covarianceMatrix << std::endl;
        cvmFile.close();*/

        //TOC("== an5sx == ");
        /*Type maxState = 0.0, minState = 1e10;
        for (size_t i = stateSize - reducedStateSize; i < stateSize; i++) {
            maxState = (state(i) > maxState) ? state(i) : maxState;
            minState = (state(i) < minState) ? state(i) : minState;
        }

        asumEVec("############# final state", state);
        std::cout << "Max = " << maxState << " min = " << minState << std::endl;*/

        sofa::helper::AdvancedTimer::stepEnd("ROUKFCorrection");
    }
}

template <class FilterType>
void ROUKFilter<FilterType>::init() {
    Inherit::init();
    assert(this->gnode);
    this->gnode->template get<StochasticStateWrapperBaseT<FilterType> >(&stateWrappers, this->getTags(), sofa::core::objectmodel::BaseContext::SearchDown);
    PRNS("found " << stateWrappers.size() << " state wrappers");
    masterStateWrapper=NULL;
    size_t numSlaveWrappers = 0;
    size_t numMasterWrappers = 0;
    numThreads = 0;
    for (size_t i = 0; i < stateWrappers.size(); i++) {
        stateWrappers[i]->setFilterKind(REDORD);
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
void ROUKFilter<FilterType>::bwdInit() {
    PRNS("bwdInit");
    assert(masterStateWrapper);

    stateSize = masterStateWrapper->getStateSize();
    matU = masterStateWrapper->getStateErrorVarianceReduced();

    reducedStateSize = matU.cols();
    matUinv = matU.inverse();   

    //PRNW("size: " << matU.rows() << " X " << matU.cols());

    /// compute sigma points
    EMatrixX matVtrans;
    /// compute sigma points
    switch (this->m_sigmaTopology) {
    case STAR:
        computeStarSigmaPoints(matVtrans);
        break;
    case SIMPLEX:
    default:
        computeSimplexSigmaPoints(matVtrans);
    }
    sigmaPointsNum = matVtrans.rows();

    PRNS("State size: " << stateSize);
    PRNS("Reduced state size: " << reducedStateSize);
    PRNS("Number of sigma points: " << sigmaPointsNum);

    // initialise observation data
    if (!initialiseObservationsAtFirstStep.getValue()) {
        observationSize = this->observationManager->getObservationSize();
        PRNS("Observation size: " << observationSize);
    }

    EMatrixX matPalphaV(reducedStateSize, reducedStateSize);
    matItrans.resize(sigmaPointsNum, reducedStateSize);

    if (alphaConstant) {
        matPalphaV = alphaVar*matVtrans.transpose()*matVtrans;
        matPalphaV = matPalphaV.inverse();
        matItrans = matVtrans*matPalphaV.transpose();
    } else {
        PRNE(" simplex method implemented only for constant alpha");
        return;
    }
    matI = matItrans.transpose();

    matDv.resize(sigmaPointsNum, sigmaPointsNum);
    matDv = alphaVar*alphaVar*matItrans*matI;

    matXi.resize(stateSize, sigmaPointsNum);

    /// prepare structures for parallel computing
    sigmaPoints2WrapperIDs.resize(sigmaPointsNum, 0);
    wrapper2SigmaPointsIDs.resize(numThreads);
    for (size_t i = 0; i < sigmaPointsNum; i++) {
        size_t threadID = i%(numThreads);
        sigmaPoints2WrapperIDs[i] = threadID;
        wrapper2SigmaPointsIDs[threadID].push_back(i);
    }

    masterStateWrapper->writeState(double(0.0));


    /// export initial stochastic state
    helper::WriteAccessor<Data <helper::vector<FilterType> > > redState = reducedState;
    helper::WriteAccessor<Data <helper::vector<FilterType> > > redVar = reducedVariance;
    helper::WriteAccessor<Data <helper::vector<FilterType> > > redCovar = reducedCovariance;

    EVectorX state = masterStateWrapper->getState();
    EMatrixX errorVarProj = masterStateWrapper->getStateErrorVarianceProjector();
    size_t reducedStateIndex = stateSize - reducedStateSize;
    EMatrixX parErrorVarProj(reducedStateSize,reducedStateSize);
    for (size_t i = 0; i < reducedStateSize; i++)
        for (size_t j = 0; j < reducedStateSize; j++)
            parErrorVarProj(i,j) = errorVarProj(reducedStateIndex+i,j);
    EMatrixX covarianceMatrix(reducedStateSize, reducedStateSize);
    covarianceMatrix = parErrorVarProj * matUinv * parErrorVarProj.transpose();

    redState.resize(reducedStateSize);
    redVar.resize(reducedStateSize);
    size_t numCovariances = (reducedStateSize*(reducedStateSize-1))/2;
    redCovar.resize(numCovariances);

    size_t gli = 0;
    for (size_t i = 0; i < reducedStateSize; i++) {
        redState[i] = state[reducedStateIndex+i];
        redVar[i] = covarianceMatrix(i,i);
        for (size_t j = i+1; j < reducedStateSize; j++) {
            redCovar[gli++] = covarianceMatrix(i,j);
        }
    }


    //std::cout << "SigmaWrapper: " << sigmaPoints2WrapperIDs << std::endl;
    //std::cout << "WrapperSigma: " << wrapper2SigmaPointsIDs << std::endl;
}

template <class FilterType>
void ROUKFilter<FilterType>::initializeStep(const core::ExecParams* _params, const size_t _step) {
    Inherit::initializeStep(_params, _step);

    if (initialiseObservationsAtFirstStep.getValue()) {
        observationSize = this->observationManager->getObservationSize();
        PRNS("Observation size: " << observationSize);
        initialiseObservationsAtFirstStep.setValue(false);
    }

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

    alpha = Type(1.0)/Type(r);
    alphaVar = (this->useUnbiasedVariance.getValue()) ? Type(1.0)/Type(r-1) : Type(1.0)/Type(r);
    vecAlphaVar.resize(r);
    vecAlphaVar.fill(alphaVar);
}

template <class FilterType>
void ROUKFilter<FilterType>::computeStarSigmaPoints(EMatrixX& sigmaMat) {
    size_t p = reducedStateSize;
    size_t r = 2 * reducedStateSize + 1;

    EMatrixX workingMatrix = EMatrixX::Zero(p, r);

    Type lambda, sqrt_vec;
    lambda = this->lambdaScale.getValue();
    PRNS("lambda: \n" << lambda);
    sqrt_vec = sqrt(p + lambda);

    for (size_t j = 0; j < p; j++) {
        workingMatrix(j,j) = sqrt_vec;
    }
    for (size_t j = p; j < 2 * p; j++) {
        workingMatrix(2*p-j-1,j) = -sqrt_vec;
    }

    sigmaMat.resize(r,p);
    for (size_t i = 0; i < r; i++)
        sigmaMat.row(i) = workingMatrix.col(i);

    vecAlpha.resize(r);
    for (size_t i = 0; i < 2 * p; i++) {
        vecAlpha(i) = Type(1.0)/Type(r);
    }
    vecAlpha(2 * p) = Type(1.0)/Type(r);
    alphaConstant = true;

    vecAlphaVar.resize(r);
    if (this->useUnbiasedVariance.getValue()) {
        for (size_t i = 0; i < 2 * p; i++) {
            vecAlphaVar(i) = Type(1.0)/Type(r - 1);
        }
        // double beta = 2.0;
        vecAlphaVar(2 * p) = Type(1.0)/Type(r - 1);

        alphaVar = Type(1.0)/Type(r-1);
    } else {
        for (size_t i = 0; i < 2 * p; i++) {
            vecAlphaVar(i) = Type(1.0)/Type(r);
        }
        // double beta = 2.0;
        vecAlphaVar(2 * p) = Type(1.0)/Type(r);

        alphaVar = Type(1.0)/Type(r);
    }
    //PRNS("sigmaMat: \n" << sigmaMat);
    //PRNS("vecAlphaVar: \n" << vecAlphaVar);
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
