#ifndef UKFilterClassic_INL
#define UKFilterClassic_INL

#include "UKFilterClassic.h"
#include <iostream>
#include <fstream>

namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType>
UKFilterClassic<FilterType>::UKFilterClassic()
    : Inherit()
    , d_exportPrefix( initData(&d_exportPrefix, "exportPrefix", "prefix for storing various quantities into files"))
    , d_state( initData(&d_state, "state", "actual expected value of reduced state (parameters) estimated by the filter" ) )
    , d_variance( initData(&d_variance, "variance", "actual variance  of reduced state (parameters) estimated by the filter" ) )
    , d_covariance( initData(&d_covariance, "covariance", "actual co-variance  of reduced state (parameters) estimated by the filter" ) )
    , d_innovation( initData(&d_innovation, "innovation", "innovation value computed by the filter" ) )
{

}

template <class FilterType>
void UKFilterClassic<FilterType>::computePerturbedStates()
{
    EVectorX xCol(stateSize);
    int currentPointID;

    for (size_t i = 0; i < sigmaPointsNum; i++) {
        xCol = matXi.col(i);
        stateWrappers[0]->transformState(xCol, mechParams, &currentPointID);
        m_sigmaPointObservationIndexes[i] = currentPointID;
        //if (i < 10)
        //    PRNS("X: " << xCol.transpose());
        matXi.col(i) = xCol;
    }
}

template <class FilterType>
void UKFilterClassic<FilterType>::computePrediction()
{    
    PRNS("Computing prediction, T= " << this->actualTime  << " ======");
    /// Computes background error variance Cholesky factorization.
    Eigen::LLT<EMatrixX> lltU(stateCovar);
    EMatrixX matPsqrt = lltU.matrixL();

    //PRNS("matPsqrt: " << matPsqrt);

    stateExp.fill(FilterType(0.0));
    stateExp = masterStateWrapper->getState();
    //PRNS("X(n): \n" << stateExp.transpose());
    //PRNS("P(n): \n" << stateCovar);

    /// Computes X_{n}^{(i)-} sigma points        
    for (size_t i = 0; i < sigmaPointsNum; i++) {
        matXi.col(i) = stateExp + matPsqrt * matI.row(i).transpose();
    }
    //PRNS("matI: \n" << matI);
    //PRNS("MatXi: \n" << matXi);

    /// Compute the state
    computePerturbedStates();

    /// compute the expected value
    stateExp.fill(FilterType(0.0));
    for (size_t i = 0; i < sigmaPointsNum; i++)
        stateExp += matXi.col(i);
    stateExp = stateExp * alpha;

    stateCovar.fill(FilterType(0.0));
    EVectorX tmpX(stateSize);
    tmpX.fill(FilterType(0.0));

    for (size_t i = 0; i < sigmaPointsNum; i++) {
       tmpX = matXi.col(i) - stateExp;
       for (size_t x = 0; x < stateSize; x++)
           for (size_t y = 0; y < stateSize; y++)
               stateCovar(x,y) += tmpX(x)*tmpX(y);
    }
    stateCovar = alpha*stateCovar;

    //PRNS("X(n+1)-: " << stateExp.transpose());
    //PRNS("P(n+1)-: \n" << stateCovar);
}

template <class FilterType>
void UKFilterClassic<FilterType>::computeCorrection()
{            
    if (observationManager->hasObservation(this->actualTime)) {
        PRNS("======= Computing correction, T= " << this->actualTime << " ======");

        EVectorX zCol(observationSize);
        matZmodel.resize(observationSize, sigmaPointsNum);
        predObsExp.resize(observationSize);
        predObsExp.fill(FilterType(0.0));

        /// compute predicted observations
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            observationManager->getPredictedObservation(this->actualTime, m_sigmaPointObservationIndexes[i],  zCol);
            //PRNS("Zcol: \n" << m_sigmaPointObservationIndexes[i]);
            //PRNS("Zcol: \n" << zCol.transpose());
            matZmodel.col(i) = zCol;
            predObsExp = predObsExp + zCol;

        }
        //PRNS("Z: \n" << matZmodel);
        predObsExp = alpha*predObsExp;

        EMatrixX matPxz(stateSize, observationSize);
        EMatrixX matPz(observationSize, observationSize);
        matPxz.fill(FilterType(0.0));
        matPz.fill(FilterType(0.0));

        EVectorX vx(stateSize), z(observationSize), vz(observationSize);
        vx.fill(FilterType(0.0));
        vz.fill(FilterType(0.0));
        z.fill(FilterType(0.0));
        for (size_t i = 0; i < sigmaPointsNum; i++) {
           vx = matXi.col(i) - stateExp;
           z = matZmodel.col(i);
           vz = z - predObsExp;
           for (size_t x = 0; x < stateSize; x++)
               for (size_t y = 0; y < observationSize; y++)
                   matPxz(x,y) += vx(x)*vz(y);

           for (size_t x = 0; x < observationSize; x++)
               for (size_t y = 0; y < observationSize; y++)
                   matPz(x,y) += vz(x)*vz(y);
        }
        matPxz = alpha * matPxz;
        matPz = alpha * matPz + obsCovar;
        //PRNS("matPxz: " << matPxz);
        //PRNS("matPz: " << matPz);
        //PRNS("ObsCovar: " << obsCovar);

        EMatrixX matK(stateSize, observationSize);
        matK = matPxz*matPz.inverse();
        //PRNS("matK: " << matK);

        EVectorX innovation(observationSize);
        observationManager->getInnovation(this->actualTime, predObsExp, innovation);

        //PRNS("innovation: " << innovation);
        stateExp = stateExp + matK * innovation;
        stateCovar = stateCovar - matK*matPxz.transpose();

        //stateWrappers[0]->computeSimulationStep(stateExp, mechParams, id);

        //PRNS("X(n+1)+: \n" << stateExp.transpose());
        //PRNS("P(n+1)+n: \n" << stateCovar);

        masterStateWrapper->setState(stateExp, mechParams);
        //masterStateWrapper->transformState(stateExp, mechParams);

        helper::WriteAccessor<Data <helper::vector<FilterType> > > stat = d_state;
        helper::WriteAccessor<Data <helper::vector<FilterType> > > var = d_variance;
        //helper::WriteAccessor<Data <helper::vector<FilterType> > > covar = d_covariance;
        helper::WriteAccessor<Data <helper::vector<FilterType> > > innov = d_innovation;

        stat.resize(stateSize);
        var.resize(stateSize);        
        //size_t numCovariances = (stateSize*(stateSize-1))/2;
        //covar.resize(numCovariances);
        innov.resize(observationSize);

        //size_t gli = 0;
        for (size_t i = 0; i < stateSize; i++) {
            stat[i] = stateExp[i];
            var[i] = stateCovar(i,i);
            //for (size_t j = i+1; j < stateSize; j++) {
            //    covar[gli++] = stateCovar(i,j);
            //}
        }
        for (size_t index = 0; index < observationSize; index++) {
            innov[index] = innovation[index];
        }

        char nstepc[100];
        sprintf(nstepc, "%04d", stepNumber);
        if (! exportPrefix.empty()) {
            std::string fileName = exportPrefix + "/covar_" + nstepc + ".txt";
            std::ofstream ofs;
            ofs.open(fileName.c_str(), std::ofstream::out);
            ofs << stateCovar << std::endl;
        }

    }
}

template <class FilterType>
void UKFilterClassic<FilterType>::init() {
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

    exportPrefix  = d_exportPrefix.getFullPath();
    PRNS("export prefix: " << exportPrefix)

}

template <class FilterType>
void UKFilterClassic<FilterType>::bwdInit() {
    assert(masterStateWrapper);
    observationSize = this->observationManager->getObservationSize();
    PRNS("Observation size: " << observationSize);
    if (observationSize == 0) {
        PRNE("No observations available, cannot allocate the structures!");
    }
    stateSize = masterStateWrapper->getStateSize();
    PRNS("StateSize " << stateSize);
    /// Initialize Observation's Error Covariance    
    obsCovar = observationManager->getErrorVariance();

    /// Initialize Model's Error Covariance
    stateCovar = masterStateWrapper->getStateErrorVariance();
    stateExp = masterStateWrapper->getState();

    /// compute sigma points
    computeSimplexSigmaPoints(matI);
    sigmaPointsNum = matI.rows();
    matXi.resize(stateSize, sigmaPointsNum);
    matXi.setZero();

    /// initialise state observation mapping for sigma points
    m_sigmaPointObservationIndexes.resize(sigmaPointsNum);
}

template <class FilterType>
void UKFilterClassic<FilterType>::initializeStep(const core::ExecParams* _params, const size_t _step) {
    Inherit::initializeStep(_params, _step);

    for (size_t i = 0; i < stateWrappers.size(); i++)
        stateWrappers[i]->initializeStep(stepNumber);

    observationManager->initializeStep(stepNumber);
}

template <class FilterType>
void UKFilterClassic<FilterType>::computeSimplexSigmaPoints(EMatrixX& sigmaMat) {
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
    alpha = vecAlpha(0);
}


} // stochastic
} // component
} // sofa

#endif // UKFilterClassic_INL
