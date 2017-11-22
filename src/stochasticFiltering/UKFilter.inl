#ifndef UKFilter_INL
#define UKFilter_INL

#include "UKFilter.h"
#include <iostream>
#include <fstream>

namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType, class mType>
UKFilter<FilterType, mType>::UKFilter()
    : Inherit()
    , observationErrorVarianceType( initData(&observationErrorVarianceType, std::string("inverse"), "observationErrorVarianceType", "if set to inverse, work directly with the inverse of the matrix" ) )
    , d_obsStdev( initData(&d_obsStdev, double(0.0), "obsStdev", "standard deviation of generated noise") )
    , d_stateStdev( initData(&d_stateStdev, double(0.0), "stateStdev", "standard deviation of generated noise") )
    , d_initModelVar( initData(&d_initModelVar, double(0.0), "initModelVar", "standard deviation of generated noise") )
    , d_estimParams( initData(&d_estimParams, 0, "estimParams", "parameters estimation type: 0 - force estimation, 1 - stiffness estimation") )
    , d_state( initData(&d_state, "state", "actual expected value of reduced state (parameters) estimated by the filter" ) )
    , d_variance( initData(&d_variance, "variance", "actual variance  of reduced state (parameters) estimated by the filter" ) )
    , d_covariance( initData(&d_covariance, "covariance", "actual co-variance  of reduced state (parameters) estimated by the filter" ) )
    , d_projectionMatrix( initData(&d_projectionMatrix, Mat3x4d(defaulttype::Vec<4,float>(1.0,0.0,0.0,0.0),
                                                                    defaulttype::Vec<4,float>(0.0,1.0,0.0,0.0),
                                                                    defaulttype::Vec<4,float>(0.0,0.0,1.0,0.0)), "projectionMatrix","Projection matrix"))
    , d_filename( initData(&d_filename, "filename", "output file name"))
    , outfile(NULL)
{

}

template <class FilterType, class mType>
UKFilter<FilterType,mType>::~UKFilter() {}

template < >
void UKFilter<double, Rigid3dTypes>::propagatePerturbedStates(EVectorX & _meanState) {

    EVectorX xCol(stateSize);
    matZmodel.resize(observationSize,sigmaPointsNum);
    helper::ReadAccessor<Data<VecCoord > > resetStockedposRA = mstate->readPositions();
    helper::ReadAccessor<Data<VecDeriv > > resetStockedvelRA = mstate->readVelocities();

    size_t stateSize = resetStockedposRA.size();
    VecCoord stockedPosition;
    VecDeriv stockedVelocity;
    stockedPosition.resize(stateSize);
    stockedVelocity.resize(stateSize);
    for (unsigned int i = 0; i < resetStockedposRA.size(); i++) {
        stockedPosition[i]=resetStockedposRA[i];
        stockedVelocity[i]=resetStockedvelRA[i];
    }
    const Mat3x4d & P = d_projectionMatrix.getValue();
    for (size_t i = 0; i < sigmaPointsNum; i++) {
        xCol = matXi.col(i);
        stateWrappers[0]->applyOperator(xCol, mechParams);
        matXi.col(i) = xCol;
        ///COMPUTE PREDICTED OBSERVATION !
        ReadVecCoord  pos = mstate->readPositions();
        helper::vector <Vector3> p;
        helper::vector <Vector2> V2D;
        p.resize(pos.size());
        V2D.resize(pos.size());
        for (unsigned int i = 0; i < pos.size(); i++){
            p[i] = pos[i].getCenter();

            double rx = P[0][0] * p[i][0] + P[0][1] * p[i][1] + P[0][2] * p[i][2] + P[0][3];
            double ry = P[1][0] * p[i][0] + P[1][1] * p[i][1] + P[1][2] * p[i][2] + P[1][3];
            double rz = P[2][0] * p[i][0] + P[2][1] * p[i][1] + P[2][2] * p[i][2] + P[2][3];

            V2D[i][0]=rx* (1.0/rz);
            V2D[i][1]=ry* (1.0/rz);
        }
        modelObservations.resize(observationSize);
        for (int i = 0; i < observationSize*(1./dim); i ++){
            for (int j= 0; j < dim; j++){
                modelObservations(dim*i+j)=V2D[i][j];}
        }
//        PRNS("sigma force = \n" << matXi);
//        PRNS("sigma obs = \n" << matZmodel.transpose());
        matZmodel.col(i) = modelObservations;
        helper::WriteAccessor<Data<VecCoord > > resetStockedpos = mstate->writePositions();
        helper::WriteAccessor<Data<VecDeriv > > resetStockedvel = mstate->writeVelocities();
        for (size_t ii = 0; ii < stateSize; ii++) {
            resetStockedpos[ii] = stockedPosition[ii];
            resetStockedvel[ii] = stockedVelocity[ii];
        }

    }
    _meanState = (1.0 / (double)sigmaPointsNum) * matXi.rowwise().sum();
}

template <>
void UKFilter<double, Vec3dTypes>::propagatePerturbedStates(EVectorX & _meanState) {

    EVectorX xCol(stateSize);
    //PRNS("stateSize: " << stateSize);
    matZmodel.resize(observationSize,sigmaPointsNum);

    if (d_estimParams.getValue() == 1) {
        // a case for estimating stiffnesses
        modelObservations.resize(observationSize);
        EVectorX innovation; // vector just to support software strcuture
        innovation.resize(observationSize);
        EVectorX predictedState; // vector just to support software strcuture
        innovation.resize(stateSize);
        stateWrappers[0]->holdCurrentState();
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            xCol = matXi.col(i);
            //PRNS("xCol: " << xCol);
            stateWrappers[0]->applyOperator(xCol, mechParams);
            matXi.col(i) = xCol;

            observationManager->predictedObservation(this->actualTime, predictedState, modelObservations, innovation);
            //PRNS("model observations values: " << modelObservations);
            matZmodel.col(i) = modelObservations;
            stateWrappers[0]->restoreState();
        }
    } else {

        helper::ReadAccessor<Data<VecCoord > > resetStockedposRA = mstate->readPositions();
        helper::ReadAccessor<Data<VecDeriv > > resetStockedvelRA = mstate->readVelocities();

        size_t stateSize = resetStockedposRA.size();
        VecCoord stockedPosition;
        VecDeriv stockedVelocity;
        stockedPosition.resize(stateSize);
        stockedVelocity.resize(stateSize);
        for (unsigned int i = 0; i < resetStockedposRA.size(); i++) {
            stockedPosition[i]=resetStockedposRA[i];
            stockedVelocity[i]=resetStockedvelRA[i];
        }
        const Mat3x4d & P = d_projectionMatrix.getValue();
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            xCol = matXi.col(i);
            stateWrappers[0]->applyOperator(xCol, mechParams);
            matXi.col(i) = xCol;
            ///COMPUTE PREDICTED OBSERVATION !
            helper::ReadAccessor<Data<VecCoord > >   pos = mstate->readPositions();
            helper::vector <Vector3> p;
            helper::vector <Vector2> V2D;
            p.resize(pos.size());
            V2D.resize(pos.size());
            for (unsigned int i = 0; i < pos.size(); i++){
                p[i] = pos[i];

                double rx = P[0][0] * p[i][0] + P[0][1] * p[i][1] + P[0][2] * p[i][2] + P[0][3];
                double ry = P[1][0] * p[i][0] + P[1][1] * p[i][1] + P[1][2] * p[i][2] + P[1][3];
                double rz = P[2][0] * p[i][0] + P[2][1] * p[i][1] + P[2][2] * p[i][2] + P[2][3];

                V2D[i][0]=rx* (1.0/rz);
                V2D[i][1]=ry* (1.0/rz);
            }
            modelObservations.resize(observationSize);
            for (int i = 0; i < observationSize*(1./dim); i ++){
                for (int j= 0; j < dim; j++){
                    modelObservations(dim*i+j)=V2D[i][j];}
            }

            matZmodel.col(i) = modelObservations;
            helper::WriteAccessor<Data<VecCoord > > resetStockedpos = mstate->writePositions();
            helper::WriteAccessor<Data<VecDeriv > > resetStockedvel = mstate->writeVelocities();
            for (size_t ii = 0; ii < stateSize; ii++) {
                resetStockedpos[ii] = stockedPosition[ii];
                resetStockedvel[ii] = stockedVelocity[ii];
            }

        }
    }
    _meanState = (1.0 / (double)sigmaPointsNum) * matXi.rowwise().sum();
}

template <class FilterType, class mType>
void UKFilter<FilterType, mType>::computePrediction()
{
    PRNS("Computing prediction, T= " << this->actualTime);

    /// Computes background error variance Cholesky factorization.
    Eigen::LLT<EMatrixX> lltU(matP);
    matPsqrt = lltU.matrixL();
    //PRNS("matP: " << matP);

    /// Computes X_{n}^{(i)-} sigma points
    EVectorX vecX = masterStateWrapper->getState();
    for (size_t i = 0; i < sigmaPointsNum; i++) {
        matXi.col(i) = vecX + matPsqrt * matI.row(i).transpose();
    }
    //PRNS("matXi: " << matXi);

    propagatePerturbedStates(vecX);
    /// Computes stateCovariance P_ = cov(X_{n + 1}^*, X_{n + 1}^*).
    EMatrixX matXiTrans= matXi.transpose();
    EMatrixX centeredPxx = matXiTrans.rowwise() - matXiTrans.colwise().mean();
    EMatrixX covPxx = (centeredPxx.adjoint() * centeredPxx) / double(centeredPxx.rows() - 1);
    matP=covPxx;
    matP=matP+matQ;

    masterStateWrapper->setState(vecX, this->mechParams);
    masterStateWrapper->writeState(this->getTime());
}

template <class FilterType, class mType>
void UKFilter<FilterType, mType>::computeCorrection()
{

        PRNS("Computing correction, T= " << this->actualTime);

        if (observationManager->hasObservation(this->actualTime)) {

    //        /// Computes the observation sigmaPoint Z_{n + 1}^(i). ****CODE FOR POSITION OR VELOCITy ESTIMATION***
    //        EVectorX vecXCol;
    //        EMatrixX matZItrans(sigmaPointsNum, observationSize);

            //        for (size_t i = 0; i < sigmaPointsNum; i++) {
            //            EVectorX vecZCol;
            //            vecXCol = matXi.col(i);
            //            observationManager->predictedObservation(this->actualTime, vecXCol,obsPrec, vecZCol);
            //            matZItrans.row(i) = vecZCol;
            //        }

            /// Computes the predicted measurement Z_{n + 1}.
            EMatrixX matZItrans = matZmodel.transpose();
            //PRNS("matZItrans: " << matZItrans);
            EVectorX Z_mean = (1.0/(double) sigmaPointsNum) * matZItrans.colwise().sum();
            obsPrec = Z_mean;

            /// Computes X_{n+1}-
            EMatrixX matXiTrans= matXi.transpose();
            //PRNS("matXi: " << matXi);

            /// Computes crosscovariance P_XZ = cov(X_{n + 1}^*, Z_{n + 1}^*)
            matPxz.resize(stateSize,observationSize);
            EMatrixX centeredCx = matXiTrans.rowwise() - matXiTrans.colwise().mean();
            //PRNS("centeredCx: " << centeredCx);
            EMatrixX centeredCz = matZItrans.rowwise() - matZItrans.colwise().mean();
            //PRNS("centeredCz: " << centeredCz);
            EMatrixX covPxz = (centeredCx.adjoint() * centeredCz) / double(centeredCx.rows() - 1);
            matPxz=covPxz;

            /// Computes P_Z = cov(Z_{n + 1}^*, Z_{n + 1}^*).
            matPzz.resize(observationSize,observationSize);
            EMatrixX covPzz = (centeredCz.adjoint() * centeredCz) / double(centeredCz.rows() - 1);
            matPzz=covPzz;
            matPzz= matR+ matPzz;
            //PRNS("matR: " << matR);
            //PRNS("matPxz: " << matPxz);
            //PRNS("covPzz: " << covPzz);

            ///  Computes the Kalman gain K_{n + 1}.
            EMatrixX matK(stateSize, observationSize);
            matK =matPxz* matPzz.inverse();
            //PRNS("matK: " << matK);

            /// Computes X_{n + 1}^+.
            EVectorX state = masterStateWrapper->getState();
            EVectorX Innovation(observationSize);

            observationManager->getInnovation(this->actualTime, Z_mean, Innovation);
            //PRNS("Z_mean values: " << Z_mean);
            //PRNS("state values: " << state);
            state = state + matK*Innovation;
            //PRNS("Innovation size: " << Innovation.size());
            //PRNS("Innovation values: " << Innovation);
            //PRNS("state values: " << state);

            ///  Computes P_{n + 1}^+.
            matP = matP - matK*matPxz.transpose();

            masterStateWrapper->setState(state, this->mechParams);
            masterStateWrapper->applyOperator(state, mechParams);

            if (d_estimParams.getValue() == 1) {
                helper::WriteAccessor<Data <helper::vector<FilterType> > > stat = d_state;
                helper::WriteAccessor<Data <helper::vector<FilterType> > > var = d_variance;
                helper::WriteAccessor<Data <helper::vector<FilterType> > > covar = d_covariance;

                stat.resize(stateSize);
                var.resize(stateSize);
                size_t numCovariances = (stateSize*(stateSize-1))/2;
                covar.resize(numCovariances);

                size_t gli = 0;
                for (size_t i = 0; i < stateSize; i++) {
                    stat[i] = state[i];
                    var[i] = matP(i,i);
                    for (size_t j = i+1; j < stateSize; j++) {
                        covar[gli++] = matP(i,j);
                    }
                }

            }

            if (outfile)
            {
                    (*outfile) << "T= "<< this->actualTime<< "\n";
                    {
                        (*outfile) << "  X= ";
                        this->mstate->writeVec(core::VecId::position(), *outfile);
                        (*outfile) << "\n";
                    }
                outfile->flush();
            }
        }
}

template <class FilterType, class mType>
void UKFilter<FilterType, mType>::init() {
    Inherit::init();
    assert(this->gnode);
    this->getContext()->get(observationSource,sofa::core::objectmodel::BaseContext::SearchDown);
    if(observationSource == NULL )
        PRNE("observationSource not found");
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

    const std::string& filename = d_filename.getFullPath();
    outfile = new std::ofstream(filename.c_str());
}

template <class FilterType, class mType>
void UKFilter<FilterType, mType>::bwdInit() {
    PRNS("bwdInit");
    assert(masterStateWrapper);
    observationSize = this->observationManager->getObservationSize();
    dim = this->observationSource->getObsDimention();
    //PRNS("observationSize " << observationSize);
    if (observationSize == 0) {
        PRNE("No observations available, cannot allocate the structures!");
    }
    stateSize = masterStateWrapper->getStateSize();
    //PRNS("stateSize " << stateSize);
    /// Initialize Observation's Error Covariance
    if (d_estimParams.getValue() == 1) {
        matR = observationManager->getErrorVariance();
    } else {
        errorVarianceValue = d_obsStdev.getValue() * d_obsStdev.getValue();
        matR = EMatrixX::Identity(observationSize, observationSize)*errorVarianceValue;
    }

    /// Initialize Model's Error Covariance
    stateVarianceValue = d_stateStdev.getValue() * d_stateStdev.getValue();
    matQ = EMatrixX::Identity(stateSize, stateSize) * stateVarianceValue;


    /// Initialize Model's Covariance
    if (d_estimParams.getValue() == 1) {
        matP = masterStateWrapper->getStateErrorVarianceDevUKF();
    } else {
        matP=EMatrixX::Identity(stateSize, stateSize)*d_initModelVar.getValue();
    }
    //PRNS("matP: " << matP);

    /// compute sigma points
    computeSimplexSigmaPoints(matI);
    sigmaPointsNum = matI.rows();
    matXi.resize(stateSize, sigmaPointsNum);
    matXi.setZero();


    /// Initialize Precedent Observation for case estimatePostion=0 && estimateVelocity=1
    obsPrec.resize(observationSize);
    obsPrec.setZero();


    /// Stock mstate Position if estimatingVelocity
    this->masterStateWrapper->getContext()->get(mstate);
    masterStateWrapper->writeState(double(0.0));

}

template <class FilterType, class mType>
void UKFilter<FilterType, mType>::initializeStep(const core::ExecParams* _params, const size_t _step) {
    Inherit::initializeStep(_params, _step);

    for (size_t i = 0; i < stateWrappers.size(); i++)
        stateWrappers[i]->initializeStep(stepNumber);

    observationManager->initializeStep(stepNumber);
}

template <class FilterType, class mType>
void UKFilter<FilterType, mType>::computeSimplexSigmaPoints(EMatrixX& sigmaMat) {


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
