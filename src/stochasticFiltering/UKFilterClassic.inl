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
    , d_filenameCov( initData(&d_filenameCov, "filenameCov", "output file name"))
    , d_filenameInn( initData(&d_filenameInn, "filenameInn", "output file name"))
    , d_filenameFinalState( initData(&d_filenameFinalState, "filenameFinalState", "output file name"))
    , d_state( initData(&d_state, "state", "actual expected value of reduced state (parameters) estimated by the filter" ) )
    , d_variance( initData(&d_variance, "variance", "actual variance  of reduced state (parameters) estimated by the filter" ) )
    , d_covariance( initData(&d_covariance, "covariance", "actual co-variance  of reduced state (parameters) estimated by the filter" ) )
    , d_innovation( initData(&d_innovation, "innovation", "innovation value computed by the filter" ) )
    , d_draw(initData(&d_draw, false, "draw","Activation of draw"))
    , d_radius_draw( initData(&d_radius_draw, "radiusDraw", "radius of the spheres") )

{

}

template <class FilterType>
void UKFilterClassic<FilterType>::computePerturbedStates()
{
    EVectorX xCol(stateSize);
    int currentPointID = 0;

    for (size_t i = 0; i < sigmaPointsNum; i++) {
        xCol = matXi.col(i);
        stateWrappers[0]->transformState(xCol, mechParams, &currentPointID);
        m_sigmaPointObservationIndexes[i] = currentPointID;
        matXi.col(i) = xCol;
    }
}

template <class FilterType>
void UKFilterClassic<FilterType>::computePrediction()
{
    hasObs = observationManager->hasObservation(this->actualTime);
    PRNS("Computing prediction, T= " << this->actualTime  << " ======");
    /// Computes background error variance Cholesky factorization.
    Eigen::LLT<EMatrixX> lltU(stateCovar);
    EMatrixX matPsqrt = lltU.matrixL();

    stateExp.fill(FilterType(0.0));
    stateExp = masterStateWrapper->getState();
    //    PRNS("X(n): \n" << stateExp.transpose());
    //    PRNS("P(n): \n" << stateCovar);

    if(hasObs){
        /// Computes X_{n}^{(i)-} sigma points
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            matXi.col(i) = stateExp + matPsqrt * matI.row(i).transpose();
        }
        genMatXi=matXi;
        EVectorX dd = matXi.row(stateSize-1);
        EVectorX p;
        p.resize(sigmaPointsNum);
        p.setZero();
        for (unsigned i=0; i<sigmaPointsNum; i++) {
            p(i)= dd(i)+stateExp(stateSize-1)  - 0.7;
        }
        d[0]=p(sigmaPointsNum-1);
        d[1]=p(sigmaPointsNum-2);

        //        PRNS("d :" << d)
        /// Compute the state
        //        PRNS("gen Sigma \n"<< matXi <<" \n ");
        computePerturbedStates();
        //        PRNS("prop Sigma \n "<< matXi);

        /// compute the expected value
        stateExp.fill(FilterType(0.0));
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            stateExp += matXi.col(i) * vecAlpha(i);
        }


        stateCovar.setZero();
        EMatrixX matXiTrans= matXi.transpose();
        EMatrixX centeredPxx = matXiTrans.rowwise() - matXiTrans.colwise().mean();
        EMatrixX weightedCenteredPxx = centeredPxx.array().colwise() * vecAlphaVar.array();
        EMatrixX covPxx = (centeredPxx.adjoint() * weightedCenteredPxx);
        EMatrixX PrintcovPxx =  covPxx;
        //EMatrixX covPxx = (centeredPxx.adjoint() * centeredPxx) / double(centeredPxx.rows() )
        stateCovar = covPxx + modelCovar;
        for (unsigned i = 0; i < stateCovar.rows(); i++) {
            diagStateCov(i)=stateCovar(i,i);
            for (unsigned j = 0; j < stateCovar.cols(); j++){
                if(PrintcovPxx(i,j)*PrintcovPxx(i,j)<=0.00000001)
                    PrintcovPxx(i,j)=0.0;

            }
        }
//        PRNS("PredCov \n "<< PrintcovPxx);
        if (masterStateWrapper->estimPosition()) {
            masterStateWrapper->setState(stateExp, mechParams);
        }else{
            masterStateWrapper->lastApplyOperator(stateExp, mechParams);
        }
    }else{
        masterStateWrapper->lastApplyOperator(stateExp, mechParams);

    }
}

template <class FilterType>
void UKFilterClassic<FilterType>::computeCorrection()
{

    if (hasObs) {
        PRNS("======= Computing correction, T= " << this->actualTime << " ======");

        EVectorX zCol(observationSize);
        matZmodel.resize(observationSize, sigmaPointsNum);
        predObsExp.resize(observationSize);
        predObsExp.fill(FilterType(0.0));

        /// compute predicted observations
        for (size_t i = 0; i < sigmaPointsNum; i++) {
            observationManager->getPredictedObservation(m_sigmaPointObservationIndexes[i],  zCol);
            matZmodel.col(i) = zCol;
            predObsExp = predObsExp + zCol * vecAlpha(i);
        }

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
        matPxz=covPxz;
        //EMatrixX covPzz = (centeredCz.adjoint() * centeredCz) / double(centeredCz.rows() );
        EMatrixX covPzz = (centeredCz.adjoint() * weightedCenteredCz);
        matPz = obsCovar + covPzz;
        EMatrixX matK(stateSize, observationSize);
        pseudoInverse(matPz, pinvmatPz);
        matK =matPxz*pinvmatPz;

        EVectorX innovation(observationSize);
        observationManager->getInnovation(this->actualTime, predObsExp, innovation);
        stateExp = stateExp + matK * innovation;

        stateCovar = stateCovar - matK*matPxz.transpose();

        for (size_t i = 0; i < (size_t)stateCovar.rows(); i++) {
            diagStateCov(i)=stateCovar(i,i);
        }

        PRNS("FINAL STATE X(n+1)+n: \n" << stateExp.transpose());
        PRNS("FINAL COVARIANCE DIAGONAL P(n+1)+n:  \n" << diagStateCov.transpose());


        if (masterStateWrapper->estimPosition()) {
            masterStateWrapper->setState(stateExp, mechParams);
        }else{
            masterStateWrapper->lastApplyOperator(stateExp, mechParams);
        }
        helper::WriteAccessor<Data <helper::vector<FilterType> > > stat = d_state;
        helper::WriteAccessor<Data <helper::vector<FilterType> > > var = d_variance;
        helper::WriteAccessor<Data <helper::vector<FilterType> > > innov = d_innovation;

        stat.resize(stateSize);
        var.resize(stateSize);
        innov.resize(observationSize);

        for (size_t i = 0; i < stateSize; i++) {
            stat[i] = stateExp[i];
            var[i] = stateCovar(i,i);
        }
        for (size_t index = 0; index < observationSize; index++) {
            innov[index] = innovation[index];
        }

        char nstepc[100];
        sprintf(nstepc, "%04lu", stepNumber);
        if (! exportPrefix.empty()) {
            std::string fileName = exportPrefix + "/covar_" + nstepc + ".txt";
            std::ofstream ofs;
            ofs.open(fileName.c_str(), std::ofstream::out);
            ofs << stateCovar << std::endl;
        }

        writeValidationPlot(d_filenameInn.getValue(),innovation);
    }else{
        PRNS("PREDICTED STATE X(n+1)+n: \n" << stateExp.transpose());
        PRNS("PREDICTED COVARIANCE DIAGONAL P(n+1)+n:  \n" << diagStateCov.transpose());
        modelCovar.setZero();
    }

    writeValidationPlot(d_filenameCov.getValue(),diagStateCov);
    writeValidationPlot(d_filenameFinalState.getValue() ,stateExp);

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

            this->saveParam = false;
    if (!d_filenameCov.getValue().empty()) {
        std::ofstream paramFile(d_filenameCov.getValue().c_str());
        if (paramFile.is_open()) {
            this->saveParam = true;
            paramFile.close();
        }
    }
    if (!d_filenameInn.getValue().empty()) {
        std::ofstream paramFileInn(d_filenameInn.getValue().c_str());
        if (paramFileInn .is_open()) {
            this->saveParam = true;
            paramFileInn.close();
        }
    }
    if (!d_filenameFinalState.getValue().empty()) {
        std::ofstream paramFileFinalState(d_filenameFinalState.getValue().c_str());
        if (paramFileFinalState .is_open()) {
            this->saveParam = true;
            paramFileFinalState.close();
        }
    }

    m_omega= 1e-5;


}

template <class FilterType>
void UKFilterClassic<FilterType>::bwdInit() {
    assert(masterStateWrapper);

    stateSize = masterStateWrapper->getStateSize();
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

    stateExp = masterStateWrapper->getState();

    /// compute sigma points
    switch (this->m_sigmaTopology) {
    case STAR:
        computeStarSigmaPoints(matI);
        break;
    case SIMPLEX:
    default:
        computeSimplexSigmaPoints(matI);
    }

    sigmaPointsNum = matI.rows();
    matXi.resize(stateSize, sigmaPointsNum);
    matXi.setZero();
    genMatXi.resize(stateSize, sigmaPointsNum);
    genMatXi.setZero();
    d.resize(2);

    /// initialise state observation mapping for sigma points
    m_sigmaPointObservationIndexes.resize(sigmaPointsNum);
}

template <class FilterType>
void UKFilterClassic<FilterType>::initializeStep(const core::ExecParams* _params, const size_t _step) {
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
void UKFilterClassic<FilterType>::stabilizeMatrix (EMatrixX& _initial, EMatrixX& _stabilized) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(_initial, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singVals = svd.singularValues();
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singValsStab = singVals;
    for (int i=0; i < singVals.rows(); i++ ){
        if ((singValsStab(i)*singValsStab(i))*(1.0/(singVals(0)*singVals(0)))< 1.0e-6) singValsStab(i)=0;
    }
    _stabilized= svd.matrixU()*singValsStab*svd.matrixV().transpose();

}

template <class FilterType>
void UKFilterClassic<FilterType>::pseudoInverse( EMatrixX& M,EMatrixX& pinvM) {
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
void UKFilterClassic<FilterType>::writeValidationPlot (std::string filename ,EVectorX& state ){
    if (this->saveParam) {
        std::ofstream paramFile(filename.c_str(), std::ios::app);
        if (paramFile.is_open()) {
            paramFile << state.transpose() << "";
            paramFile << '\n';
            paramFile.close();
        }
    }
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
    workingMatrix(0,0) = scal;
    workingMatrix(0,1) = -scal;

    for (size_t i = 1; i < p; i++) {
        scal = Type(1.0) / sqrt(beta * Type(i+1) * Type(i+2));

        for (size_t j = 0; j < i+1; j++)
            workingMatrix(i,j) = -scal;
        workingMatrix(i,i+1) = Type(i+1) * scal;
    }


    sigmaMat.resize(r,p);
    for (size_t i = 0; i < r; i++)
        sigmaMat.row(i) = workingMatrix.col(i) * sqrt_p;

    vecAlpha.resize(r);
    vecAlpha.fill(Type(1.0)/Type(r));
    alphaConstant = true;
    alpha = vecAlpha(0);

    alphaVar = (this->useUnbiasedVariance.getValue()) ? Type(1.0)/Type(r-1) : Type(1.0)/Type(r);
    vecAlphaVar.resize(r);
    vecAlphaVar.fill(alphaVar);
}

template <class FilterType>
void UKFilterClassic<FilterType>::computeStarSigmaPoints(EMatrixX& sigmaMat) {
    size_t p = stateSize;
    size_t r = 2 * stateSize + 1;

    EMatrixX workingMatrix = EMatrixX::Zero(p, r);

    Type lambda, k, sqrt_vec;
    k = 3 - p;
    lambda = this->lambdaScale.getValue() * this->lambdaScale.getValue() * (p + k) - p;
    sqrt_vec = sqrt(p + lambda);

    for (size_t j = 0; j < p; j++) {
        for (size_t i = 0; i < p; i++)
            workingMatrix(i,j) = sqrt_vec;
    }
    for (size_t j = p + 1; j < 2 * p; j++) {
        for (size_t i = 0; i < p; i++)
            workingMatrix(i,j) = -sqrt_vec;
    }


    sigmaMat.resize(r,p);
    for (size_t i = 0; i < r; i++)
        sigmaMat.row(i) = workingMatrix.col(i);

    vecAlpha.resize(r);
    for (size_t i = 0; i < 2 * p; i++) {
        vecAlpha(i) = Type(1.0)/Type(2 * (p + lambda));
    }
    // double beta = 2.0;
    vecAlpha(2 * p) = Type(lambda)/Type(2 * (p + lambda)); // + (1 - this->lambdaScale * this->lambdaScale + beta)
    alphaConstant = false;

    vecAlphaVar.resize(r);
    if (this->useUnbiasedVariance.getValue()) {
        for (size_t i = 0; i < 2 * p; i++) {
            vecAlphaVar(i) = Type(1.0)/Type(2 * (p + lambda));
        }
        // double beta = 2.0;
        vecAlphaVar(2 * p) = Type(lambda)/Type(2 * (p + lambda)); // + (1 - this->lambdaScale * this->lambdaScale + beta)

        alphaVar = Type(1.0)/Type(r-1);
    } else {
        for (size_t i = 0; i < 2 * p; i++) {
            vecAlphaVar(i) = Type(1.0)/Type(2 * (p + lambda) + 1);
        }
        // double beta = 2.0;
        vecAlphaVar(2 * p) = Type(lambda)/Type(2 * (p + lambda) + 1); // + (1 - this->lambdaScale * this->lambdaScale + beta)

        alphaVar = Type(1.0)/Type(r);
    }
}

template <class FilterType>
void UKFilterClassic<FilterType>::draw(const core::visual::VisualParams* vparams ) {
    if(d_draw.getValue()){
        if (vparams->displayFlags().getShowVisualModels()) {
            helper::vector<sofa::defaulttype::Vec3d> genSigma;

            genSigma.resize(sigmaPointsNum);
            for (unsigned i =0;  i < sigmaPointsNum; i++){
                for(unsigned j =0;  j < 3; j++){
                    genSigma[i][j]=genMatXi(j,i) ;
                }
            }
            vparams->drawTool()->drawSpheres(genSigma,  d_radius_draw.getValue(), sofa::defaulttype::Vec<4, float>(1.0f,0.0f,0.0f,1.0f));

        }


        //        float size;
        //        if (size == 0.0f)
        //            size = (float)10.0f;

        //        defaulttype::Vec3d normal; normal =defaulttype::Vec3d(0,1,0);
        //        defaulttype::RGBAColor d_drawColor= defaulttype::RGBAColor(1.0f,.0f,.0f,1.0f);
        //        defaulttype::RGBAColor d_drawColor2= defaulttype::RGBAColor(0.0f,.0f,1.0f,1.0f);

        //        // find a first vector inside the plane
        //        defaulttype::Vec3d v1;
        //        if( 0.0 != normal[0] ) v1 = defaulttype::Vec3d(-normal[1]/normal[0], 1.0, 0.0);
        //        else if ( 0.0 != normal[1] ) v1 = defaulttype::Vec3d(1.0, -normal[0]/normal[1],0.0);
        //        else if ( 0.0 != normal[2] ) v1 = defaulttype::Vec3d(1.0, 0.0, -normal[0]/normal[2]);
        //        v1.normalize();

        //        // find a second vector inside the plane and orthogonal to the first
        //        defaulttype::Vec3d v2;
        //        v2 = v1.cross(normal);
        //        v2.normalize();

        //        defaulttype::Vec3d center = normal*d[0];
        //        defaulttype::Vec3d corners[4];
        //        corners[0] = center-v1*size-v2*size;
        //        corners[1] = center+v1*size-v2*size;
        //        corners[2] = center+v1*size+v2*size;
        //        corners[3] = center-v1*size+v2*size;

        //        std::vector< defaulttype::Vector3 > points;

        //        points.push_back(corners[0]);
        //        points.push_back(corners[1]);
        //        points.push_back(corners[2]);

        //        points.push_back(corners[0]);
        //        points.push_back(corners[2]);
        //        points.push_back(corners[3]);

        //        vparams->drawTool()->setPolygonMode(2,false); //Cull Front face

        //        vparams->drawTool()->drawTriangles(points, defaulttype::Vec<4,float>(d_drawColor[0],d_drawColor[1],d_drawColor[2],0.5));
        //        vparams->drawTool()->setPolygonMode(0,false); //No Culling


        //        defaulttype::Vec3d center2 = normal*d[1];
        //        defaulttype::Vec3d corners2[4];
        //        corners2[0] = center2-v1*size-v2*size;
        //        corners2[1] = center2+v1*size-v2*size;
        //        corners2[2] = center2+v1*size+v2*size;
        //        corners2[3] = center2-v1*size+v2*size;

        //        std::vector< defaulttype::Vector3 > points2;

        //        points2.push_back(corners2[0]);
        //        points2.push_back(corners2[1]);
        //        points2.push_back(corners2[2]);

        //        points2.push_back(corners2[0]);
        //        points2.push_back(corners2[2]);
        //        points2.push_back(corners2[3]);

        //        vparams->drawTool()->setPolygonMode(2,false); //Cull Front face

        //        vparams->drawTool()->drawTriangles(points2, defaulttype::Vec<4,float>(d_drawColor2[0],d_drawColor2[1],d_drawColor2[2],0.5));
        //        vparams->drawTool()->setPolygonMode(0,false); //No Culling


    }
}

} // stochastic
} // component
} // sofa

#endif // UKFilterClassic_INL


/*matPxzB.fill(FilterType(0.0));
matPzB.fill(FilterType(0.0));

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
           matPxzB(x,y) += vx(x)*vz(y);

   for (size_t x = 0; x < observationSize; x++)
       for (size_t y = 0; y < observationSize; y++)
           matPzB(x,y) += vz(x)*vz(y);
}
matPxzB = alphaVar * matPxzB;
matPzB = alphaVar * matPzB + obsCovar;*/

/*
    stateCovar.fill(FilterType(0.0));
    EVectorX tmpX(stateSize);
    tmpX.fill(FilterType(0.0));
    for (size_t i = 0; i < sigmaPointsNum; i++) {
       tmpX = matXi.col(i) - stateExp;
       for (size_t x = 0; x < stateSize; x++)
           for (size_t y = 0; y < stateSize; y++)
               stateCovar(x,y) += tmpX(x)*tmpX(y);
    }
    stateCovar=alphaVar*stateCovar;
}*/

