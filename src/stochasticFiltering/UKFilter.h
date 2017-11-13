#ifndef UKFilter_H_
#define UKFilter_H_

#include "initOptimusPlugin.h"
#include "StochasticFilterBase.h"
#include "StochasticStateWrapper.h"
#include "ObservationManagerBase.h"
#include "genericComponents/SimulatedStateObservationSource.h"

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#ifdef Success
#undef Success // dirty workaround to cope with the (dirtier) X11 define. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=253
#endif
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
//#include <Accelerate/Accelerate.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType>
struct WorkerThreadData
{
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;

    StochasticStateWrapperBaseT<FilterType>* wrapper;
    size_t threadID;
    helper::vector<size_t>* sigmaIDs;
    EMatrixX* stateMatrix;
    const core::ExecParams* execParams;
    bool saveLog;

    void set(size_t _threadID,StochasticStateWrapperBaseT<FilterType>* _wrapper,  helper::vector<size_t> *_sigID,
             EMatrixX* _stateMat, const core::ExecParams* _execParams, bool _saveLog) {
        threadID=_threadID;
        sigmaIDs=_sigID;
        stateMatrix=_stateMat;
        saveLog = _saveLog;
        wrapper = _wrapper;
        execParams = _execParams;
    }
};

extern "C"{
    // product C= alphaA.B + betaC
   void dgemm_(char* TRANSA, char* TRANSB, const int* M,
               const int* N, const int* K, double* alpha, double* A,
               const int* LDA, double* B, const int* LDB, double* beta,
               double* C, const int* LDC);
    // product Y= alphaA.X + betaY
   void dgemv_(char* TRANS, const int* M, const int* N,
               double* alpha, double* A, const int* LDA, double* X,
               const int* INCX, double* beta, double* C, const int* INCY);
   }


using namespace defaulttype;

template <class FilterType>
class UKFilter: public sofa::component::stochastic::StochasticFilterBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(UKFilter, FilterType), StochasticFilterBase);

    typedef sofa::component::stochastic::StochasticFilterBase Inherit;
    typedef FilterType Type;

    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;
    typedef defaulttype::Rigid3dTypes MechanicalType;
    typedef typename MechanicalType::VecCoord VecCoord;
    typedef typename MechanicalType::Coord Coord;
    typedef typename core::behavior::MechanicalState<Rigid3dTypes> MechanicalState;
    typedef  sofa::component::container::SimulatedStateObservationSourceBase ObservationSource;

    typedef typename MechanicalState::ReadVecCoord ReadVecCoord;
    typedef typename MechanicalType::VecDeriv VecDeriv;
UKFilter();
~UKFilter();

protected:
    StochasticStateWrapperBaseT<FilterType>* masterStateWrapper;
    helper::vector<StochasticStateWrapperBaseT<FilterType>*> stateWrappers;
    ObservationManager<FilterType>* observationManager;
    ObservationSource *observationSource;


    /// vector sizes
    size_t observationSize, stateSize, reducedStateSize;
    size_t numThreads;

    /// number of sigma points (according to the filter type)
    size_t sigmaPointsNum;
    bool alphaConstant;
    enum { Dim = Coord::spatial_dimensions };
    int dim;
    EVectorX vecAlpha;
    EVectorX obsPrec;
    EMatrixX matItrans, matI;
    EMatrixX matDv;
    EMatrixX matXi, matZi, matXAi, matZmodel;
    EMatrixX matP, matPsqrt, matPxz, matPzz, matVinv, matR, matQ;
    FilterType errorVarianceValue;
    FilterType stateVarianceValue;
    Data<Mat3x4d> d_projectionMatrix;

    sofa::core::objectmodel::DataFileName d_filename;
    std::ofstream* outfile;

    Type alpha;

    /// structures for parallel computing:
    helper::vector<size_t> sigmaPoints2WrapperIDs;
    helper::vector<helper::vector<size_t> > wrapper2SigmaPointsIDs;

    /// functions
    void computeSimplexSigmaPoints(EMatrixX& sigmaMat);

public:
    Data<std::string> observationErrorVarianceType;
    Data<double> d_obsStdev;
    Data<double> d_stateStdev;
    Data<double> d_initModelVar;


    EVectorX modelObservations;
    helper::vector<size_t> fixedNodes, freeNodes;
    helper::vector<std::pair<size_t, size_t> > positionPairs;

//    helper::vector<helper::vector<Vector3  > > modelObservations;
    core::behavior::MechanicalState<MechanicalType>* mstate;



    void init();
    void bwdInit();


    virtual void computePrediction(); // Compute perturbed state included in computeprediction
    virtual void propagatePerturbedStates(EVectorX &_meanState);
    virtual void computeCorrection();

    virtual void initializeStep(const core::ExecParams* _params, const size_t _step);

}; /// class

} // stochastic
} // component
} // sofa

#endif // UKFilter_H


