#ifndef UKFilterGeneral_H_
#define UKFilterGeneral_H_

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
class UKFilterClassic: public sofa::component::stochastic::StochasticFilterBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(UKFilterClassic, FilterType), StochasticFilterBase);

    typedef sofa::component::stochastic::StochasticFilterBase Inherit;
    typedef FilterType Type;

    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

    UKFilterClassic();
    ~UKFilterClassic() {}

protected:
    StochasticStateWrapperBaseT<FilterType>* masterStateWrapper;
    helper::vector<StochasticStateWrapperBaseT<FilterType>*> stateWrappers;
    ObservationManager<FilterType>* observationManager;
    //ObservationSource *observationSource;


    /// vector sizes
    size_t observationSize, stateSize, reducedStateSize;
    size_t numThreads;

    /// number of sigma points (according to the filter type)
    size_t sigmaPointsNum;
    bool alphaConstant;
    std::vector<int> m_sigmaPointObservationIndexes;
    helper::vector<double> d;

    EVectorX vecAlpha, vecAlphaVar;
    EVectorX stateExp, predObsExp;
    EMatrixX stateCovar, obsCovar, modelCovar;

    EMatrixX matItrans, matI;
    EMatrixX matXi, matZmodel, genMatXi;

    sofa::core::objectmodel::DataFileName d_exportPrefix;
    std::string exportPrefix;
    std::string  filenameFinalState;
    Data< std::string >  d_filenameFinalState;
    bool saveParam;
    Type alpha, alphaVar;


    /// structures for parallel computing:
    helper::vector<size_t> sigmaPoints2WrapperIDs;
    helper::vector<helper::vector<size_t> > wrapper2SigmaPointsIDs;

    /// functions_initial
    void computeSimplexSigmaPoints(EMatrixX& sigmaMat);
    void computeStarSigmaPoints(EMatrixX& sigmaMat);

public:    
    Data<helper::vector<FilterType> > d_state;
    Data<helper::vector<FilterType> > d_variance;
    Data<helper::vector<FilterType> > d_covariance;
    Data<helper::vector<FilterType> > d_innovation;
    Data< bool  > d_draw;
    Data< double  > d_radius_draw;
    Data< double  > d_MOnodes_draw;
    double m_omega;
    bool hasObs;
    void init() override;
    void bwdInit() override;

    /*virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const UKFilterClassic<FilterType>* = NULL)
    {
        return
    }*/
    void stabilizeMatrix (EMatrixX& _initial, EMatrixX& _stabilized);
    void pseudoInverse (EMatrixX& M,EMatrixX& pinvM );
    void writeValidationPlot (std::string filename ,EVectorX& state );
void sqrtMat(EMatrixX& A, EMatrixX& sqrtA);
    virtual void computePerturbedStates();

    virtual void computePrediction() override; // Compute perturbed state included in computeprediction
    virtual void computeCorrection() override;

    virtual void initializeStep(const core::ExecParams* _params, const size_t _step) override;
    void draw(const core::visual::VisualParams* vparams) override;

    virtual void updateState() override { }

}; /// class

} // stochastic
} // component
} // sofa

#endif // UKFilterClassic_H


