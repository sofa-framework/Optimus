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

#include "initOptimusPlugin.h"
#include "StochasticFilterBase.h"
#include "StochasticStateWrapper.h"
#include "ObservationManagerBase.h"
#include "genericComponents/SimulatedStateObservationSource.h"

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
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


/// to speed up, wrappers for BLAS matrix multiplications created, much faster that Eigen by default
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

/**
 * Class implementing a special version of the unscented Kalman filter for data assimilation.
 * Partially inspired by the reduced order Kalman filter proposed by Moireau & Chapelle
 * (Reduced-order Unscented Kalman Filtering with application to parameter identification in large-dimensional systems)
 * This version keeps _only_ the parameters in the stochastic state.
 * Filter requires StochasticStateWrapper which provides the interface with SOFA.
 */

using namespace defaulttype;

template <class FilterType>
class UKFilterSimCorr: public sofa::component::stochastic::StochasticUnscentedFilterBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(UKFilterSimCorr, FilterType), StochasticUnscentedFilterBase);

    typedef sofa::component::stochastic::StochasticUnscentedFilterBase Inherit;
    typedef FilterType Type;

    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

UKFilterSimCorr();
~UKFilterSimCorr() {}

protected:
    StochasticStateWrapperBaseT<FilterType>* masterStateWrapper;
    helper::vector<StochasticStateWrapperBaseT<FilterType>*> stateWrappers;
    ObservationManager<FilterType>* observationManager;    

    /// vector sizes
    size_t observationSize, stateSize, reducedStateSize;
    size_t numThreads;

    /// number of sigma points (according to the filter type)
    size_t sigmaPointsNum;
    bool alphaConstant;

    EVectorX vecAlpha, vecAlphaVar;
    EVectorX stateExp, predObsExp;
    EMatrixX stateCovar, obsCovar;


    EMatrixX matItrans, matI;
    EMatrixX matXi, matZmodel;

    sofa::core::objectmodel::DataFileName d_filename;
    std::ofstream* outfile;

    Type alpha, alphaVar;


    /// structures for parallel computing:
    helper::vector<size_t> sigmaPoints2WrapperIDs;
    helper::vector<helper::vector<size_t> > wrapper2SigmaPointsIDs;

    /// functions
    void computeSimplexSigmaPoints(EMatrixX& sigmaMat);
    void computeStarSigmaPoints(EMatrixX& sigmaMat);

public:    
    Data<helper::vector<FilterType> > d_state;
    Data<helper::vector<FilterType> > d_variance;
    Data<helper::vector<FilterType> > d_covariance;
    Data<helper::vector<FilterType> > d_innovation;    

    void init() override;
    void bwdInit() override;

    /*virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const UKFilterSimCorr<FilterType>* = NULL)
    {
        return
    }*/

    virtual void computePrediction() override; // Compute perturbed state included in computeprediction
    virtual void computeCorrection() override;

    virtual void initializeStep(const core::ExecParams* _params, const size_t _step) override;

    virtual void updateState() override { }

}; /// class

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



} // stochastic

} // component

} // sofa

