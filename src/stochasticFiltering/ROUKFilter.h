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
#ifndef ROUKFILTER_H_
#define ROUKFILTER_H_

#include "initOptimusPlugin.h"
#include "StochasticFilterBase.h"
#include "StochasticStateWrapper.h"
#include "ObservationManagerBase.h"

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/defaulttype.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>

#ifdef Success
#undef Success // dirty workaround to cope with the (dirtier) X11 define. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=253
#endif
#include <Eigen/Dense>

#include <Accelerate/Accelerate.h>

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
class ROUKFilter: public sofa::component::stochastic::StochasticFilterBase
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ROUKFilter, FilterType), StochasticFilterBase);

    typedef sofa::component::stochastic::StochasticFilterBase Inherit;
    typedef FilterType Type;

    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, Eigen::Dynamic> EMatrixX;
    typedef typename Eigen::Matrix<FilterType, Eigen::Dynamic, 1> EVectorX;

ROUKFilter();
~ROUKFilter();

protected:
    StochasticStateWrapperBaseT<FilterType>* stateWrapper;
    ObservationManager<FilterType>* observationManager;

    // vector sizes
    size_t observationSize, stateSize, reducedStateSize;

    // number of sigma points (according to the filter type)
    size_t sigmaPointsNum;
    bool alphaConstant;

    EVectorX vecAlpha;
    EMatrixX matU, matUinv;
    EMatrixX matItrans, matI;
    EMatrixX matDv;
    EMatrixX matXi;

    Type alpha;

    void computeSimplexSigmaPoints(EMatrixX& sigmaMat);
    void blasMultAdd(EMatrixX& _a, EMatrixX& _b, EMatrixX& _c, Type _alpha, Type _beta);

public:
    Data<std::string> observationErrorVarianceType;
    Data<bool> useBlasToMultiply;

    void init();
    void bwdInit();

    virtual void computePrediction();
    virtual void computeCorrection();





}; /// class


} // stochastic
} // component
} // sofa

#endif // ROUKFILTER_H


