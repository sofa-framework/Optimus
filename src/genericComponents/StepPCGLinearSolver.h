/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2016 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_COMPONENT_LINEARSOLVER_StepPCGLinearSolver_H
#define SOFA_COMPONENT_LINEARSOLVER_StepPCGLinearSolver_H

#include <SofaCommon/config.h>
#include <sofa/core/behavior/LinearSolver.h>
#include <SofaBaseLinearSolver/MatrixLinearSolver.h>
#include <sofa/helper/map.h>

#include "../initOptimusPlugin.h"

#include <math.h>

namespace sofa
{

namespace component
{

namespace linearsolver
{


/// Linear system solver using the conjugate gradient iterative algorithm
template<class TMatrix, class TVector>
class StepPCGLinearSolver : public sofa::component::linearsolver::MatrixLinearSolver<TMatrix,TVector>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(StepPCGLinearSolver,TMatrix,TVector),SOFA_TEMPLATE2(sofa::component::linearsolver::MatrixLinearSolver,TMatrix,TVector));

    typedef TMatrix Matrix;
    typedef TVector Vector;
    typedef sofa::component::linearsolver::MatrixLinearSolver<TMatrix,TVector> Inherit;

    Data<unsigned> f_maxIter;
    Data<double> f_tolerance;
    Data<bool> f_use_precond;
    Data<bool> f_precondOnTimeStep;
    Data<unsigned> f_update_step;
    Data<bool> f_build_precond;
    Data<bool> f_forceFactorization;
    Data<int> f_iterationsNeeded;
    Data<int> f_numIterationsToRefactorize;
    Data< std::string > f_preconditioners;
    Data<std::map < std::string, sofa::helper::vector<double> > > f_graph;
    Data<bool> verbose;


protected:
    StepPCGLinearSolver();
public:
    void solve (Matrix& M, Vector& x, Vector& b) override;
    void init() override;
    void setSystemMBKMatrix(const core::MechanicalParams* mparams) override;
    //void setSystemRHVector(VecId v);
    //void setSystemLHVector(VecId v);

private :
    unsigned next_refresh_step;
    sofa::core::behavior::LinearSolver* preconditioners;
    bool first;
    int newton_iter;

    double lastTime;

protected:
    /// This method is separated from the rest to be able to use custom/optimized versions depending on the types of vectors.
    /// It computes: p = p*beta + r
    inline void cgstep_beta(Vector& p, Vector& r, double beta);
    /// This method is separated from the rest to be able to use custom/optimized versions depending on the types of vectors.
    /// It computes: x += p*alpha, r -= q*alpha
    inline void cgstep_alpha(Vector& x,Vector& p,double alpha);

    void handleEvent(sofa::core::objectmodel::Event* event) override;


    sofa::simulation::Node* gnode;
    double firstTemporalInvocation;
};

template<class TMatrix, class TVector>
inline void StepPCGLinearSolver<TMatrix,TVector>::cgstep_beta(Vector& p, Vector& r, double beta)
{
    p *= beta;
    p += r; //z;
}

template<class TMatrix, class TVector>
inline void StepPCGLinearSolver<TMatrix,TVector>::cgstep_alpha(Vector& x,Vector& p,double alpha)
{
    x.peq(p,alpha);                 // x = x + alpha p
}

template<>
inline void StepPCGLinearSolver<component::linearsolver::GraphScatteredMatrix,component::linearsolver::GraphScatteredVector>::cgstep_beta(Vector& p, Vector& r, double beta);

template<>
inline void StepPCGLinearSolver<component::linearsolver::GraphScatteredMatrix,component::linearsolver::GraphScatteredVector>::cgstep_alpha(Vector& x,Vector& p,double alpha);

} // namespace linearsolver

} // namespace component

} // namespace sofa

#endif
