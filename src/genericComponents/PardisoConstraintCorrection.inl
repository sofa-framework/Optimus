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

#include "PardisoConstraintCorrection.h"
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/behavior/ConstraintCorrection.inl>

#include <sstream>
#include <list>



namespace sofa
{

namespace component
{

namespace constraintset
{


template<class DataTypes>
PardisoConstraintCorrection<DataTypes>::PardisoConstraintCorrection(sofa::core::behavior::MechanicalState<DataTypes> *mm)
    : Inherit(mm)
    , schurSolverName( initData(&schurSolverName, "schurSolverName", "name of solver that computes the Schur complement (optimized CPU version with Pardiso only)") )
{

}


template<class DataTypes>
void PardisoConstraintCorrection<DataTypes>::init()
{
    Inherit::init();

    const helper::vector<std::string>& schurSolverNames = schurSolverName.getValue();

    schurLinearSolvers.clear();
    if (schurSolverNames.size() > 0)
    {
        sofa::core::objectmodel::BaseContext* c = this->getContext();
        for (unsigned int i=0; i<schurSolverNames.size(); ++i) {
            sofa::core::behavior::LinearSolver* s = NULL;
            c->get(s, schurSolverNames[i]);
            if (s) {
                schurLinearSolvers.push_back(s);
                std::cout << "Found Schur solver: " << s->getName() << std::endl;
            }
        }
    }

    std::cout << "Schur solvers: " << schurLinearSolvers.size() << std::endl;
    if (schurLinearSolvers.size() != this->linearsolvers.size())
        schurLinearSolvers = this->linearsolvers;
}

template<class DataTypes>
void PardisoConstraintCorrection<DataTypes>::addComplianceInConstraintSpace(const sofa::core::ConstraintParams *cparams, sofa::defaulttype::BaseMatrix* W)
{
    if (!this->mstate || !this->odesolver || (this->linearsolvers.size()==0)) return;

    // use the OdeSolver to get the position integration factor
    double factor = 1.0;

    switch (cparams->constOrder())
    {
    case core::ConstraintParams::POS_AND_VEL :
    case core::ConstraintParams::POS :
        factor = this->odesolver->getPositionIntegrationFactor();
        break;

    case core::ConstraintParams::ACC :
    case core::ConstraintParams::VEL :
        factor = this->odesolver->getVelocityIntegrationFactor();
        break;

    default :
        break;
    }

    const unsigned int numDOFs = this->mstate->getSize();
    const unsigned int N = Deriv::size();
    const unsigned int numDOFReals = numDOFs*N;

    // Compute J
    const MatrixDeriv& c = this->mstate->read(core::ConstMatrixDerivId::holonomicC())->getValue();
    const unsigned int totalNumConstraints = W->rowSize();

    this->J.resize(totalNumConstraints, numDOFReals);

    MatrixDerivRowConstIterator rowItEnd = c.end();

    for (MatrixDerivRowConstIterator rowIt = c.begin(); rowIt != rowItEnd; ++rowIt)
    {
        const int cid = rowIt.index();

        MatrixDerivColConstIterator colItEnd = rowIt.end();

        for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != colItEnd; ++colIt)
        {
            const unsigned int dof = colIt.index();
            const Deriv n = colIt.val();

            for (unsigned int r = 0; r < N; ++r)
            {
                this->J.add(cid, dof * N + r, n[r]);
            }
        }
    }

    // use the Linear solver to compute J*inv(M)*Jt, where M is the mechanical linear system matrix
    for (unsigned i = 0; i < schurLinearSolvers.size(); i++)
    {
        double startTime, stopTime;
        sofa::helper::system::thread::CTime *timer;
        startTime = double(timer->getTime());
        this->linearsolvers[i]->setSystemLHVector(sofa::core::MultiVecDerivId::null());
        schurLinearSolvers[i]->addJMInvJt(W, &this->J, factor);
        stopTime = double(timer->getTime());
        std::cout << "[" << this->getName() << "] WTIME: compute Schur total" << " " << stopTime - startTime << std::endl;
    }

}



}   // constraintset namespace

}   // component

}   // sofa

