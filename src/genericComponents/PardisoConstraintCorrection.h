/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2020 MGH, INRIA, USTL, UJF, CNRS                    *
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
#pragma once

#include "initOptimusPlugin.h"

#include <sofa/core/behavior/ConstraintCorrection.h>

#include <sofa/core/behavior/OdeSolver.h>
#include <sofa/core/behavior/LinearSolver.h>

#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Vec.h>

#include <SofaBaseLinearSolver/SparseMatrix.h>
#include <SofaBaseLinearSolver/FullMatrix.h>

#include <SofaConstraint/LinearSolverConstraintCorrection.h>



namespace sofa
{

namespace component
{

namespace constraintset
{


/*/// to avoid compilation problem under gcc3.3
extern inline sofa::core::behavior::OdeSolver* getOdeSolver(sofa::core::objectmodel::BaseContext* context)
{
    return context->get<sofa::core::behavior::OdeSolver>();
}*/

/**
 *  \brief Component computing contact forces within a simulated body using the compliance method.
 */
template<class TDataTypes>
class PardisoConstraintCorrection : public sofa::component::constraintset::LinearSolverConstraintCorrection< TDataTypes > // public sofa::core::behavior::ConstraintCorrection< TDataTypes >
{
public:
    //SOFA_CLASS(SOFA_TEMPLATE(PardisoConstraintCorrection, TDataTypes), SOFA_TEMPLATE(sofa::core::behavior::ConstraintCorrection, TDataTypes));
    SOFA_CLASS(SOFA_TEMPLATE(PardisoConstraintCorrection, TDataTypes), SOFA_TEMPLATE(sofa::component::constraintset::LinearSolverConstraintCorrection, TDataTypes));

    typedef TDataTypes DataTypes;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::MatrixDeriv::RowConstIterator MatrixDerivRowConstIterator;
    typedef typename DataTypes::MatrixDeriv::ColConstIterator MatrixDerivColConstIterator;
    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef typename DataTypes::MatrixDeriv::ColIterator MatrixDerivColIterator;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;

    //typedef std::list<int> ListIndex;

    typedef sofa::component::constraintset::LinearSolverConstraintCorrection< TDataTypes > Inherit;
protected:
    PardisoConstraintCorrection(sofa::core::behavior::MechanicalState<DataTypes> *mm = NULL);

    virtual ~PardisoConstraintCorrection() {}
public:
    Data< helper::vector< std::string > >  schurSolverName;

    virtual void init();
    virtual void addComplianceInConstraintSpace(const sofa::core::ConstraintParams *cparams, defaulttype::BaseMatrix* W);


protected:
    std::vector<sofa::core::behavior::LinearSolver*> schurLinearSolvers;


};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_CONSTRAINT_PARDISOCONSTRAINTCORRECTION_CPP)
extern template class SOFA_OPTIMUSPLUGIN_API PardisoConstraintCorrection<sofa::defaulttype::Vec3Types>;
extern template class SOFA_OPTIMUSPLUGIN_API PardisoConstraintCorrection<sofa::defaulttype::Vec1Types>;
extern template class SOFA_OPTIMUSPLUGIN_API PardisoConstraintCorrection<sofa::defaulttype::Rigid3Types>;


} // namespace collision

} // namespace component

} // namespace sofa

