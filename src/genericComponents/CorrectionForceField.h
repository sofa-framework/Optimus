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

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/helper/vector.h>
#include <SofaBaseTopology/TopologySubsetData.h>
#include <SofaBaseTopology/TopologySubsetData.inl>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/Event.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>



namespace sofa
{

namespace component
{

namespace forcefield
{


/**
* @brief This class describes a simple elastic springs ForceField between DOFs positions and rest positions.
*
* Springs are applied to given degrees of freedom between their current positions and their rest shape positions.
* An external MechanicalState reference can also be passed to the ForceField as rest shape position.
*/
template<class DataTypes>
class CorrectionForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(CorrectionForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    typedef core::behavior::ForceField<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::CPos CPos;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real Real;
    typedef type::vector< unsigned int > VecIndex;
    typedef type::vector< Real > VecReal;
    typedef sofa::component::topology::PointSubsetData< VecIndex > SetIndex;

    typedef core::objectmodel::Data<VecCoord> DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;


    Data< Deriv > d_force;
    SetIndex d_indices;
    Data< VecDeriv > d_forces;
    Data< type::vector<double> > d_Optimforces;
    Data< double > d_delta;
    Data< double > d_paramF;

    type::vector<bool> m_active;


protected:
    CorrectionForceField();

public:
    /// BaseObject initialization method.
    void bwdInit() override;

    /// Add the forces.
    virtual void addForce(const core::MechanicalParams* /* mparams */, DataVecDeriv& f, const DataVecCoord& /* x */, const DataVecDeriv& /* v */) override;

    virtual void addDForce(const core::MechanicalParams* /* mparams */, DataVecDeriv& /* df */, const DataVecDeriv& /* dx */) override;

    /// Brings ForceField contribution to the global system stiffness matrix.
    virtual void addKToMatrix(const core::MechanicalParams* /* mparams */, const sofa::core::behavior::MultiMatrixAccessor* /* matrix */) override;

    virtual SReal getPotentialEnergy(const core::MechanicalParams* /*mparams*/, const DataVecCoord&  /* x */) const override
    {
        serr << "Get potentialEnergy not implemented" << sendl;
        return 0.0;
    }

    virtual void draw(const core::visual::VisualParams* /* vparams */) override;
    void plusF();
    void minusF();
    void handleEvent(sofa::core::objectmodel::Event *event) override;
};



} // namespace forcefield

} // namespace component

} // namespace sofa

