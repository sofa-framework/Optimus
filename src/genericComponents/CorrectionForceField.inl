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
#ifndef SOFA_COMPONENT_FORCEFIELD_FACIASPRINGFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_FACIASPRINGFORCEFIELD_INL

#include "genericComponents/CorrectionForceField.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/helper/gl/template.h>
#include <assert.h>
#include <iostream>


namespace sofa
{

namespace component
{

namespace forcefield
{

template<class DataTypes>
CorrectionForceField<DataTypes>::CorrectionForceField()
    : d_force(initData(&d_force, "force","applied forces at each point"))

    ,d_indices(initData(&d_indices, "indices",
                        "indices where the forces are applied"))


    , d_forces(initData(&d_forces, "forces",
                        "applied forces at each point"))
    , d_Optimforces(initData(&d_Optimforces, "optimForces",
                        "applied forces at each point"))
{
}


template<class DataTypes>
void CorrectionForceField<DataTypes>::bwdInit()
{
    core::behavior::ForceField<DataTypes>::init();
}

template<class DataTypes>
void CorrectionForceField<DataTypes>::addForce(const core::MechanicalParams* /*params*/,
                                               DataVecDeriv& f1, const DataVecCoord& p1, const DataVecDeriv&)
{
    sofa::helper::WriteAccessor< core::objectmodel::Data< VecDeriv > > _f1 = f1;
    _f1.resize(p1.getValue().size());

    Deriv singleForce;


    Deriv forceVal;
    for (int i=0; i< d_Optimforces.getValue().size(); i++)
        forceVal[i]=d_Optimforces.getValue()[i];
    const VecIndex& indices = d_indices.getValue();
    const VecDeriv& f = d_forces.getValue();
    unsigned int i = 0, nbForcesIn = f.size(), nbForcesOut = _f1.size();


    if (forceVal * forceVal > 0.0)
        singleForce = forceVal;

    const Deriv f_end = (f.empty() ? singleForce : f[f.size()-1]);

    {
        unsigned int nbIndices = indices.size();
        unsigned int nbCopy = std::min(nbForcesIn, nbIndices); // forces & points are not garanteed to be of the same size
       {
            for (; i < nbCopy; ++i)
                _f1[indices[i]] += f[i];
            for (; i < nbIndices; ++i)
                _f1[indices[i]] += f_end;
        }

    }
}


template<class DataTypes>
void CorrectionForceField<DataTypes>::addDForce(const core::MechanicalParams* /* mparams */, DataVecDeriv& /* df */, const DataVecDeriv& /* dx */)
{
    //    sofa::helper::WriteAccessor< DataVecDeriv > df1 = df;
    //    Real kFactor = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    //    for (unsigned int i=0; i<df1.size(); i++)
    //    {
    //        df1[i] += d_forces.getValue()[i] * kFactor;
    //    }
}


template<class DataTypes>
void CorrectionForceField<DataTypes>::addKToMatrix(const core::MechanicalParams* /* mparams */, const sofa::core::behavior::MultiMatrixAccessor* /* matrix */)
{
    //    sofa::core::behavior::MultiMatrixAccessor::MatrixRef mref = matrix->getMatrix(this->mstate);
    //    sofa::defaulttype::BaseMatrix* mat = mref.matrix;
    //    unsigned int offset = mref.offset;
    //    Real kFact = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());
    //    const int N = Coord::total_size;

    //    for (unsigned int index = 0; index < d_forces.getValue().size(); index++) {
    //        for(int i = 0; i < N; i++)
    //            mat->add(offset + N * index + i, offset + N * index + i, kFact);
    //    }
}

template<class DataTypes>
void CorrectionForceField<DataTypes>::draw(const core::visual::VisualParams* /* vparams */) {



    //    if (! vparams->displayFlags().getShowCollisionModels()) return;

    //    helper::ReadAccessor<Data <VecCoord> > x = *this->mstate->read(core::VecCoordId::position());
    //    helper::ReadAccessor<Data <VecCoord> > y = *this->m_extState->read(core::VecCoordId::position());

    //    glDisable(GL_LIGHTING);
    //    glLineWidth(2);


    //    glBegin(GL_LINES);
    //    for (unsigned int i=0; i<m_active.size(); i++) {
    //        if (!m_active[i]) glColor4f(1.0f,0.0f,0.0f,1.0f);
    //        else glColor4f(0.0f,0.0f,1.0f,1.0f);

    //        helper::gl::glVertexT(x[i]);
    //        helper::gl::glVertexT(y[i]);
    //    }
    //    glEnd();

    //    glPointSize(1);
    //    glLineWidth(1);

}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_RESTSHAPESPRINGFORCEFIELD_INL



