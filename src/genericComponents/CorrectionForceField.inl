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

#include "genericComponents/CorrectionForceField.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/helper/gl/template.h>
#include <assert.h>
#include <iostream>
#include <sofa/core/objectmodel/Event.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>



namespace sofa
{

namespace component
{

namespace forcefield
{


template<class DataTypes>
CorrectionForceField<DataTypes>::CorrectionForceField()
    : d_force(initData(&d_force, "force","applied forces at each point"))

    , d_indices(initData(&d_indices, "indices",
                         "indices where the forces are applied"))

    , d_forces(initData(&d_forces, "forces",
                        "applied forces at each point"))
    , d_Optimforces(initData(&d_Optimforces, "optimForces",
                             "applied forces at each point"))
    , d_delta(initData(&d_delta, "delta","translation rotation vector"))

    , d_paramF(initData(&d_paramF, "paramF",
                             "parameter modeling force uncertainty"))
{
}


template<class DataTypes>
void CorrectionForceField<DataTypes>::bwdInit()
{
    core::behavior::ForceField<DataTypes>::init();
    this->f_listening.setValue(true);

}

template<class DataTypes>
void CorrectionForceField<DataTypes>::plusF() {

    double temp = d_Optimforces.getValue()[0];
    temp+= d_delta.getValue();
    helper::vector<double> T;
    T.resize(d_Optimforces.getValue().size());

    T[0]=d_Optimforces.getValue()[0];
    T[1]=temp;
    for (unsigned int i=2; i<T.size();i++)
        T[i]=d_Optimforces.getValue()[i];

    d_Optimforces=T;
    std::cout<<"newForc: "<< d_Optimforces.getValue() <<std::endl;
}

template<class DataTypes>
void CorrectionForceField<DataTypes>::minusF() {

    double temp = d_Optimforces.getValue()[0];
    temp-= d_delta.getValue();
    helper::vector<double> T;
    T.resize(d_Optimforces.getValue().size());

    T[0]=d_Optimforces.getValue()[0];

    T[1]=temp;
    for (unsigned int i=2; i<T.size();i++)
        T[i]=d_Optimforces.getValue()[i];

    d_Optimforces=T;
}

template<class DataTypes>
void CorrectionForceField<DataTypes>::handleEvent(sofa::core::objectmodel::Event *event)
{
   if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)) {
        if ((ev->getKey() == 'z') || (ev->getKey() == 'Z'))      {
            plusF();
        }else if ((ev->getKey() == 'a') || (ev->getKey() == 'A')){
            minusF();
        }
    }
}

template<class DataTypes>
void CorrectionForceField<DataTypes>::addForce(const core::MechanicalParams* /*params*/,
                                               DataVecDeriv& f1, const DataVecCoord& p1, const DataVecDeriv&)
{
    sofa::helper::WriteAccessor< core::objectmodel::Data< VecDeriv > > _f1 = f1;
    _f1.resize(p1.getValue().size());

    Deriv singleForce;


    Deriv forceVal;
    for (unsigned int i=0; i< d_Optimforces.getValue().size(); i++)
        forceVal[i]=d_paramF.getValue()*d_Optimforces.getValue()[i];
    const VecIndex& indices = d_indices.getValue();
    const VecDeriv& f = d_forces.getValue();
    unsigned int i = 0, nbForcesIn = f.size(); // nbForcesOut = _f1.size();


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

