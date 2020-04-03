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
#ifndef SOFA_COMPONENT_FORCEFIELD_EXTENDEDRESTSHAPESPRINGFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_EXTENDEDRESTSHAPESPRINGFORCEFIELD_INL


#include <sofa/core/behavior/ForceField.inl>
#include "ExtendedRestShapeSpringForceField.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/helper/gl/template.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <sofa/helper/AdvancedTimer.h>


namespace sofa
{

namespace component
{

namespace forcefield
{

template<class DataTypes>
ExtendedRestShapeSpringForceField<DataTypes>::ExtendedRestShapeSpringForceField()
    : points(initData(&points, "points", "points controlled by the rest shape springs"))
    , stiffness(initData(&stiffness, "stiffness", "stiffness values between the actual position and the rest shape position"))
    , angularStiffness(initData(&angularStiffness, "angularStiffness", "angularStiffness assigned when controlling the rotation of the points"))
    , pivotPoints(initData(&pivotPoints, "pivot_points", "global pivot points used when translations instead of the rigid mass centers"))
    , external_rest_shape(initData(&external_rest_shape, "external_rest_shape", "rest_shape can be defined by the position of an external Mechanical State"))
    , external_points(initData(&external_points, "external_points", "points from the external Mechancial State that define the rest shape springs"))
    , recompute_indices(initData(&recompute_indices, false, "recompute_indices", "Recompute indices (should be false for BBOX)"))
    , restMState(NULL)
    , forceDir(initData(&forceDir,std::string(""),"forceDir","directory to print forces in each step"))
    , f_startDTAppl(initData(&f_startDTAppl, 0, "startTimeSpringOn", "time step when the forcefield starts to be applied"))
    , f_numDTAppl(initData(&f_numDTAppl,0,"numStepsSpringOn", "number of time steps to apply the forcefield"))
    , f_updateStiffness(initData(&f_updateStiffness,false,"updateStiffness","update stiffness on begin animate event (activates listening)"))
    , f_totalForce(initData(&f_totalForce,"totalForce","total forces summed over all springs (naive summation)"))    
    , springThickness(initData(&springThickness,double(0.0),"springThickness","thickness of the spring, 0: no spring"))
    , pointSize(initData(&pointSize,double(0.0),"pointSize","size of a point indicating a position of the spring even if length is zero"))
    , springColor(initData(&springColor,"springColor","spring color"))
    , showIndicesScale(initData(&showIndicesScale, (float) 0.02, "showIndicesScale", "Scale for indices display. (default=0.02)"))
//	, pp_0(NULL)
{        
}


template<class DataTypes>
void ExtendedRestShapeSpringForceField<DataTypes>::bwdInit()
{
    core::behavior::ForceField<DataTypes>::init();

    if (stiffness.getValue().empty())
    {
        std::cout << "ExtendedRestShapeSpringForceField : No stiffness is defined, assuming equal stiffness on each node, k = 100.0 " << std::endl;

        VecReal stiffs;
        stiffs.push_back(100.0);
        stiffness.setValue(stiffs);
    }

    const std::string path = external_rest_shape.getValue();

    restMState = NULL;

    if (path.size() > 0)
    {
        this->getContext()->get(restMState ,path);
    }

    std::cout << "Path: " << path << " restMS: " << restMState << std::endl;
    if (!restMState)
    {        
        useRestMState = false;

        if (path.size() > 0)
        {
            std::cout << "[" << this->getName() <<"]: " << external_rest_shape.getValue() << " not found" << std::endl;
        }
    }
    else
    {
        useRestMState = true;

        // sout << "ExtendedRestShapeSpringForceField : Mechanical state named " << restMState->getName() << " found for RestShapeSpringFF named " << this->getName() << sendl;
    }

    if (useRestMState)
        std::cout << "[" << this->getName() << "]: using the external state " << restMState->getName() << std::endl;
    else
        std::cout << "[" << this->getName() << "]: using the rest state " << this->mstate->getName() << std::endl;


    this->k = stiffness.getValue();

    recomputeIndices();
    nbStep = 0;
    actualStep = 0;

    if (f_numDTAppl.getValue() == 0 || f_startDTAppl.getValue() == 0) {
        applyFactor = 1.0;        
    } else {
        if (this->f_listening.getValue() == false) {
            std::cout << this->getName() << ": WARNING listening false!" << std::endl;
        }
    }


#ifdef SOFA_HAVE_EIGEN2
    core::behavior::BaseMechanicalState* state = this->getContext()->getMechanicalState();
    assert(state);
    matS.resize(state->getMatrixSize(),state->getMatrixSize());
    lastUpdatedStep = -1.0;
#endif

    if (f_updateStiffness.getValue())
        this->f_listening.setValue(true);

    this->getContext()->get(colorMap);
    if (!colorMap) {
        serr << this->getName() << ": no color map found, using default." << sendl;        
        colorMap = sofa::helper::ColorMap::getDefault();
    } else
        sout << this->getName() << ": using color map found in the node" << sendl;
}


template<class DataTypes>
void ExtendedRestShapeSpringForceField<DataTypes>::recomputeIndices()
{
    m_indices.clear();
    m_ext_indices.clear();

    for (unsigned int i = 0; i < points.getValue().size(); i++)
        m_indices.push_back(points.getValue()[i]);

    for (unsigned int i = 0; i < external_points.getValue().size(); i++)
        m_ext_indices.push_back(external_points.getValue()[i]);

    m_pivots = pivotPoints.getValue();

    if (m_indices.size()==0)
    {
        //	std::cout << "in ExtendedRestShapeSpringForceField no point are defined, default case: points = all points " << std::endl;

        for (unsigned int i = 0; i < (unsigned)this->mstate->getSize(); i++)
        {
            m_indices.push_back(i);
        }
    }

    if (m_ext_indices.size()==0)
    {
        // std::cout << "in ExtendedRestShapeSpringForceField no external_points are defined, default case: points = all points " << std::endl;

        if (useRestMState)
        {
            for (unsigned int i = 0; i < (unsigned)restMState->getSize(); i++)
            {
                m_ext_indices.push_back(i);
            }
        }
        else
        {
            for (unsigned int i = 0; i < (unsigned)this->mstate->getSize(); i++)
            {
                m_ext_indices.push_back(i);
            }
        }
    }

    if (m_indices.size() > m_ext_indices.size())
    {
        std::cerr << "Error : the dimention of the source and the targeted points are different " << std::endl;
        m_indices.clear();
    }
}


template<class DataTypes>
const typename ExtendedRestShapeSpringForceField<DataTypes>::DataVecCoord* ExtendedRestShapeSpringForceField<DataTypes>::getExtPosition() const
{
    return (useRestMState ? restMState->read(core::VecCoordId::position()) : this->mstate->read(core::VecCoordId::restPosition()));
}

template<class DataTypes>
void ExtendedRestShapeSpringForceField<DataTypes>::addForce(const core::MechanicalParams* /* mparams */ /* PARAMS FIRST */, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& /* v */)
{
    sofa::helper::WriteAccessor< DataVecDeriv > f1 = f;
    sofa::helper::ReadAccessor< DataVecCoord > p1 = x;
    sofa::helper::ReadAccessor< DataVecCoord > p0 = *getExtPosition();

    //std::cout << this->getName() << "P1 = " << p1 << std::endl;
    //std::cout << this->getName() << "P0 = " << p0 << std::endl;
    //std::cout << this->getName() << "F(begin) = " << f << std::endl;

    if (this->f_printLog.getValue())
        std::cout << "[" <<  this->getName() << "]: addForce" << std::endl;


    f1.resize(p1.size());

    if (recompute_indices.getValue())
    {
        recomputeIndices();
    }

    //Springs_dir.resize(m_indices.size() );

    Deriv totalForce(0.0,0.0,0.0);
    if ( k.size()!= m_indices.size() )
    {
        //sout << "WARNING : stiffness is not defined on each point, first stiffness is used" << sendl;
        const Real k0 = k[0];

        std::ofstream file;
        char s[100];
        if (!forceDir.getValue().empty()) {
            //std::stringstream ss;
            //ss << forceDir.getValue() << "/forces_" << nbStep << ".out";
            sprintf(s, "%s/force%04d.out", forceDir.getValue().c_str(), nbStep);
            //sprintf(s, "%s/force_final.out", forceDir.getValue().c_str());
            file.open(s);
        }


        for (unsigned int i=0; i<m_indices.size(); i++)
        {
            const unsigned int index = m_indices[i];

            unsigned int ext_index = m_indices[i];
            if(useRestMState)
                ext_index= m_ext_indices[i];

            Deriv dx = p1[index] - p0[ext_index];
            //Springs_dir[i] = p1[index] - p0[ext_index];
            //Springs_dir[i].normalize();
            f1[index] -=  dx * k0 * applyFactor ;
            totalForce += dx * k0;

            //std::cout << "Added force[" << i << "]: " << dx <<  " * " << k0 << " = " << dx * k0 << std::endl;
            //std::cout << "Added force[" << i << "]: " << dx * k0 << std::endl;

            if (!forceDir.getValue().empty()) {
                file << -dx*k0 << std::endl;
            }

            //	if (dx.norm()>0.00000001)
            //		std::cout<<"force on point "<<index<<std::endl;

            //	Deriv dx = p[i] - p_0[i];
            //	f[ indices[i] ] -=  dx * k[0] ;
        }
        //std::cout << "[" << this->getName() << "] Total force: " << totalForce << std::endl;

        if (!forceDir.getValue().empty()) {

            file.close();
            char cmd[200];
            sprintf( cmd, "/bin/cp %s %s/force_final.out", s, forceDir.getValue().c_str());
            system( cmd);
        }
        nbStep++;
    }
    else
    {
        for (unsigned int i=0; i<m_indices.size(); i++)
        {
            const unsigned int index = m_indices[i];
            unsigned int ext_index = m_indices[i];
            if(useRestMState)
                ext_index= m_ext_indices[i];

            Deriv dx = p1[index] - p0[ext_index];
            //Springs_dir[i] = p1[index] - p0[ext_index];
            //Springs_dir[i].normalize();
            f1[index] -=  dx * k[i] * applyFactor;

            //	if (dx.norm()>0.00000001)
            //		std::cout<<"force on point "<<index<<std::endl;

            //	Deriv dx = p[i] - p_0[i];
            //	f[ indices[i] ] -=  dx * k[i] ;
        }
    }
    f_totalForce.setValue(totalForce);
    //std::cout << this->getName() << "F(end) = " << f << std::endl;
}


template<class DataTypes>
void ExtendedRestShapeSpringForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& df, const DataVecDeriv& dx)
{
    //  remove to be able to build in parallel
    // 	const VecIndex& indices = points.getValue();
    // 	const VecReal& k = stiffness.getValue();

    if (this->f_printLog.getValue())
        std::cout << "[" <<  this->getName() << "]: addDforce" << std::endl;


    sofa::helper::WriteAccessor< DataVecDeriv > df1 = df;
    sofa::helper::ReadAccessor< DataVecDeriv > dx1 = dx;
    Real kFactor = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    if (k.size()!= m_indices.size() )
    {
        //sout << "WARNING : stiffness is not defined on each point, first stiffness is used" << sendl;
        const Real k0 = k[0];

        for (unsigned int i=0; i<m_indices.size(); i++)
        {
            df1[m_indices[i]] -=  dx1[m_indices[i]] * k0 * kFactor * applyFactor;
        }
    }
    else
    {
        for (unsigned int i=0; i<m_indices.size(); i++)
        {
            df1[m_indices[i]] -=  dx1[m_indices[i]] * k[i] * kFactor * applyFactor ;
        }
    }
}

template<class DataTypes>
void ExtendedRestShapeSpringForceField<DataTypes>::draw(const core::visual::VisualParams * vparams)
{    
    if (vparams->displayFlags().getShowForceFields()) {
        if(springThickness.getValue() > 0.0)
        {
            const VecIndex& indices = points.getValue();
            const VecIndex& ext_indices=external_points.getValue();

            sofa::helper::ReadAccessor< DataVecCoord > p0 = *getExtPosition();
            sofa::helper::ReadAccessor< DataVecCoord > p = this->mstate->read(core::VecCoordId::position());


            std::vector< defaulttype::Vector3 > points;

            for (unsigned int i=0; i<indices.size(); i++)
            {
                const unsigned int index = indices[i];
                points.push_back(p[index]);

                if(useRestMState)
                {
                    const unsigned int ext_index = ext_indices[i];
                    points.push_back(p0[ext_index]);
                }
                else
                {
                    points.push_back(p0[index]);
                }

                vparams->drawTool()->drawLines(points,5,springColor.getValue());
            }
        }
    }

}

template<class DataTypes>
void ExtendedRestShapeSpringForceField<DataTypes>::addKToMatrix(const core::MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix )
{    
    //      remove to be able to build in parallel
    // 	const VecIndex& indices = points.getValue();
    // 	const VecReal& k = stiffness.getValue();
    //if (matrixAlreadyAssembled)
    //    return;

    if (this->f_printLog.getValue())
        std::cout << "[" <<  this->getName() << "]: addKToMatrix" << std::endl;

    sofa::helper::AdvancedTimer::stepBegin("restShapeSpringAddKToMatrix");
    sofa::core::behavior::MultiMatrixAccessor::MatrixRef mref = matrix->getMatrix(this->mstate);
    sofa::defaulttype::BaseMatrix* mat = mref.matrix;
    unsigned int offset = mref.offset;
    Real kFact = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    /*std::cout << this->getName() << ": !!!!!!  stiffness: ";
    for (size_t i = 0; i < k.size(); i++)
        std::cout << k[i] << " ";
    std::cout << std::endl;*/

    const int N = Coord::total_size;

    unsigned int curIndex = 0;

    size_t numNNZ = 0;
    if (k.size()!= m_indices.size() )
    {
        const Real k0 = k[0];
        for (unsigned int index = 0; index < m_indices.size(); index++)
        {
            curIndex = m_indices[index];

            for(int i = 0; i < N; i++)
            {

                //	for (unsigned int j = 0; j < N; j++)
                //	{
                //		mat->add(offset + N * curIndex + i, offset + N * curIndex + j, kFact * k[0]);
                //	}

                mat->add(offset + N * curIndex + i, offset + N * curIndex + i, -kFact * k0);
                if (this->f_printLog.getValue())
                    std::cout << this->getName() << " " << numNNZ << ": [" << offset + N +curIndex + i << "]+= " << -kFact << " * " << k[index] << std::endl;
            }
        }
    }
    else
    {
        for (unsigned int index = 0; index < m_indices.size(); index++)
        {
            curIndex = m_indices[index];

            for(int i = 0; i < N; i++)
            {

                //	for (unsigned int j = 0; j < N; j++)
                //	{
                //		mat->add(offset + N * curIndex + i, offset + N * curIndex + j, kFact * k[curIndex]);
                //	}

                mat->add(offset + N * curIndex + i, offset + N * curIndex + i, -kFact * k[index]);
                if (this->f_printLog.getValue())
                    std::cout << this->getName() << " " << numNNZ << ": [" << offset + N +curIndex + i << "]+= " << -kFact << " * " << k[index] << std::endl;
                numNNZ++;
            }
        }
    }
    sofa::helper::AdvancedTimer::stepEnd("restShapeSpringAddKToMatrix");
}

template<class DataTypes>
void ExtendedRestShapeSpringForceField<DataTypes>::addSubKToMatrix(const core::MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix, const helper::vector<unsigned> & addSubIndex )
{
    //      remove to be able to build in parallel
    // 	const VecIndex& indices = points.getValue();
    // 	const VecReal& k = stiffness.getValue();
    sofa::core::behavior::MultiMatrixAccessor::MatrixRef mref = matrix->getMatrix(this->mstate);
    sofa::defaulttype::BaseMatrix* mat = mref.matrix;
    unsigned int offset = mref.offset;
    Real kFact = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

    const int N = Coord::total_size;

    unsigned int curIndex = 0;

    if (k.size()!= m_indices.size() )
    {
        const Real k0 = k[0];
        for (unsigned int index = 0; index < m_indices.size(); index++)
        {
            curIndex = m_indices[index];

            bool contains=false;
            for (unsigned s=0;s<addSubIndex.size() && !contains;s++) if (curIndex==addSubIndex[s]) contains=true;
            if (!contains) continue;

            for(int i = 0; i < N; i++)
            {

                //	for (unsigned int j = 0; j < N; j++)
                //	{
                //		mat->add(offset + N * curIndex + i, offset + N * curIndex + j, kFact * k[0]);
                //	}

                mat->add(offset + N * curIndex + i, offset + N * curIndex + i, -kFact * k0);
            }
        }
    }
    else
    {
        for (unsigned int index = 0; index < m_indices.size(); index++)
        {
            curIndex = m_indices[index];

            bool contains=false;
            for (unsigned s=0;s<addSubIndex.size() && !contains;s++) if (curIndex==addSubIndex[s]) contains=true;
            if (!contains) continue;

            for(int i = 0; i < N; i++)
            {

                //	for (unsigned int j = 0; j < N; j++)
                //	{
                //		mat->add(offset + N * curIndex + i, offset + N * curIndex + j, kFact * k[curIndex]);
                //	}

                mat->add(offset + N * curIndex + i, offset + N * curIndex + i, -kFact * k[index]);
            }
        }
    }
}


template <class DataTypes>
void ExtendedRestShapeSpringForceField<DataTypes>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {        
        if (f_updateStiffness.getValue())
            k = stiffness.getValue();

        if (f_numDTAppl.getValue() == 0 || f_startDTAppl.getValue() == 0) {
            applyFactor = 1.0;
            return;
        }

        if (actualStep > f_startDTAppl.getValue()) {
            applyFactor = Real(actualStep - f_startDTAppl.getValue())/Real(abs(f_numDTAppl.getValue()));
            applyFactor = (applyFactor > 1.0) ? 1.0 : applyFactor;
        }
        else
            applyFactor = 0.0;


        if (f_numDTAppl.getValue() < 0)
            applyFactor = 1.0 - applyFactor;

        actualStep++;
        std::cout << this->getName() << "[" << actualStep << "]  applyFactor = " << applyFactor << std::endl;
    }
}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_RESTSHAPESPRINGFORCEFIELD_INL



