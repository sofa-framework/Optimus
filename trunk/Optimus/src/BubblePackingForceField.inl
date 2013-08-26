/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include "BubblePackingForceField.h"
#include <fstream> // for reading the file
#include <iostream> //for debugging
#include <sofa/helper/gl/template.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/behavior/ForceField.inl>
#include <sofa/helper/AdvancedTimer.h>
#include <GL/glut.h>


namespace sofa
{

namespace component
{

namespace forcefield
{

    using namespace sofa::defaulttype;
    using namespace sofa::component::topology;
    using namespace core::topology;
    using namespace core::objectmodel;



// --------------------------------------------------------------------------------------
// ---  Constructor
// --------------------------------------------------------------------------------------
template <class DataTypes>
BubblePackingForceField<DataTypes>::BubblePackingForceField()
: m_l01(initData(&m_l01, (Real)1.0, "l01","Equilibrium distance between active particles"))
, m_l02(initData(&m_l02, (Real)1.0, "l02","Equilibrium distance between passive particles"))
, m_k01(initData(&m_k01, (Real)1.0, "k01","Stiffness for active particles"))
, m_k02(initData(&m_k02, (Real)1.0, "k02","Stiffness for passive particles"))
, m_damping(initData(&m_damping, (Real)0.0, "damping","Damping value c.dx/dt"))
, m_nbActive(initData(&m_nbActive, (unsigned int)1, "nbActive","Number of active bubble"))
, m_nbPassive(initData(&m_nbPassive, (unsigned int)1, "nbPassive","Number of passive bubble"))
, m_radiusActive(initData(&m_radiusActive, (Real)1, "radiusActive","Visual radius used for active bubbles"))
, m_radiusPassive(initData(&m_radiusPassive, (Real)1, "radiusPassive","Visual radius used for passive bubbles"))
{
}



// --------------------------------------------------------------------------------------
// ---  Destructor
// --------------------------------------------------------------------------------------
template <class DataTypes>
BubblePackingForceField<DataTypes>::~BubblePackingForceField()
{
}


// --------------------------------------------------------------------------------------
// ---  Initialization stage
// --------------------------------------------------------------------------------------
template <class DataTypes>
void BubblePackingForceField<DataTypes>::init()
{
    this->Inherited::init();
    if( (m_l01.getValue() <= 0.0) || (m_l02.getValue() <= 0.0) )
        serr<<"ERROR in the parameters l01/l02"<<sout;

    if( (m_nbActive.getValue() == 0) || (m_nbPassive.getValue() == 0) )
        serr<<"ERROR in the nb of bubbles"<<sout;

    // Computation of the parameters: alpha, beta. gamma, epsilon
    l01 = m_l01.getValue();
    k01 = m_k01.getValue();
    l02 = m_l02.getValue();
    k02 = m_k02.getValue();

    alpha1 = 5.0*k01/(l01*l01);
    beta1  = -19.0*k01/(28.0*l01);
    gamma1 = 0.0;
    epsilon1 = 47.0/48.0*k01*l01;

    alpha2 = 5.0*k02/(l02*l02);
    beta2  = -19.0*k02/(28.0*l02);
    gamma2 = 0.0;
    epsilon2 = 47.0/48.0*k02*l02;

    // Save the mechanical object link
    mState = dynamic_cast<core::behavior::MechanicalState<DataTypes> *>(this->getContext()->getMechanicalState());
}




// --------------------------------------------------------------------------------------
// ---  Reinit
// --------------------------------------------------------------------------------------
template <class DataTypes>
void BubblePackingForceField<DataTypes>::reinit()
{
}



//****************************************************************************************************************************

template <class DataTypes>
void BubblePackingForceField<DataTypes>::handleTopologyChange()
{
//    std::list<const TopologyChange *>::const_iterator itBegin=m_topology->beginChange();
//    std::list<const TopologyChange *>::const_iterator itEnd=m_topology->endChange();
}



template <class DataTypes>
double BubblePackingForceField<DataTypes>::getPotentialEnergy(const VecCoord& /*x*/) const
{
    serr<<"BubblePackingForceField::getPotentialEnergy-not-implemented !!!"<<sendl;
    return 0;
}


template <class DataTypes>
void BubblePackingForceField<DataTypes>::addForce (const core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, DataVecDeriv& dataf, const DataVecCoord& datax, const DataVecDeriv& /*v*/)
{
    sofa::helper::AdvancedTimer::stepBegin("addForce-BP");
    VecDeriv& f = *dataf.beginEdit() ;
    const VecCoord& x = datax.getValue();

    unsigned int nbActive  = m_nbActive.getValue();
    unsigned int nbPassive = m_nbPassive.getValue();

    Coord vectorDistance;
    Real distance, force;

    for(unsigned int i=0; i<nbActive; i++)
    {
        unsigned int activeBubbleID = nbPassive+i;
        for(unsigned int j=0; j<nbPassive; j++)
        {
            vectorDistance = x[j]-x[activeBubbleID];
            distance = sqrt(vectorDistance[0]*vectorDistance[0] + vectorDistance[1]*vectorDistance[1] + vectorDistance[2]*vectorDistance[2]);
            force = 0.0;
            if((distance>0.0) && (distance<=1.5*l02))
            {
                force = alpha2*distance*distance*distance + beta2*distance*distance + gamma2*distance + epsilon2;

                f[activeBubbleID][0] += force / distance * vectorDistance[0];
                f[activeBubbleID][1] += force / distance * vectorDistance[1];
                f[activeBubbleID][2] += force / distance * vectorDistance[2];
            }
        }
        for(unsigned int j=0; j<nbActive; j++)
        {
            if(j!=i)
            {
                vectorDistance = x[j+nbPassive]-x[activeBubbleID];
                distance = sqrt(vectorDistance[0]*vectorDistance[0] + vectorDistance[1]*vectorDistance[1] + vectorDistance[2]*vectorDistance[2]);
                force = 0.0;
                if((distance>0.0) && (distance<=1.5*l01))
                {
                    force = alpha1*distance*distance*distance + beta1*distance*distance + gamma1*distance + epsilon1;

                    f[activeBubbleID][0] += force / distance * vectorDistance[0];
                    f[activeBubbleID][1] += force / distance * vectorDistance[1];
                    f[activeBubbleID][2] += force / distance * vectorDistance[2];
                }
            }
        }
//        std::cout<<"--f[activeBubbleID] = "<< f[activeBubbleID]<<std::endl;
    }

    dataf.endEdit();
    sofa::helper::AdvancedTimer::stepEnd("addForce-BP");
}



template <class DataTypes>
void BubblePackingForceField<DataTypes>::addDForce(const sofa::core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv&   datadF , const DataVecDeriv&   datadX )
{
    if(m_damping.getValue()!=0.0)
    {
        VecDeriv& df = *datadF.beginEdit();
        const VecDeriv& dx = datadX.getValue();
        const Real& kFactor = mparams->kFactor();
        Real damping = m_damping.getValue();

//        std::cout<<"kFactor = "<<kFactor<<std::endl;

        for(unsigned int i=0; i<dx.size(); i++ )
        {
            df[i][0] += dx[i][0] * damping * kFactor;
            df[i][1] += dx[i][1] * damping * kFactor;
            df[i][2] += dx[i][2] * damping * kFactor;
        }
        datadF.endEdit();
    }
    else
    {
        (void)datadF;
        (void)datadX;
        mparams->setKFactorUsed(true);
    }
}




template <class DataTypes>
void BubblePackingForceField<DataTypes>::addKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
    (void)matrix;
    mparams->setKFactorUsed(true);
}




template<class DataTypes>
void BubblePackingForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowForceFields())
        return;
    if (!this->mstate)
        return;

    if (vparams->displayFlags().getShowWireFrame())
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    if (vparams->displayFlags().getShowWireFrame())
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    sofa::helper::ReadAccessor<Data<VecCoord> > position = *mState->read(core::VecCoordId::position());
    unsigned int nbPassive = m_nbPassive.getValue();

    glDisable(GL_LIGHTING);
    glColor3d(1.0, 1.0, 0.0);
    glPointSize(0.05f);
    std::vector< Vector3 > points;

    // Draw passive bubbles
    points.clear();
    points.resize(nbPassive);
    for(unsigned int i=0; i<nbPassive; i++)
        points[i] = position[i];

    vparams->drawTool()->drawSpheres(points, (float)(m_radiusPassive.getValue()), Vec<4,float>(1.0f,1.0f,0.35f,0.05f));

    // Draw active bubbles
    points.clear();
    points.resize(m_nbActive.getValue());
    for(unsigned int i=0; i<m_nbActive.getValue(); i++)
        points[i] = position[i+nbPassive];

    vparams->drawTool()->drawSpheres(points, (float)(m_radiusActive.getValue()), Vec<4,float>(0.5f,1.0f,0.35f,1.0f));

    glEnable(GL_LIGHTING);
}




} // namespace forcefield

} // namespace Components

} // namespace Sofa
