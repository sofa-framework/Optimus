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
#ifndef SOFA_COMPONENT_FORCEFIELD_BUBBLEPACKINGFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_BUBBLEPACKINGFORCEFIELD_H

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif
#include "initOptimusPlugin.h"
#include <sofa/core/behavior/ForceField.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/helper/fixed_array.h>
#include <sofa/helper/vector.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/core/visual/VisualModel.h>

#include <sofa/core/VecId.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/core/behavior/MultiVec.h>



namespace sofa
{

namespace component
{


namespace forcefield
{

/**
 * This class simulates the phenomenon of Bubble Packing
 * It requires to define active and passive bubbles and to classify them:
 *      - First, the passive bubbles [0 ; nbPassive-1]
 *      - Second, the active ones [nbPassive ; nbPassive+nbActive-1]
 */


    using namespace sofa::defaulttype;
    using namespace sofa::component::topology;
    using namespace sofa::component::container;





template<class DataTypes>
class BubblePackingForceField;



  template<class DataTypes>
  class BubblePackingForceField : public core::behavior::ForceField<DataTypes>
  {
    public:
       SOFA_CLASS(SOFA_TEMPLATE(BubblePackingForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));


       typedef core::behavior::ForceField<DataTypes> Inherited;
       typedef typename DataTypes::VecCoord VecCoord;
       typedef typename DataTypes::VecDeriv VecDeriv;
       typedef typename DataTypes::Coord    Coord   ;
       typedef typename DataTypes::Deriv    Deriv   ;
       typedef typename Coord::value_type   Real    ;

       // Vectors GPU compatible
       typedef typename VecCoord::template rebind<unsigned int>::other VecUInt;
       typedef typename VecCoord::template rebind<Real>::other VecReal;


       /// assumes the mechanical object type (3D)
       typedef Vec<3,Real>                            Vec3;
       typedef Vec<2,Real>                            Vec2;
       typedef StdVectorTypes< Vec3, Vec3, Real >     MechanicalTypes;
       typedef MechanicalObject<MechanicalTypes>      MechObject;
       typedef typename MechanicalTypes::Real                  MecaReal;

       typedef core::objectmodel::Data<VecCoord> DataVecCoord;
       typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;



       /// Constructor
       BubblePackingForceField();
       /// Destructor
       virtual ~BubblePackingForceField();
       //@{
       /** Other usual SOFA functions */
       virtual void init();
       virtual void reinit();
       virtual void draw(const core::visual::VisualParams*);
       //@}

       /// Forcefield functions for Matrix system. Adding force to global forcefield vector.
       virtual void addForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v);

       /// Forcefield functions for Matrix system. Adding derivate force to global forcefield vector.
       virtual void addDForce(const sofa::core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, DataVecDeriv& df , const DataVecDeriv& dx );

       /// Forcefield functions for Matrix system. Adding derivate force to global forcefield vector. (direct solver)
       void addKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix);

       /// Return Potential energy of the mesh.
       virtual double getPotentialEnergy(const VecCoord& x) const;

       /// handle topological changes
       virtual void handleTopologyChange(); //old version
       
       virtual double getPotentialEnergy(const sofa::core::MechanicalParams* /*mparams*/ /* PARAMS FIRST */, const DataVecCoord&   x) const {
           return 0.0;
       }


       //Definition of DATA
       /// Equilibrium distance between active particles
       Data<Real> m_l01;
       /// Equilibrium distance between active and passive particles
       Data<Real> m_l02;
       /// Stiffness for active particles
       Data<Real> m_k01;
       /// Stiffness for active and passive particles
       Data<Real> m_k02;
       /// Damping value
       Data<Real> m_damping;
       /// Number of active bubble
       Data<unsigned int> m_nbActive;
       /// Number of passive bubble
       Data<unsigned int> m_nbPassive;
       /// Visual radius for active bubble
       Data<Real> m_radiusActive;
       /// Visual radius for passive bubble
       Data<Real> m_radiusPassive;

    protected:
       Real l01;
       Real k01;
       Real l02;
       Real k02;

       Real alpha1;
       Real beta1;
       Real gamma1;
       Real epsilon1;

       Real alpha2;
       Real beta2;
       Real gamma2;
       Real epsilon2;


       /// Link to the mechanical state
       sofa::core::behavior::MechanicalState<DataTypes> *mState;

};

#if defined(WIN32) && !defined(SOFA_COMPONENT_FORCEFIELD_BUBBLEPACKINGFORCEFIELD_CPP)
#pragma warning(disable : 4231)
#ifndef SOFA_FLOAT
  template class BubblePackingForceField<Vec1dTypes>;
  template class BubblePackingForceField<Vec2dTypes>;
  template class BubblePackingForceField<Vec3dTypes>;
#endif //SOFA_FLOAT
#ifndef SOFA_DOUBLE
  template class BubblePackingForceField<Vec1fTypes>;
  template class BubblePackingForceField<Vec2fTypes>;
  template class BubblePackingForceField<Vec3fTypes>;
#endif //SOFA_DOUBLE
#endif

} //namespace forcefield

} // namespace Components


} // namespace Sofa

#endif /* SOFA_COMPONENT_FORCEFIELD_BUBBLEPACKINGFORCEFIELD_H */
