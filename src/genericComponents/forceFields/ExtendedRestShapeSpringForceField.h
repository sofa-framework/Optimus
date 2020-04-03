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
#ifndef SOFA_COMPONENT_FORCEFIELD_EXTENDEDRESTSHAPESPRINGFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_EXTENDEDRESTSHAPESPRINGFORCEFIELD_H

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/helper/vector.h>
#include <sofa/core/BaseMapping.h>

#include <sofa/helper/ColorMap.h>

#ifdef SOFA_HAVE_EIGEN2
#include <sofa/component/linearsolver/EigenSparseMatrix.h>
#endif


namespace sofa
{
namespace core
{
namespace behavior
{
template< class T > class MechanicalState;

} // namespace behavior
} // namespace core
} // namespace sofa

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
class ExtendedRestShapeSpringForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ExtendedRestShapeSpringForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    typedef core::behavior::ForceField<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::CPos CPos;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::Real Real;
    typedef helper::vector< unsigned int > VecIndex;
    typedef helper::vector< Real >	 VecReal;

    typedef core::objectmodel::Data<VecCoord> DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;


    Data< helper::vector< unsigned int > > points;
    Data< VecReal > stiffness;
    Data< VecReal > angularStiffness;
    Data< helper::vector< CPos > > pivotPoints;
    Data< std::string > external_rest_shape;
    Data< helper::vector< unsigned int > > external_points;
    Data< bool > recompute_indices;    
    sofa::core::behavior::MechanicalState< DataTypes > *restMState;
    Data< std::string > forceDir;
    Data<int> f_startDTAppl;
    Data<int> f_numDTAppl;
    Data<bool> f_updateStiffness;
    Data<Coord> f_totalForce;    
    Data< double > springThickness;
    Data< double > pointSize;
    Data< sofa::defaulttype::Vec4f > springColor;
    Data<float> showIndicesScale;


    Real applyFactor;
    int actualStep;    

#ifdef SOFA_HAVE_EIGEN2
    linearsolver::EigenBaseSparseMatrix<typename DataTypes::Real> matS;    
#endif

    //VecDeriv Springs_dir;
protected:
    ExtendedRestShapeSpringForceField();
public:
    /// BaseObject initialization method.
    void bwdInit() override;

    /// Add the forces.
    virtual void addForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v) override;

    virtual void addDForce(const core::MechanicalParams* mparams /* PARAMS FIRST */, DataVecDeriv& df, const DataVecDeriv& dx) override;

    /// Brings ForceField contribution to the global system stiffness matrix.
    virtual void addKToMatrix(const core::MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix ) override;

    virtual void addSubKToMatrix(const core::MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix, const helper::vector<unsigned> & addSubIndex ) override;

    virtual void draw(const core::visual::VisualParams* vparams) override;

    virtual SReal getPotentialEnergy(const core::MechanicalParams* /*mparams*/, const DataVecCoord&  /* x */) const override
    {
        serr << "Get potentialEnergy not implemented" << sendl;
        return 0.0;
    }

    void addMBKToMatrix(const core::MechanicalParams* mparams /* PARAMS FIRST */, const sofa::core::behavior::MultiMatrixAccessor* matrix) override
    {
        sofa::core::BaseMapping *bmapping;
        this->getContext()->get(bmapping);
        if (bmapping ) /// do not call addKToMatrix since the object is mapped
            return;
        if (mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue()) != 0.0 )
            this->addKToMatrix(mparams /* PARAMS FIRST */, matrix);
        if (mparams->bFactor() != 0.0)
            this->addBToMatrix(mparams /* PARAMS FIRST */, matrix);
    }


    const DataVecCoord* getExtPosition() const;
    const VecIndex& getIndices() const { return m_indices; }
    const VecIndex& getExtIndices() const { return (useRestMState ? m_ext_indices : m_indices); }

protected :

    void recomputeIndices();

    VecIndex m_indices;
    VecReal k;
    VecIndex m_ext_indices;
    helper::vector<CPos> m_pivots;

    sofa::helper::ColorMap *colorMap;
    int nbStep;

#ifdef SOFA_HAVE_EIGEN2
    double lastUpdatedStep;
#endif
private :

    bool useRestMState; /// An external MechanicalState is used as rest reference.

    void handleEvent(core::objectmodel::Event *event) override;
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_EXTENDEDRESTSHAPESPRINGFORCEFIELD_CPP)

using namespace sofa::defaulttype;

#ifndef SOFA_FLOAT
extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Vec3dTypes>;
//extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Vec2dTypes>;
extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Vec1dTypes>;
//extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Vec6dTypes>;
extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Rigid3dTypes>;
//extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Rigid2dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Vec3fTypes>;
//extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Vec2fTypes>;
extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Vec1fTypes>;
//extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Vec6fTypes>;
extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Rigid3fTypes>;
//extern template class SOFA_DEFORMABLE_API ExtendedRestShapeSpringForceField<Rigid2fTypes>;
#endif

#endif // defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_FORCEFIELD_EXTENDEDRESTSHAPESPRINGFORCEFIELD_CPP)

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_EXTENDEDRESTSHAPESPRINGFORCEFIELD_H
