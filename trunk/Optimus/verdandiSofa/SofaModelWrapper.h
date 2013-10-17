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
#ifndef SOFA_SIMULATION_SOFA_MODEL_WRAPPER_H
#define SOFA_SIMULATION_SOFA_MODEL_WRAPPER_H

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseAnimationLoop.h>
#include <sofa/core/ExecParams.h>
#include <sofa/simulation/common/common.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/helper/AdvancedTimer.h>

#include "../src/OptimParams.h"

/*#define VERDANDI_DEBUG_LEVEL_4
#define SELDON_WITH_BLAS
#define SELDON_WITH_LAPACK

#define VERDANDI_WITH_ABORT
#define VERDANDI_DENSE

#define VERDANDI_WITH_DIRECT_SOLVER
//#define SELDON_WITH_MUMPS

//#include "seldon/SeldonSolver.hxx"

//#include "model/ClampedBar.cxx"
//#include "method/ForwardDriver.cxx"

//#include "Verdandi.hxx"
//#include "method/ForwardDriver.cxx"*/

//#include "VerdandiAnimationLoop.h"

#define VERDANDI_DEBUG_LEVEL_4

#include "VerdandiHeader.hxx"
#include "seldon/SeldonHeader.hxx"
#include "seldon/computation/basic_functions/Functions_Matrix.cxx"
#include "seldon/computation/basic_functions/Functions_Vector.cxx"
#include "seldon/computation/basic_functions/Functions_MatVect.cxx"

#include "sofa/component/projectiveconstraintset/FixedConstraint.h"

using namespace sofa::core::objectmodel;
using namespace sofa::core::behavior;

namespace sofa
{

namespace simulation
{

/**
 *  \brief Wrapper class implementing an interface between SOFA and Verdandi
 */

enum FilterType { UNDEF, FORWARD, UKF, ROUKF };

template <class Type>
class SOFA_SIMULATION_COMMON_API SofaModelWrapper : public Verdandi::VerdandiBase, public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SofaModelWrapper,Type),sofa::core::objectmodel::BaseObject);
    /// SOFA TYPES
    typedef sofa::core::objectmodel::BaseObject Inherit;

    typedef sofa::component::container::OptimParams<sofa::helper::vector<Type> > OPVector;


    /// VERDANDI TYPES

    //! The numerical type (e.g., double).
    typedef Type value_type;
    //! Pointer to the numerical type.
    typedef Type* pointer;
    //! Const pointer to the numerical type.
    typedef const Type* const_pointer;
    //! Reference to the numerical type.
    typedef Type& reference;
    //! Const reference to the numerical type.
    typedef const Type& const_reference;
#ifdef VERDANDI_SPARSE
    //! Type of the background error covariance matrix.
    typedef Seldon::Matrix<Type, General, RowSparse> state_error_variance;
    //! \brief Type of the reduced matrix \f$U\f$ in the \f$LUL^T\f$
    //  decomposition of the background error covariance matrix.
    typedef Seldon::Matrix<Type, General, RowSparse> state_error_variance_reduced;
    //! Type of a row of the background error variance.
    typedef Seldon::Vector<Type> state_error_variance_row;
    //! Type of the model/observation crossed matrix.
    typedef Seldon::Matrix<Type, General, RowSparse> matrix_state_observation;
    //! Type of the tangent linear operator.
    typedef Seldon::Matrix<Type, General, RowSparse> tangent_linear_operator;
#else
    //! Type of the background error covariance matrix.
    typedef Seldon::Matrix<Type> state_error_variance;
    // ! \brief Type of the reduced matrix \f$U\f$ in the \f$LUL^T\f$
    //  decomposition of the background error covariance matrix.
    typedef Seldon::Matrix<Type> state_error_variance_reduced;
    //! Type of a row of the background error variance.
    typedef Seldon::Vector<Type> state_error_variance_row;
    //! Type of the model/observation crossed matrix.
    typedef Seldon::Matrix<Type> matrix_state_observation;
    //! Type of the tangent linear operator.
    typedef Seldon::Matrix<Type> tangent_linear_operator;
#endif
    //! Type of the model state vector.
    typedef Seldon::Vector<Type> state;
    //! Collection of vector state.
    typedef Seldon::Vector<state, Seldon::Collection> state_collection;

    typedef core::behavior::MechanicalState<defaulttype::Vec3dTypes> MechStateVec3d;
    typedef sofa::component::projectiveconstraintset::FixedConstraint<sofa::defaulttype::Vec3dTypes> FixedConstraintVec3d;

    typedef struct {
        simulation::Node* gnode;

        FilterType filterType;
        bool positionInState;
        bool velocityInState;
        double errorVarianceSofaState;
        double errorVarianceSofaParams;
        bool verbose;
    } ModelData;

public:
    const core::ExecParams* execParams;
    int numStep;

    int current_row_;
    size_t dim_;
    size_t state_size_;
    size_t reduced_state_size_;
    size_t reduced_state_index_;
    size_t free_nodes_size;

    state state_, duplicated_state_;

    OPVector* vecParams;
    MechStateVec3d * mechanicalObject;
    FixedConstraintVec3d* fixedConstraints;
    helper::vector<size_t> freeIndices;


    //bool positionInState, velocityInState, verbose;

    /// error variance
    //double state_error_variance_state_, state_error_variance_params_;

    //! Background error covariance matrix (B).
    state_error_variance state_error_variance_;
    //! Inverse of the background error covariance matrix (B^-1).
    state_error_variance state_error_variance_inverse_;
    //! Value of the row of B currently stored.
    state_error_variance_row state_error_variance_row_;

    /*! \brief Projector matrix L in the decomposition of the
      background error covariance matrix (\f$B\f$) as a product LUL^T */
    state_error_variance state_error_variance_projector_;
    /*! \brief Reduced matrix U in the decomposition of the
      background error covariance matrix (\f$B\f$) as a product LUL^T */
    state_error_variance_reduced state_error_variance_reduced_;
    //! Is state error variance projector allocated?
    bool variance_projector_allocated_;
    //! Is reduced state error variance allocated?
    bool variance_reduced_allocated_;



    double time_;

    ModelData modelData;


public:
    SofaModelWrapper();
    virtual ~SofaModelWrapper();

    void setInitStepData(const core::ExecParams* _execParams) {
        execParams = _execParams;     
    }

    void initSimuData(ModelData& _md);

    /// functions propagating state between SOFA and Verdandi

    void StateSofa2Verdandi();
    void StateVerdandi2Sofa();

    /// functions required by Verdandi API:

    int GetNstate() const { return state_size_; }
    double GetTime() { return time_; }
    state& GetState();
    void GetStateCopy(state& _copy);

    void Initialize(std::string &);
    void InitializeStep() { numStep++; time_ = numStep*modelData.gnode->getDt(); }
    void Finalize() {}
    void FinalizeStep();
    bool HasFinished() { return(false); }
    void StateUpdated();
    void SetTime(double _time);

    double ApplyOperator(state& _x, bool _preserve_state = true, bool _update_force = true);
    void Forward(bool _update_force = true, bool _update_time = true);

    void Message(string _message);
    void Verb(string _s) {
        if (modelData.verbose)
            std::cout << "[" << this->getName() << "]: " << _s << std::endl;
    }

    state_error_variance& GetStateErrorVariance();
    state_error_variance_row& GetStateErrorVarianceRow(int row);
    state_error_variance& GetStateErrorVarianceProjector();
    state_error_variance_reduced& GetStateErrorVarianceReduced();


    /// Construction method called by ObjectFactory.
    /*template<class T>
    static typename T::SPtr create(T*, BaseContext* context, BaseObjectDescription* arg)
    {
        simulation::Node* gnode = dynamic_cast<simulation::Node*>(context);
        typename T::SPtr obj = sofa::core::objectmodel::New<T>(gnode);
        if (context) context->addObject(obj);
        if (arg) obj->parse(arg);
        return obj;
    }*/

    void printMatrix(Seldon::Matrix<Type>& M) {
        for (int i = 0; i < M.GetM(); i++)
          {
            for (int j = 0; j < M.GetN(); j++)
              std::cout << M(i, j) << '\t';
            std::cout << std::endl;
          }
    }

private :

    //simulation::Node* gnode;  ///< the node controlled by the loop

};



} // namespace simulation

} // namespace sofa

#endif  /* SOFA_SIMULATION_SOFA_MODEL_WRAPPER_H */
