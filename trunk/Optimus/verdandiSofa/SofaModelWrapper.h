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

#include "VerdandiHeader.hxx"
#include "seldon/SeldonHeader.hxx"
#include "seldon/computation/basic_functions/Functions_Matrix.cxx"
#include "seldon/computation/basic_functions/Functions_Vector.cxx"
#include "seldon/computation/basic_functions/Functions_MatVect.cxx"




using namespace sofa::core::objectmodel;
using namespace sofa::core::behavior;

namespace sofa
{

namespace simulation
{

/**
 *  \brief Wrapper class implementing an interface between SOFA and Verdandi
 */

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

protected:
    const core::ExecParams* execParams;
    int numStep;

    int state_size_;
    state state_, duplicated_state_;




    OPVector* vecParams;
    core::behavior::MechanicalState<defaulttype::Vec3dTypes>* mechanicalObject;

    bool positionInState, velocityInState;

    /// error variance
    double state_error_variance_value_;

    //! Background error covariance matrix (B).
    state_error_variance state_error_variance_;
    //! Inverse of the background error covariance matrix (B^-1).
    state_error_variance state_error_variance_inverse_;
    //! Value of the row of B currently stored.
    state_error_variance_row state_error_variance_row_;

    int current_row_;

    double time_;


public:
    SofaModelWrapper();
    virtual ~SofaModelWrapper();

    void setInitStepData(const core::ExecParams* _execParams, bool _posInState, bool _velInState) {
        execParams = _execParams;
        positionInState = _posInState;
        velocityInState = _velInState;
    }

    void initSimuData( simulation::Node* _gnode, bool _posInState = true, bool _velInState = true, double _stateErrorVarianceValue = 1.0);

    /// functions propagating state between SOFA and Verdandi

    void StateSofa2Verdandi();
    void StateVerdandi2Sofa();

    /// functions required by Verdandi API:

    int GetNstate() const { return state_size_; }
    double GetTime() { return time_; }
    state& GetState();
    void GetStateCopy(state& _copy);

    void Initialize(std::string &);
    void InitializeStep() { time_ =  double(numStep)*this->gnode->getDt(); }
    void Finalize() {}
    void FinalizeStep() { numStep++; }
    bool HasFinished() { return(false); }
    void StateUpdated();
    void SetTime(double _time) { time_ = _time;}

    double ApplyOperator(state& _x, bool _preserve_state = true, bool _update_force = true);
    void Forward(bool _update_force = true);

    void Message(string _message);

    state_error_variance& GetStateErrorVariance();
    state_error_variance_row& GetStateErrorVarianceRow(int row);


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

private :

    simulation::Node* gnode;  ///< the node controlled by the loop

};



} // namespace simulation

} // namespace sofa

#endif  /* SOFA_SIMULATION_SOFA_MODEL_WRAPPER_H */
