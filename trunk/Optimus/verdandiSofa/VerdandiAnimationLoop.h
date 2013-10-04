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
#ifndef SOFA_SIMULATION_VERDANDIANIMATIONLOOP_H
#define SOFA_SIMULATION_VERDANDIANIMATIONLOOP_H

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseAnimationLoop.h>
#include <sofa/core/ExecParams.h>
#include <sofa/simulation/common/common.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/helper/AdvancedTimer.h>

/*#define VERDANDI_DEBUG_LEVEL_4
#define SELDON_WITH_BLAS
#define SELDON_WITH_LAPACK

#define VERDANDI_WITH_ABORT
#define VERDANDI_DENSE

#define VERDANDI_WITH_DIRECT_SOLVER
//#define SELDON_WITH_MUMPS*/

//#include "SofaModelWrapper.h"

#include "Verdandi.hxx"
#include "method/ForwardDriver.cxx"


//#include "seldon/SeldonSolver.hxx"
//#include "model/ClampedBar.cxx"


using namespace sofa::core::objectmodel;
using namespace sofa::core::behavior;

namespace sofa
{

namespace simulation
{

template <class T>
class SOFA_SIMULATION_COMMON_API SofaModelWrapper : public sofa::core::objectmodel::BaseObject, public Verdandi::VerdandiBase
{
public:
    typedef sofa::core::objectmodel::BaseObject Inherit;
    //SOFA_CLASS(SofaModelWrapper,sofa::core::objectmodel::BaseObject);

    //! The numerical type (e.g., double).
    typedef T value_type;
    //! Pointer to the numerical type.
    typedef T* pointer;
    //! Const pointer to the numerical type.
    typedef const T* const_pointer;
    //! Reference to the numerical type.
    typedef T& reference;
    //! Const reference to the numerical type.
    typedef const T& const_reference;
#ifdef VERDANDI_SPARSE
    //! Type of the background error covariance matrix.
    typedef Seldon::Matrix<T, General, RowSparse> state_error_variance;
    /*! \brief Type of the reduced matrix \f$U\f$ in the \f$LUL^T\f$
      decomposition of the background error covariance matrix. */
    typedef Seldon::Matrix<T, General, RowSparse> state_error_variance_reduced;
    //! Type of a row of the background error variance.
    typedef Seldon::Vector<T> state_error_variance_row;
    //! Type of the model/observation crossed matrix.
    typedef Seldon::Matrix<T, General, RowSparse> matrix_state_observation;
    //! Type of the tangent linear operator.
    typedef Seldon::Matrix<T, General, RowSparse> tangent_linear_operator;
#else
    //! Type of the background error covariance matrix.
    typedef Seldon::Matrix<T> state_error_variance;
    /*! \brief Type of the reduced matrix \f$U\f$ in the \f$LUL^T\f$
      decomposition of the background error covariance matrix. */
    typedef Seldon::Matrix<T> state_error_variance_reduced;
    //! Type of a row of the background error variance.
    typedef Seldon::Vector<T> state_error_variance_row;
    //! Type of the model/observation crossed matrix.
    typedef Seldon::Matrix<T> matrix_state_observation;
    //! Type of the tangent linear operator.
    typedef Seldon::Matrix<T> tangent_linear_operator;
#endif
    //! Type of the model state vector.
    typedef Seldon::Vector<T> state;
    //! Collection of vector state.
    typedef Seldon::Vector<state, Seldon::Collection> state_collection;

protected:
    const core::ExecParams* execParams;

    state State;

public:
    SofaModelWrapper();
    virtual ~SofaModelWrapper();

    void setExecParams(const core::ExecParams* _execParams) {
        execParams = _execParams;
    }

    void SetNode(simulation::Node* _gnode);

    /// verdandi functions:

    void Finalize() {}
    void FinalizeStep() {}
    double GetTime() {
        return this->gnode->getDt();
    }

    bool HasFinished() {
        return(false);
    }

    void Initialize() {
    }

    void InitializeStep() {
    }

    void Forward();

    state& GetState() {
        return State;
    }
    /*virtual void setNode( simulation::Node* );

    /// Set the simulation node to the local context if not specified previously
    virtual void init();

    /// perform one animation step
    virtual void step(const core::ExecParams* params, double dt);


    /// Construction method called by ObjectFactory.
    template<class T>
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


/// /**********************************************************************************************/


class SOFA_SIMULATION_COMMON_API VerdandiAnimationLoop : public sofa::core::behavior::BaseAnimationLoop
{
public:
    typedef sofa::core::behavior::BaseAnimationLoop Inherit;
    SOFA_CLASS(VerdandiAnimationLoop,sofa::core::behavior::BaseAnimationLoop);
protected:
    VerdandiAnimationLoop(simulation::Node* gnode = NULL);

    //SofaModelWrapper* modelWrapper;

    Data<std::string> _configFile;

    virtual ~VerdandiAnimationLoop();
public:
    /// Set the simulation node this animation loop is controlling
    virtual void setNode( simulation::Node* _gnode);

    /// Set the simulation node to the local context if not specified previously
    virtual void init();

    /// perform one animation step
    virtual void step(const core::ExecParams* params, double /*dt*/);


    /// Construction method called by ObjectFactory.
    template<class T>
    static typename T::SPtr create(T*, BaseContext* context, BaseObjectDescription* arg)
    {
        simulation::Node* gnode = dynamic_cast<simulation::Node*>(context);
        typename T::SPtr obj = sofa::core::objectmodel::New<T>(gnode);
        if (context) context->addObject(obj);
        if (arg) obj->parse(arg);
        return obj;
    }

private :
    Verdandi::ForwardDriver<SofaModelWrapper<double> >* driver;

    simulation::Node* gnode;  ///< the node controlled by the loop

};


} // namespace simulation

} // namespace sofa

#endif  /* SOFA_SIMULATION_VERDANDIANIMATIONLOOP_H */
