// Copyright (C) 2010 INRIA
// Author(s): Vivien Mallet, Claire Mouton
//
// This file is part of the data assimilation library Verdandi.
//
// Verdandi is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1 of the License, or (at your option)
// any later version.
//
// Verdandi is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
// more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Verdandi. If not, see http://www.gnu.org/licenses/.
//
// For more information, visit the Verdandi web site:
//      http://verdandi.gforge.inria.fr/


#ifndef VERDANDI_FILE_MODEL_MODELTEMPLATE_CXX


#include "ModelTemplate.hxx"


namespace Verdandi
{


    ////////////////////////////////
    // CONSTRUCTOR AND DESTRUCTOR //
    ////////////////////////////////


    //! Constructor.
    ModelTemplate::ModelTemplate()
    {
    }


    //! Destructor.
    ModelTemplate::~ModelTemplate()
    {
    }


    ////////////////
    // INITIALIZE //
    ////////////////


    //! Initializes the model.
    /*!
      \param[in] configuration_file configuration file.
    */
    void ModelTemplate::Initialize(string configuration_file)
    {
        throw ErrorUndefined("ModelTemplate"
                             "::Initialize(string configuration_file)");
    }


    //! Initializes the current time step for the model.
    void ModelTemplate::InitializeStep()
    {
        throw ErrorUndefined("ModelTemplate::InitializeStep()");
    }


    ////////////////
    // PROCESSING //
    ////////////////


    //! Advances one step forward in time.
    /*! \f[x^f_{h+1} = \mathcal{M}_h(x^a_h, p_h)\,.\f] */
    void ModelTemplate::Forward()
    {
        throw ErrorUndefined("ModelTemplate::Forward()");
    }


    //! Performs one step backward in adjoint model.
    /*!
      \param[in] observation_term \f$ H^T R^{-1}(y - Hx) \f$.
    */
    void ModelTemplate::BackwardAdjoint(state& observation_term)
    {
        throw ErrorUndefined("ModelTemplate::BackwardAdjoint(state&)");
    }


    //! Checks whether the model has finished.
    /*!
      \return True if the simulation is done, false otherwise.
    */
    bool ModelTemplate::HasFinished() const
    {
        throw ErrorUndefined("ModelTemplate::HasFinished() const");
    }


    //! Finalizes the current time step for the model.
    void ModelTemplate::FinalizeStep()
    {
        throw ErrorUndefined("ModelTemplate::FinalizeStep()");
    }


    //! Finalizes the model.
    void ModelTemplate::Finalize()
    {
        throw ErrorUndefined("ModelTemplate::Finalize()");
    }


    ////////////////////
    // PARALLEL MODEL //
    ////////////////////


#ifdef VERDANDI_WITH_MPI
    //! Sets the MPI communicator.
    /*!
     \param[in] mpi_communicator the MPI communicator to be set.
     */
    void ModelTemplate::SetMPICommunicator(MPI_Comm& mpi_communicator)
    {
        throw ErrorUndefined("ModelTemplate::SetMPICommunicator()");
    }
#endif


    ///////////////
    // OPERATORS //
    ///////////////


    //! Applies the model to a given vector.
    /*! The current state of the model is modified.
      \param[in,out] x on entry, the state vector to which the model should be
      applied; on exit, the result.
      \param[in] preserve_state Boolean to indicate if the model state has to
      be preserved.
      \return The time associated with \a x on exit plus one time step.
      \warning The time of the model has to be preserved.
    */
    double ModelTemplate::ApplyOperator(state& x, bool preserve_state)
    {
        throw ErrorUndefined("ModelTemplate::ApplyOperator(state& x, "
                             "bool forward, bool preserve_state)");
    }


    //! Applies the tangent linear model to a given vector.
    /*!
      \param[in,out] x on entry, the state vector to which the tangent linear
      model should be applied; on exit, the result.
      \return The time associated with \a x on exit. This time should be the
      model time plus one time step.
    */
    double ModelTemplate::ApplyTangentLinearOperator(state& x)
    {
        throw ErrorUndefined("ModelTemplate"
                             "::ApplyTangentLinearOperator(state& x)");
    }


    //! Gets the tangent linear model.
    /*!
      \param[out] A the matrix of the tangent linear model.
    */
    ModelTemplate::tangent_linear_operator& ModelTemplate
    ::GetTangentLinearOperator()
    {
        throw ErrorUndefined("ModelTemplate::tangent_linear_operatorx& "
                             "ModelTemplate::GetTangentLinearOperator()");
    }


    ////////////////////
    // ACCESS METHODS //
    ////////////////////


    //! Returns the current time.
    /*!
      \return The current time.
    */
    double ModelTemplate::GetTime()
    {
        throw ErrorUndefined("ModelTemplate::GetTime() const");
    }


    //! Sets the time of the model to a given time.
    /*!
      \param[in] time a given time.
    */
    void ModelTemplate::SetTime(double time)
    {
        throw ErrorUndefined("ModelTemplate::SetTime(double time)");
    }


    //! Returns the state vector size.
    /*!
      \return The state vector size.
    */
    int ModelTemplate::GetNstate()
    {
        throw ErrorUndefined("ModelTemplate::GetNstate()");
    }


    //! Returns the size of the full state vector.
    /*!
      \return The size of the full state vector.
    */
    int ModelTemplate::GetNfull_state()
    {
        throw ErrorUndefined("ModelTemplate::GetNfull_state()");
    }


    //! Provides the controlled state vector.
    /*!
      \return state the controlled state vector.
    */
    ModelTemplate::state& ModelTemplate::GetState()
    {
        throw ErrorUndefined("ModelTemplate::state& "
                             "ModelTemplate::GetState()");
    }


    //! Carries out some calculations when the model state has been updated.
    void ModelTemplate::StateUpdated()
    {
        throw ErrorUndefined("ModelTemplate"
                             "::StateUpdated");
    }


    //! Provides the state lower bound.
    /*!
      \return The state lower bound (componentwise).
    */
    ModelTemplate::state& ModelTemplate::GetStateLowerBound()
    {
        throw ErrorUndefined("ModelTemplate::state& ModelTemplate"
                             "::GetStateLowerBound(state& lower_bound)");
    }


    //! Provides the state upper bound.
    /*!
      \return The state upper bound (componentwise).
    */
    ModelTemplate::state& ModelTemplate::GetStateUpperBound()
    {
        throw ErrorUndefined("ModelTemplate::state& ModelTemplate"
                             "::GetStateUpperBound(state& upper_bound)");
    }


    //! Provides the full state vector.
    /*!
      \return state the controlled state vector.
    */
    ModelTemplate::state& ModelTemplate::GetFullState()
    {
        throw ErrorUndefined("ModelTemplate::state& "
                             "ModelTemplate::GetFullState()");
    }


    //! Carries out some calculations when the model state has been updated.
    void ModelTemplate::FullStateUpdated()
    {
        throw ErrorUndefined("ModelTemplate"
                             "::StateUpdated");
    }


    //! Returns the adjoint state vector.
    /*!
      \return The adjoint state vector.
    */
    ModelTemplate::state& ModelTemplate::GetAdjointState()
    {
        throw ErrorUndefined("ModelTemplate::state& ModelTemplate"
                             "::GetAdjointState()");
    }


    /*! Carries out some calculations when the adjoint state
      has been updated.
    */
    void ModelTemplate::AdjointStateUpdated()
    {
        throw ErrorUndefined("ModelTemplate::SetAdjointState(const state& "
                             "state_adjoint)");
    }


    //! Returns a term that will be added to the adjoint state.
    /*!
      \return The additional adjoint term.
    */
    ModelTemplate::state& ModelTemplate::GetAdditionalAdjointTerm()
    {
        throw ErrorUndefined("ModelTemplate::GetAdditionalAdjointTerm()");
    }


    //! Returns a term that will be added to the cost function.
    /*!
      \return The additional cost term.
    */
    double ModelTemplate::GetAdditionalCostTerm()
    {
        throw ErrorUndefined("ModelTemplate::GetAdditionalCostTerm()");
    }


    //! Returns the number of parameters to be perturbed.
    /*!
      \return The number of parameters to be perturbed.
    */
    int ModelTemplate::GetNparameter()
    {
        throw ErrorUndefined("ModelTemplate::GetNparameter()");
    }


    //! Gets the i-th uncertain parameter.
    /*!
      \param[in] i index of the parameter.
      \return The vector associated with the i-th parameter.
    */
    ModelTemplate::uncertain_parameter& ModelTemplate::GetParameter(int i)
    {
        throw ErrorUndefined("ModelTemplate::GetParameter(int i)");
    }


    /*! Carries out some calculations when the i-th parameter
      has been updated.
    */
    /*!
      \param[in] i index of the parameter.
    */
    void ModelTemplate::ParameterUpdated(int i)
    {
        throw ErrorUndefined("ModelTemplate::ParameterUpdated(int i)");
    }


    //! Returns the correlation between the uncertain parameters.
    /*!
      \param[in] i parameter index.
      \return The correlation between the uncertain parameters.
    */
    Vector<double>& ModelTemplate::GetParameterCorrelation(int i)
    {
        throw ErrorUndefined("ModelTemplate::GetParameterCorrelation(int i)");
    }


    //! Returns the PDF of the i-th parameter.
    /*!
      \param[in] i uncertain-variable index.
      \return The PDF of the i-th parameter.
    */
    string ModelTemplate::GetParameterPDF(int i)
    {
        throw ErrorUndefined("ModelTemplate::GetParameterPDF(int i)");
    }


    //! Returns the covariance matrix associated with the i-th parameter.
    /*!
      \param[in] i parameter index.
      \return The covariance matrix associated with the i-th parameter.
    */
    ModelTemplate::parameter_variance&
    ModelTemplate::GetParameterVariance(int i)
    {
        throw ErrorUndefined("ModelTemplate::GetParameterVariance(int i)");
    }


    //! Returns parameters associated with the PDF of some model parameter.
    /*! In case of normal or log-normal distribution, the parameters are
      clipping parameters.
      \param[in] i model parameter index.
      \return The parameters associated with the i-th parameter.
    */
    Vector<double>& ModelTemplate::GetParameterPDFData(int i)
    {
        throw ErrorUndefined("ModelTemplate::GetParameterParameter(int i)");
    }


    //! Returns the perturbation option of the i-th parameter.
    /*!
      \param[in] i parameter index.
      \return The perturbation option of the i-th parameter.
    */
    string ModelTemplate::GetParameterOption(int i)
    {
        throw ErrorUndefined("ModelTemplate::GetParameterOption(int i)");
    }


    ////////////
    // ERRORS //
    ////////////


    //! Computes a row of the variance of the state error.
    /*!
      \param[in] row row index.
      \return The row with index \a row in the state error variance.
    */
    ModelTemplate::state_error_variance_row&
    ModelTemplate::GetStateErrorVarianceRow(int row)
    {
        throw ErrorUndefined("ModelTemplate::state_error_variance_row& "
                             "ModelTemplate::GetStateErrorVarianceRow"
                             "(int row)");
    }


    //! Returns the state error variance.
    /*!
      \return The state error variance.
    */
    ModelTemplate::state_error_variance&
    ModelTemplate::GetStateErrorVariance()
    {
        throw ErrorUndefined("ModelTemplate::state_error_variance& "
                             "ModelTemplate::GetStateErrorVariance()");
    }


    /*! Returns the matrix L in the decomposition of the
      state error covariance matrix (\f$B\f$) as a product \f$LUL^T\f$.
    */
    /*!
      \return The matrix \f$L\f$.
    */
    ModelTemplate::state_error_variance&
    ModelTemplate::GetStateErrorVarianceProjector()
    {
        throw ErrorUndefined("ModelTemplate::state_error_variance& "
                             "ModelTemplate::"
                             "GetStateErrorVarianceProjector()");
    }


    /*! Returns the matrix U in the decomposition of the
      state error covariance matrix (\f$B\f$) as a product \f$LUL^T\f$.
    */
    /*!
      \return The matrix \f$U\f$.
    */
    ModelTemplate::state_error_variance_reduced&
    ModelTemplate::GetStateErrorVarianceReduced()
    {
        throw ErrorUndefined("ModelTemplate::state_error_variance_reduced& "
                             "ModelTemplate::"
                             "GetStateErrorVarianceReduced()");
    }


    //! Returns the inverse of the background error variance (\f$B^{-1}\f$).
    /*!
      \return The inverse of the background error variance (\f$B^{-1}\f$).
    */
    const ModelTemplate::state_error_variance&
    ModelTemplate::GetStateErrorVarianceInverse() const
    {
        throw ErrorUndefined("ModelTemplate::state_error_variance& "
                             "ModelTemplate::GetStateErrorVarianceInverse()");
    }


    //! Returns the square root of the model error variance.
    /*!
      \return The square root of the model error variance.
    */
    ModelTemplate::error_variance& ModelTemplate::GetErrorVarianceSqrt()
    {
        throw ErrorUndefined("ModelTemplate::GetErrorVarianceSqrt()");
    }


    //! Returns the name of the class.
    /*!
      \return The name of the class.
    */
    string ModelTemplate::GetName() const
    {
        return "ModelTemplate";
    }


    //! Receives and handles a message.
    /*
      \param[in] message the received message.
    */
    void ModelTemplate::Message(string message)
    {
        // Put here any processing you need.
    }


}

#define VERDANDI_FILE_MODEL_MODELTEMPLATE_CXX
#endif
