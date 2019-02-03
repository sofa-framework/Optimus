// Copyright (C) 2009-2010 INRIA
// Author(s): Vivien Mallet
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


#ifndef VERDANDI_FILE_MODEL_QUADRATICMODEL_CXX


#include "QuadraticModel.hxx"


namespace Verdandi
{


    /////////////////////////////////
    // CONSTRUCTORS AND DESTRUCTOR //
    /////////////////////////////////


    //! Constructor.
    template <class T>
    QuadraticModel<T>::QuadraticModel():
        Delta_t_(1.), time_(0.), current_row_(-1)
    {
    }


    //! Constructor.
    /*! It reads the initial condition and the time settings.
      \param[in] configuration_file path to the configuration file.
    */
    template <class T>
    QuadraticModel<T>::QuadraticModel(string configuration_file):
        Delta_t_(1.), time_(0.), current_row_(-1)
    {
        Initialize(configuration_file);
    }


    //! Destructor.
    template <class T>
    QuadraticModel<T>::~QuadraticModel()
    {
        int start_nullify = 0;
        if (with_constant_term_)
            start_nullify = 1;
        for (int i = start_nullify; i < Nparameter_; i++)
        {
            parameter_[i]->Nullify();
            delete(parameter_[i]);
        }
    }


    /////////////////////
    // INITIALIZATIONS //
    /////////////////////


    //! Initializes the model.
    /*! It reads the initial condition and the time settings.
      \param[in] configuration_file configuration file.
    */
    template <class T>
    void QuadraticModel<T>::Initialize(string configuration_file)
    {

        /*** Configuration ***/

        VerdandiOps configuration(configuration_file);

        configuration.SetPrefix("quadratic_model.definition.");

        configuration.Set("initial_state", state_);
        Nstate_ = state_.GetLength();

        configuration.Set("with_quadratic_term", with_quadratic_term_);
        configuration.Set("with_linear_term", with_linear_term_);
        configuration.Set("with_constant_term", with_constant_term_);

        if (with_quadratic_term_)
        {
            S_state_.Reallocate(Nstate_);
            S_.resize(Nstate_);
            for (int i = 0; i < Nstate_; i++)
                S_[i].Reallocate(Nstate_, Nstate_);
            configuration.Set("quadratic_term", S_);
            if (int(S_.size()) != Nstate_)
                throw ErrorConfiguration("QuadraticModel::QuadraticModel",
                                         "The initial state has "
                                         + to_str(Nstate_) + " elements, but "
                                         "the entry \"quadratic_term\" has "
                                         + to_str(int(S_.size()))
                                         + " elements.");
            for (int i = 0; i < Nstate_; i++)
                if (S_[i].GetM() != Nstate_ || S_[i].GetN() != Nstate_)
                    throw ErrorConfiguration("QuadraticModel::QuadraticModel",
                                             "The initial state has "
                                             + to_str(Nstate_) + " elements, "
                                             "but the matrix " + to_str(i)
                                             + " of \"quadratic_term\" has "
                                             + to_str(int(S_[i].GetM()))
                                             + " rows and "
                                             + to_str(int(S_[i].GetN()))
                                             + " columns.");
        }

        if (with_linear_term_)
        {
            L_.Reallocate(Nstate_, Nstate_);
            configuration.Set("linear_term", L_);
            if (L_.GetM() != Nstate_ || L_.GetN() != Nstate_)
                throw ErrorConfiguration("QuadraticModel::QuadraticModel",
                                         "The initial state has "
                                         + to_str(Nstate_) + " elements, but "
                                         "the entry \"linear_term\" is a "
                                         + to_str(int(L_.GetM())) + " x "
                                         + to_str(int(L_.GetN()))
                                         + " matrix.");
        }

        if (with_constant_term_)
        {
            configuration.Set("constant", b_);
            if (b_.GetLength() != Nstate_)
                throw ErrorConfiguration("QuadraticModel::QuadraticModel",
                                         "The initial state has "
                                         + to_str(Nstate_) + " elements, but "
                                         "the entry \"constant\" has "
                                         + to_str(b_.GetLength())
                                         + " elements.");
        }

        configuration.Set("Delta_t", Delta_t_);
        configuration.Set("initial_time", time_);
        configuration.Set("final_time", final_time_);

        /*** Distribution for the constant term ***/

        configuration.ClearPrefix();

        if (configuration.Exists("quadratic_model.uncertainty"))
        {
            configuration.SetPrefix("quadratic_model.uncertainty.");

            configuration.Set("uncertain_parameter_list",
                              "ops_in(v, {'quadratic_term', 'linear_term', "
                              "'constant'})",
                              uncertain_parameter_vector_);

            // We perturb a term only if it is in the uncertain parameter list
            // and if it is applied in the model.
            is_quadratic_perturbed_ =
                (find (uncertain_parameter_vector_.begin(),
                       uncertain_parameter_vector_.end(),  "quadratic_term")
                 != uncertain_parameter_vector_.end()) &&
                with_quadratic_term_;

            is_linear_perturbed_ =
                (find (uncertain_parameter_vector_.begin(),
                       uncertain_parameter_vector_.end(),  "linear_term")
                 != uncertain_parameter_vector_.end()) &&
                with_linear_term_;

            is_constant_perturbed_ =
                (find (uncertain_parameter_vector_.begin(),
                       uncertain_parameter_vector_.end(),  "constant")
                 != uncertain_parameter_vector_.end()) &&
                with_constant_term_;

            if (is_constant_perturbed_)
            {
                Vector<T> constant_mean;
                constant_mean.Reallocate(b_.GetLength());
                configuration.Set("constant.mean", constant_mean);
                if (constant_mean.GetLength() != Nstate_)
                    throw ErrorConfiguration("QuadraticModel::QuadraticModel",
                                             "The initial state has "
                                             + to_str(Nstate_) + " elements, "
                                             "but the entry \"uncertainty."
                                             "constant.mean\" has "
                                             + to_str(int(constant_mean
                                                          .GetLength()))
                                             + " elements.");
                Add(T(1), constant_mean, b_);

                Matrix<T, Symmetric, RowSymPacked> constant_variance;
                constant_variance.Reallocate(Nstate_, Nstate_);
                configuration.Set("constant.variance",
                                  constant_variance);
                if (constant_variance.GetM() != Nstate_
                    || constant_variance.GetN() != Nstate_)
                    throw ErrorConfiguration("QuadraticModel::QuadraticModel",
                                             "The initial state has "
                                             + to_str(Nstate_) + " elements, "
                                             "but the entry \"uncertainty."
                                             "constant.variance\" has "
                                             "dimensions "
                                             + to_str(int(constant_variance
                                                          .GetM()))
                                             + " x "
                                             + to_str(int(constant_variance
                                                          .GetN())) + ".");
                variance_.push_back(constant_variance);

                string constant_pdf;
                configuration.Set("constant.distribution",
                                  "ops_in(v, {'Normal', 'LogNormal', "
                                  "'NormalHomogeneous',"
                                  "'LogNormalHomogeneous'})",
                                  constant_pdf);
                pdf_.push_back(constant_pdf);

                Vector<T> constant_parameter;
                configuration.Set("constant.parameter",
                                  constant_parameter);
                optional_parameters_.push_back(constant_parameter);

                parameter_.push_back(&b_);
            }

            if (is_linear_perturbed_)
            {
                Vector<T> linear_mean;
                linear_mean.Reallocate(Nstate_);
                configuration.Set("linear_term.mean", linear_mean);
                if (linear_mean.GetLength() != Nstate_)
                    throw ErrorConfiguration("QuadraticModel::QuadraticModel",
                                             "The initial state has "
                                             + to_str(Nstate_) + " elements, "
                                             "but the entry \"uncertainty."
                                             "linear.mean\" has "
                                             + to_str(int(linear_mean
                                                          .GetLength()))
                                             + " elements.");

                Matrix<T, Symmetric, RowSymPacked> linear_variance;
                linear_variance.Reallocate(Nstate_, Nstate_);
                configuration.Set("linear_term.variance",
                                  linear_variance);
                if (linear_variance.GetM() != Nstate_
                    || linear_variance.GetN() != Nstate_)
                    throw ErrorConfiguration("QuadraticModel::QuadraticModel",
                                             "The initial state has "
                                             + to_str(Nstate_) + " elements, "
                                             "but the entry \"uncertainty."
                                             "linear.variance\" has "
                                             "dimensions "
                                             + to_str(int(linear_variance
                                                          .GetM()))
                                             + " x "
                                             + to_str(int(linear_variance
                                                          .GetN())) + ".");

                string linear_pdf;
                configuration.Set("linear_term.distribution",
                                  "ops_in(v, {'Normal', 'LogNormal', "
                                  "'NormalHomogeneous', "
                                  "'LogNormalHomogeneous'})",
                                  linear_pdf);

                Vector<T> linear_parameter;
                configuration.Set("linear_term.parameter",
                                  linear_parameter);

                for (int i = 0; i < Nstate_; i++)
                {
                    Vector<T> row;
                    GetRow(L_, i, row);
                    Add(T(1), linear_mean, row);
                    SetRow(row, i, L_);
                }

                for (int i = 0; i < Nstate_; i++)
                {
                    Vector<T> *row = new Vector<T>;
                    row->SetData(Nstate_, L_.GetMe()[i]);
                    parameter_.push_back(row);
                    variance_.push_back(linear_variance);
                    pdf_.push_back(linear_pdf);
                    optional_parameters_.push_back(linear_parameter);
                }
            }

            if (is_quadratic_perturbed_)
            {
                Vector<T> quadratic_mean;
                quadratic_mean.Reallocate(Nstate_);
                configuration.Set("quadratic_term.mean", quadratic_mean);
                if (quadratic_mean.GetLength() != Nstate_)
                    throw ErrorConfiguration("QuadraticModel::QuadraticModel",
                                             "The initial state has "
                                             + to_str(Nstate_) + " elements, "
                                             "but the entry \"uncertainty."
                                             "quadratic.mean\" has "
                                             + to_str(int(quadratic_mean
                                                          .GetLength()))
                                             + " elements.");

                Matrix<T, Symmetric, RowSymPacked> quadratic_variance;
                quadratic_variance.Reallocate(Nstate_, Nstate_);
                configuration.Set("quadratic_term.variance",
                                  quadratic_variance);
                if (quadratic_variance.GetM() != Nstate_
                    || quadratic_variance.GetN() != Nstate_)
                    throw ErrorConfiguration("QuadraticModel::QuadraticModel",
                                             "The initial state has "
                                             + to_str(Nstate_) + " elements, "
                                             "but the entry \"uncertainty."
                                             "quadratic.variance\" has "
                                             "dimensions "
                                             + to_str(int(quadratic_variance
                                                          .GetM()))
                                             + " x "
                                             + to_str(int(quadratic_variance
                                                          .GetN())) + ".");

                string quadratic_pdf;
                configuration.Set("quadratic_term.distribution",
                                  "ops_in(v, {'Normal', 'LogNormal', "
                                  "'NormalHomogeneous',"
                                  "'LogNormalHomogeneous'})",
                                  quadratic_pdf);

                Vector<T> quadratic_parameter;
                configuration.Set("quadratic_term.parameter",
                                  quadratic_parameter);

                Vector<T> row;
                for (int i = 0; i < Nstate_; i++)
                    for (int j = 0; j < Nstate_; j++)
                    {
                        GetRow(S_[i], j, row);
                        Add(T(1), quadratic_mean, row);
                        SetRow(row, j, S_[i]);
                    }

                for (int i = 0; i < Nstate_; i++)
                    for (int j = 0; j < Nstate_; j++)
                    {
                        Vector<T> *row = new Vector<T>;
                        row->SetData(Nstate_, S_[i].GetMe()[j]);
                        parameter_.push_back(row);
                        variance_.push_back(quadratic_variance);
                        pdf_.push_back(quadratic_pdf);
                        optional_parameters_.push_back(quadratic_parameter);
                    }
            }
        }

        // No correlation.
        Vector<T> empty_vector;
        correlation_.push_back(empty_vector);

        Nparameter_ = parameter_.size();

        /*** Errors ***/

        configuration.SetPrefix("quadratic_model.");

        if (configuration.Exists("error"))
        {
            Q_.Reallocate(Nstate_, Nstate_);
            if (configuration.Get<bool>("error.scaled_identity"))
            {
                Q_.SetIdentity();
                Mlt(configuration.Get<T>("error.diagonal_value", "v >= 0"),
                    Q_);
            }
            else
                configuration.Set("error.value", Q_);
        }

        if (configuration.Exists("error_sqrt"))
        {
            Q_sqrt_.Reallocate(Nstate_, 0);
            configuration.Set("error_sqrt.value", Q_sqrt_);
        }

        if (configuration.Exists("state_error"))
        {
            P_.Reallocate(Nstate_, Nstate_);
            if (configuration.Get<bool>("state_error.scaled_identity"))
            {
                P_.SetIdentity();
                Mlt(configuration.Get<T>("state_error.diagonal_value",
                                         "v >= 0"),
                    P_);
            }
            else
                configuration.Set("state_error.value", P_);
        }

        if (configuration.Exists("state_error_sqrt"))
        {
            P_sqrt_.Reallocate(Nstate_, 0);
            configuration.Set("state_error_sqrt.value", P_sqrt_);
        }

        /*** Output saver ***/

        configuration.SetPrefix("quadratic_model.output_saver.");
        output_saver_.Initialize(configuration);
        if (with_quadratic_term_)
        {
            output_saver_.Empty("S");
            for (int i = 0; i < Nstate_; i++)
                output_saver_.Save(S_[i], "S");
        }
        if (with_linear_term_)
        {
            output_saver_.Empty("L");
            output_saver_.Save(L_, "L");
        }
        if (with_constant_term_)
        {
            output_saver_.Empty("b");
            output_saver_.Save(b_, "b");
        }
        output_saver_.Empty("state");
    }


    //! Initializes the current time step for the model.
    template <class T>
    void QuadraticModel<T>::InitializeStep()
    {
    }


    ////////////////
    // PROCESSING //
    ////////////////


    //! Advances one step forward in time.
    template <class T>
    void QuadraticModel<T>::Forward()
    {
        if (with_quadratic_term_)
        {
            state current_state = state_;
            for (int i = 0; i < Nstate_; i++)
            {
                MltAdd(Delta_t_, S_[i], current_state, T(0), S_state_);
                state_(i) += DotProd(S_state_, current_state);
            }
            if (with_linear_term_)
                MltAdd(Delta_t_, L_, current_state, T(1), state_);
        }
        else if (with_linear_term_)
            MltAdd(Delta_t_, L_, state_, T(1), state_);
        if (with_constant_term_)
            Add(Delta_t_, b_, state_);

        time_ += Delta_t_;
    }


    //! Applies the model to a given state vector.
    /*!
      \param[in,out] x on entry, the state vector to which the model is
      applied; on exit, the state vector after the model is applied.
      \param[in] forward Boolean to indicate if the model has to go on to the
      next step.
      \param[in] preserve_state Boolean to indicate if the model state has to
      be preserved.
      \return The time associated with \a x on exit plus one time step.
      \warning The time of the model has to be preserved.
    */
    template <class T>
    double QuadraticModel<T>
    ::ApplyOperator(state& x, bool preserve_state)
    {
        state saved_state;
        if (preserve_state)
            saved_state.SetData(state_);

        state_.Nullify();
        state_.SetData(x);
        Forward();
        time_ -= Delta_t_;
        state_.Nullify();

        if (preserve_state)
        {
            state_.SetData(saved_state);
            saved_state.Nullify();
        }

        return time_ + Delta_t_;
    }


    //! Applies the tangent linear model to a given vector.
    /*!
      \param[in,out] x on entry, a vector to which the tangent linear model
      should be applied; on exit, the result.
      \return The time associated with \a x on exit. This time should be the
      model time plus one time step.
    */
    template <class T>
    double QuadraticModel<T>
    ::ApplyTangentLinearOperator(state& x)
    {
        state input = x;
        if (with_quadratic_term_)
        {
            for (int i = 0; i < Nstate_; i++)
            {
                MltAdd(Delta_t_, S_[i], state_, T(0), S_state_);
                MltAdd(Delta_t_, SeldonTrans, S_[i], state_, T(1), S_state_);
                x(i) += DotProd(S_state_, input);
            }
            if (with_linear_term_)
                MltAdd(Delta_t_, L_, input, T(1), x);
        }
        else if (with_linear_term_)
            MltAdd(Delta_t_, L_, input, T(1), x);

        return time_ + Delta_t_;
    }


    //! Returns the tangent linear model.
    /*!
      \return The matrix of the tangent linear model.
    */
    template <class T>
    typename QuadraticModel<T>::tangent_linear_operator& QuadraticModel<T>
    ::GetTangentLinearOperator()
    {
        tangent_linear_operator_.Reallocate(Nstate_, Nstate_);
        if (with_quadratic_term_)
        {
            Vector<T> M_row(Nstate_);
            for (int i = 0; i < Nstate_; i++)
            {
                MltAdd(T(1), S_[i], state_, T(0), M_row);
                MltAdd(T(Delta_t_), SeldonTrans, S_[i], state_,
                       T(Delta_t_), M_row);
                SetRow(M_row, i, tangent_linear_operator_);
                tangent_linear_operator_(i, i) += T(1);
            }
            if (with_linear_term_)
                Add(T(Delta_t_), L_, tangent_linear_operator_);
        }
        else if (with_linear_term_)
        {
            tangent_linear_operator_.Copy(L_);
            Mlt(Delta_t_, tangent_linear_operator_);
            for (int i = 0; i < Nstate_; i++)
                tangent_linear_operator_(i, i) += T(1);
        }
        else
            tangent_linear_operator_.SetIdentity();

        return tangent_linear_operator_;
    }


    //! Checks whether the model has finished.
    /*!
      \return True if no more data assimilation is required, false otherwise.
    */
    template <class T>
    bool QuadraticModel<T>::HasFinished() const
    {
        return time_ >= final_time_;
    }


    //! Saves the simulated data.
    /*! It saves the state.
     */
    template <class T>
    void QuadraticModel<T>::Save()
    {
        output_saver_.Save(state_, time_, "state");
    }


    //! Finalizes the current time step for the model.
    template <class T>
    void QuadraticModel<T>::FinalizeStep()
    {
    }


    //! Finalizes the model.
    template <class T>
    void QuadraticModel<T>::Finalize()
    {
    }


    ///////////////////
    // ACCESS METHOD //
    ///////////////////


    //! Returns the time step.
    /*!
      \return The time step.
    */
    template <class T>
    T QuadraticModel<T>::GetDelta_t() const
    {
        return Delta_t_;
    }


    //! Returns the current time.
    /*!
      \return The current time.
    */
    template <class T>
    double QuadraticModel<T>::GetTime() const
    {
        return time_;
    }


    //! Sets the current time.
    /*!
      \param[in] time the current time.
    */
    template <class T>
    void QuadraticModel<T>::SetTime(double time)
    {
        time_ = time;
    }


    //! Returns the dimension of the state.
    /*!
      \return The dimension of the state.
    */
    template <class T>
    int QuadraticModel<T>::GetNstate() const
    {
        return Nstate_;
    }


    //! Returns the dimension of the full state.
    /*!
      \return The dimension of the full state.
    */
    template <class T>
    int QuadraticModel<T>::GetNfull_state() const
    {
        return GetNstate();
    }


    //! Provides the controlled state vector.
    /*!
      \return state the controlled state vector.
    */
    template <class T>
    typename QuadraticModel<T>::state& QuadraticModel<T>
    ::GetState()
    {
        return state_;
    }


    //! Performs some calculations when the update of the model state is done.
    template <class T>
    void QuadraticModel<T>::StateUpdated()
    {
    }


    //! Provides the full state vector.
    /*!
      \return state the full state vector.
    */
    template <class T>
    typename QuadraticModel<T>::state& QuadraticModel<T>
    ::GetFullState()
    {
        return state_;
    }


    /*! \brief Performs some calculations when the update of the full model
      state is done.*/
    template <class T>
    void QuadraticModel<T>::FullStateUpdated()
    {
    }


    //! Returns the number of parameters to be perturbed.
    /*!
      \return The number of parameters to be perturbed.
    */
    template <class T>
    int QuadraticModel<T>::GetNparameter()
    {
        return Nparameter_;
    }


    //! Gets the i-th uncertain parameter.
    /*!
      \param[in] i index of the parameter.
      \return The vector associated with the i-th parameter.
    */
    template<class T>
    typename QuadraticModel<T>::uncertain_parameter&
    QuadraticModel<T>::GetParameter(int i)
    {
        return *(parameter_[i]);
    }


    /*! Performs some calculations when the update of the i-th uncertain
      parameter is done.
    */
    /*!
      \param[in] i index of the parameter.
    */
    template<class T>
    void QuadraticModel<T>::ParameterUpdated(int i)
    {
    }


    //! Returns the correlation between the uncertain parameters.
    /*! Since there is only one parameter, an empty vector is
      returned.
      \param[in] i parameter index.
      \return An empty vector.
    */
    template<class T>
    Vector<T>& QuadraticModel<T>::GetParameterCorrelation(int i)
    {
        return correlation_[0];
    }


    //! Returns the PDF of the i-th parameter.
    /*!
      \param[in] i uncertain-variable index.
      \return The PDF of the i-th parameter.
    */
    template<class T>
    string QuadraticModel<T>::GetParameterPDF(int i)
    {
        return pdf_[i];
    }


    /*! \brief Returns the covariance matrix associated with the i-th
      parameter. */
    /*!
      \param[in] i parameter index.
      \return The covariance matrix associated with the i-th parameter.
    */
    template<class T>
    typename QuadraticModel<T>::parameter_variance&
    QuadraticModel<T>::GetParameterVariance(int i)
    {
        return variance_[i];
    }


    //! Returns parameters associated with the PDF of some model parameter.
    /*! In case of normal or log-normal distribution, the parameters are
      clipping parameters.
      \param[in] i model parameter index.
      \return The parameters associated with the i-th parameter.
    */
    template<class T>
    Vector<T>& QuadraticModel<T>::GetParameterPDFData(int i)
    {
        return optional_parameters_[i];
    }


    //! Returns the perturbation option of the i-th parameter.
    /*!
      \param[in] i parameter index.
      \return The perturbation option of the i-th parameter.
    */
    template<class T>
    string QuadraticModel<T>::GetParameterOption(int i)
    {
        return "init_step";
    }


    ////////////
    // ERRORS //
    ////////////


    //! Returns the model error variance.
    /*!
      \return The model error variance.
    */
    template <class T>
    typename QuadraticModel<T>::error_variance&
    QuadraticModel<T>::GetErrorVariance()
    {
        return Q_;
    }


    //! Returns the model error variance.
    /*!
      \return The model error variance.
    */
    template <class T>
    const typename QuadraticModel<T>::error_variance&
    QuadraticModel<T>::GetErrorVariance() const
    {
        return Q_;
    }


    //! Returns the square root of the model error variance.
    /*!
      \return The square root of the model error variance.
    */
    template <class T>
    typename QuadraticModel<T>::error_variance&
    QuadraticModel<T>::GetErrorVarianceSqrt()
    {
        return Q_sqrt_;
    }


    //! Returns the square root of the model error variance.
    /*!
      \return The square root of the model error variance.
    */
    template <class T>
    const typename QuadraticModel<T>::error_variance&
    QuadraticModel<T>::GetErrorVarianceSqrt() const
    {
        return Q_sqrt_;
    }


    //! Returns the state error variance.
    /*!
      \return The state error variance.
    */
    template <class T>
    typename QuadraticModel<T>::state_error_variance&
    QuadraticModel<T>::GetStateErrorVariance()
    {
        return P_;
    }


    //! Returns the state error variance.
    /*!
      \return The state error variance.
    */
    template <class T>
    const typename QuadraticModel<T>::state_error_variance&
    QuadraticModel<T>::GetStateErrorVariance() const
    {
        return P_;
    }


    //! Returns a row of the state error variance.
    /*!
      \param[in] row row index.
      \return The row with index \a row in the state error variance.
    */
    template <class T>
    typename QuadraticModel<T>::state_error_variance_row&
    QuadraticModel<T>::GetStateErrorVarianceRow(int row)
    {
        if (current_row_ == row)
            return state_error_variance_row_;
        else
            current_row_ = row;
            GetRow(P_, row, state_error_variance_row_);
        return state_error_variance_row_;
    }


    //! Returns the square root of the state error variance.
    /*!
      \return The square root of the state error variance.
    */
    template <class T>
    typename QuadraticModel<T>::state_error_variance&
    QuadraticModel<T>::GetStateErrorVarianceSqrt()
    {
        return P_sqrt_;
    }


    //! Returns the square root of the state error variance.
    /*!
      \return The square root of the state error variance.
    */
    template <class T>
    const typename QuadraticModel<T>::state_error_variance&
    QuadraticModel<T>::GetStateErrorVarianceSqrt() const
    {
        return P_sqrt_;
    }


    //! Returns the name of the class.
    /*!
      \return The name of the class.
    */
    template <class T>
    string QuadraticModel<T>::GetName() const
    {
        return "QuadraticModel";
    }


    //! Receives and handles a message.
    /*
      \param[in] message the received message.
    */
    template <class T>
    void QuadraticModel<T>::Message(string message)
    {
        if (message.find("initial condition") != string::npos
            || message.find("forecast") != string::npos)
            Save();
    }


} // namespace Verdandi.


#define VERDANDI_FILE_MODEL_QUADRATICMODEL_CXX
#endif
