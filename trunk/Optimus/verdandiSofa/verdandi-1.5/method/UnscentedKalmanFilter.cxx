// Copyright (C) 2008-2010 INRIA
// Author(s): Marc Fragu, Vivien Mallet, Philippe Moireau
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


#ifndef VERDANDI_FILE_METHOD_UNSCENTEDKALMANFILTER_CXX

#include "UnscentedKalmanFilter.hxx"

#include "seldon/vector/VectorCollection.cxx"

#include "BLUE.cxx"

#include "SigmaPoint.cxx"

#include "seldon/computation/solver/SparseCholeskyFactorisation.cxx"

namespace Verdandi
{


    ////////////////////////////////
    // CONSTRUCTOR AND DESTRUCTOR //
    ////////////////////////////////


    //! Main constructor.
    /*! Builds the driver and reads option keys in the configuration file.
      \param[in] configuration_file configuration file.
    */
    template <class Model, class ObservationManager>
    UnscentedKalmanFilter<Model, ObservationManager>
    ::UnscentedKalmanFilter()
    {

        /*** Initializations ***/

        MessageHandler::AddRecipient("model", model_, Model::StaticMessage);
        MessageHandler::AddRecipient("observation_manager",
                                     observation_manager_,
                                     ObservationManager::StaticMessage);
        MessageHandler::AddRecipient("driver", *this,
                                     UnscentedKalmanFilter::StaticMessage);
    }


    //! Destructor.
    template <class Model, class ObservationManager>
    UnscentedKalmanFilter<Model, ObservationManager>
    ::~UnscentedKalmanFilter()
    {
        sigma_point_collection_.Deallocate();
    }


    /////////////
    // METHODS //
    /////////////


    //! Initializes the driver.
    /*! Initializes the model and the observation manager. Optionally computes
      the analysis of the first step. */
    template <class Model, class ObservationManager>
    void UnscentedKalmanFilter<Model, ObservationManager>
    ::Initialize(string configuration_file,
                 bool initialize_model, bool initialize_observation_manager)
    {
        VerdandiOps configuration(configuration_file);
        Initialize(configuration, initialize_model,
                   initialize_observation_manager);
    }


    //! Initializes the driver.
    /*! Initializes the model and the observation manager. Optionally computes
      the analysis of the first step. */
    template <class Model, class ObservationManager>
    void UnscentedKalmanFilter<Model, ObservationManager>
    ::Initialize(VerdandiOps& configuration,
                 bool initialize_model, bool initialize_observation_manager)
    {
        MessageHandler::Send(*this, "all", "::Initialize begin");
        configuration_file_ = configuration.GetFilePath();


        /***************************
         * Reads the configuration *
         ***************************/


        configuration.SetPrefix("unscented_kalman_filter.");

        /*** Model ***/

        configuration.Set("model.configuration_file", "",
                          configuration_file_ ,
                          model_configuration_file_);

        /*** Observation manager ***/

        configuration.Set("observation_manager.configuration_file", "",
                          configuration_file_,
                          observation_configuration_file_);

        /*** Display options ***/

        // Should iterations be displayed on screen?
        configuration.Set("display.show_iteration",
                          option_display_["show_iteration"]);
        // Should current time be displayed on screen?
        configuration.Set("display.show_time", option_display_["show_time"]);

        /*** Assimilation options ***/

        configuration.Set("data_assimilation.analyze_first_step",
                          analyze_first_step_);

        /*** Sigma-points ***/

        configuration.Set("sigma_point.type",
                          "ops_in(v, {'canonical', 'star', 'simplex'})",
                          sigma_point_type_);

        /*** Ouput saver ***/

        configuration.SetPrefix("unscented_kalman_filter.output_saver.");
        output_saver_.Initialize(configuration);
        output_saver_.Empty("forecast_time");
        output_saver_.Empty("forecast_state");
        output_saver_.Empty("analysis_time");
        output_saver_.Empty("analysis_state");

        /*** Logger and read configuration ***/

        configuration.SetPrefix("unscented_kalman_filter.");

        if (configuration.Exists("output.log"))
            Logger::SetFileName(configuration.Get<string>("output.log"));

        if (configuration.Exists("output.configuration"))
        {
            string output_configuration;
            configuration.Set("output.configuration", output_configuration);
            configuration.WriteLuaDefinition(output_configuration);
        }

        /*** Initializations ***/

        if (initialize_model)
            model_.Initialize(model_configuration_file_);
        if (initialize_observation_manager)
            observation_manager_.Initialize(model_,
                                            observation_configuration_file_);

        Nstate_ = model_.GetNstate();
        Nobservation_  = observation_manager_.GetNobservation();

        Copy(model_.GetStateErrorVariance(), background_error_variance_);

        /*** Sigma-points ***/

        if (sigma_point_type_ == "canonical")
            ComputeCanonicalSigmaPoint(Nstate_, sigma_point_collection_,
                                       alpha_i_, alpha_constant_);
        else if (sigma_point_type_ == "star")
            ComputeStarSigmaPoint(Nstate_, sigma_point_collection_, alpha_i_,
                                  alpha_constant_);
        else if (sigma_point_type_ == "simplex")
            ComputeSimplexSigmaPoint(Nstate_, sigma_point_collection_,
                                     alpha_i_, alpha_constant_);

        if (alpha_constant_)
            alpha_ = alpha_i_(0);

        Nsigma_point_ = sigma_point_collection_.GetNvector();

        /*** Assimilation ***/

        if (analyze_first_step_)
            Analyze();

        if (initialize_model)
        {
            MessageHandler::Send(*this, "model", "initial condition");
            MessageHandler::Send(*this, "driver", "initial condition");
        }

        MessageHandler::Send(*this, "all", "::Initialize end");
    }


    //! Initializes a step for the unscented Kalman filter.
    /*! Initializes a step for the model.
     */
    template <class Model, class ObservationManager>
    void UnscentedKalmanFilter<Model, ObservationManager>::InitializeStep()
    {
        MessageHandler::Send(*this, "all", "::InitializeStep begin");

        model_.InitializeStep();

        MessageHandler::Send(*this, "all", "::InitializeStep end");
    }


    //! Performs a step forward, with optimal interpolation at the end.
    template <class Model, class ObservationManager>
    void UnscentedKalmanFilter<Model, ObservationManager>::Forward()
    {
        MessageHandler::Send(*this, "all", "::Forward begin");

        // Computes of background error variance Cholesky factorization.
        model_state_error_variance background_error_variance_sqrt;
        Copy(background_error_variance_, background_error_variance_sqrt);
        GetCholesky(background_error_variance_sqrt);

        // Computes X_n^{(i)+}.
        model_state_error_variance_row x(Nstate_);
        Copy(model_.GetState(), x);

        X_i_trans_.Reallocate(Nsigma_point_, Nstate_);
        sigma_point x_col;
        for (int i = 0; i < Nsigma_point_; i++)
        {
            SetRow(x, i, X_i_trans_);
            GetRowPointer(X_i_trans_, i, x_col);
            MltAdd(Ts(1), background_error_variance_sqrt,
                   sigma_point_collection_.GetVector(i), Ts(1), x_col);
            x_col.Nullify();
        }

        if (alpha_constant_)
        {
            double new_time(0.);
            // Computes X_{n + 1}^-.
            x.Fill(Ts(0));
            x_col.Reallocate(Nstate_);
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRow(X_i_trans_, i, x_col);
                new_time = model_.ApplyOperator(x_col, false);
                Add(Ts(1), x_col, x);
                SetRow(x_col, i, X_i_trans_);
            }

            Mlt(alpha_, x);
            model_.GetState().Copy(x);
            model_.StateUpdated();
            model_.SetTime(new_time);

            // Computes P_{n + 1}^-.
            background_error_variance_.Fill(Ts(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(X_i_trans_, i, x_col);
                Add(Ts(-1), x, x_col);
                Rank1Update(Ts(1), x_col, x_col, background_error_variance_);
                x_col.Nullify();
            }
            Mlt(alpha_, background_error_variance_);
        }
        else
        {
            double new_time(0.);
            // Computes X_{n + 1}^-.
            x.Fill(Ts(0));
            x_col.Reallocate(Nstate_);
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRow(X_i_trans_, i, x_col);
                new_time =  model_.ApplyOperator(x_col, false);
                Add(alpha_i_(i), x_col, x);
                SetRow(x_col, i, X_i_trans_);
            }
            model_.GetState().Copy(x);
            model_.StateUpdated();
            model_.SetTime(new_time);

            // Computes P_{n + 1}^-.
            background_error_variance_.Fill(Ts(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(X_i_trans_, i, x_col);
                Add(Ts(-1), x, x_col);
                Rank1Update(alpha_i_(i), x_col, x_col,
                            background_error_variance_);
                x_col.Nullify();
            }
        }

        MessageHandler::Send(*this, "model", "forecast");
        MessageHandler::Send(*this, "observation_manager", "forecast");
        MessageHandler::Send(*this, "driver", "forecast");

        MessageHandler::Send(*this, "all", "::Forward end");
    }


    //! Computes an analysis.
    /*! Whenever observations are available, it computes BLUE. */
    template <class Model, class ObservationManager>
    void UnscentedKalmanFilter<Model, ObservationManager>::Analyze()
    {

        MessageHandler::Send(*this, "all", "::Analyze begin");

        observation_manager_.SetTime(model_, model_.GetTime());

        if (!observation_manager_.HasObservation())
        {
            MessageHandler::Send(*this, "all", "::Analyze end");
            return;
        }

        if (option_display_["show_time"])
            cout << "Performing UKF at time step ["
                 << model_.GetTime() << "]..." << endl;

        // Computes background error variance Cholesky factorization.
        model_state_error_variance background_error_variance_sqrt;
        Copy(background_error_variance_, background_error_variance_sqrt);
        GetCholesky(background_error_variance_sqrt);

        // Computes X_{n + 1}^{(i)-}.
        model_state& x = model_.GetState();
        X_i_trans_.Reallocate(Nsigma_point_, Nstate_);
        sigma_point x_col;
        for (int i = 0; i < Nsigma_point_; i++)
        {
            SetRow(x, i, X_i_trans_);
            GetRowPointer(X_i_trans_, i, x_col);
            MltAdd(Ts(1), background_error_variance_sqrt,
                   sigma_point_collection_.GetVector(i), Ts(1), x_col);
            x_col.Nullify();
        }

        // Computes Z_{n + 1}^(i).
        Nobservation_ = observation_manager_.GetNobservation();
        sigma_point_matrix Z_i_trans(Nsigma_point_, Nobservation_);
        observation z_col(Nobservation_);
        for (int i = 0; i < Nsigma_point_; i++)
        {
            GetRowPointer(X_i_trans_, i, x_col);
            observation_manager_.ApplyOperator(x_col, z_col);
            SetRow(z_col, i, Z_i_trans);
            x_col.Nullify();
        }

        if (alpha_constant_)
        {
            // Computes the predicted measurement Z_{n + 1}.
            observation z;
            z.Reallocate(Nobservation_);
            z.Fill(To(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(Z_i_trans, i, z_col);
                Add(To(1), z_col, z);
                z_col.Nullify();
            }
            Mlt(alpha_, z);

            // Computes X_{n+1}-.
            x.Fill(Ts(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(X_i_trans_, i, x_col);
                Add(Ts(1), x_col, x);
                x_col.Nullify();
            }
            Mlt(alpha_, x);

            // Computes P_XZ = cov(X_{n + 1}^*, Z_{n + 1}^*).
            model_state_error_variance P_xz(Nstate_, Nobservation_);
            P_xz.Fill(Ts(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(X_i_trans_, i, x_col);
                Add(Ts(-1), x, x_col);
                GetRowPointer(Z_i_trans, i, z_col);
                Add(To(-1), z, z_col);

                Rank1Update(Ts(1), x_col, z_col, P_xz);

                x_col.Nullify();
                z_col.Nullify();
            }
            Mlt(alpha_, P_xz);

            // Computes P_Z = cov(Z_{n + 1}^*, Z_{n + 1}^*).
            model_state_error_variance P_z(Nobservation_, Nobservation_);
            P_z.Fill(To(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(Z_i_trans, i, z_col);
                Rank1Update(To(1), z_col, z_col, P_z);
                z_col.Nullify();
            }
            Mlt(alpha_, P_z);
            Add(To(1), observation_manager_.GetErrorVariance(), P_z);

            // Computes the Kalman gain K_{n + 1}.
            matrix_state_observation K(Nstate_, Nobservation_);
            K.Fill(Ts(0));
            GetInverse(P_z);
            MltAdd(Ts(1), P_xz, P_z, Ts(0), K);

            // Computes X_{n + 1}^+.
            MltAdd(Ts(1), K, observation_manager_.GetInnovation(x), Ts(1), x);

            model_.StateUpdated();

            // Computes P_{n + 1}^+.
            MltAdd(Ts(-1), SeldonNoTrans, K, SeldonTrans, P_xz, Ts(1),
                   background_error_variance_);
        }
        else
        {
            // Computes the predicted measurement Z_{n + 1}.
            observation z;
            z.Reallocate(Nobservation_);
            z.Fill(To(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(Z_i_trans, i, z_col);
                Add(alpha_i_(i), z_col, z);
                z_col.Nullify();
            }

            // Computes X_{n+1}-.
            x.Fill(Ts(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(X_i_trans_, i, x_col);
                Add(alpha_i_(i), x_col, x);
                x_col.Nullify();
            }

            // Computes P_XZ = cov(X_{n + 1}^*, Z_{n + 1}^*).
            model_state_error_variance P_xz(Nstate_, Nobservation_);
            P_xz.Fill(Ts(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(X_i_trans_, i, x_col);
                Add(Ts(-1), x, x_col);
                GetRowPointer(Z_i_trans, i, z_col);
                Add(To(-1), z, z_col);

                Rank1Update(alpha_i_(i), x_col, z_col, P_xz);

                x_col.Nullify();
                z_col.Nullify();
            }

            // Computes P_Z = cov(Z_{n + 1}^*, Z_{n + 1}^*).
            model_state_error_variance P_z(Nobservation_, Nobservation_);
            P_z.Fill(To(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(Z_i_trans, i, z_col);
                Rank1Update(alpha_i_(i), z_col, z_col, P_z);
                z_col.Nullify();
            }
            Add(To(1), observation_manager_.GetErrorVariance(), P_z);

            // Computes the Kalman gain K_{n + 1}.
            matrix_state_observation K(Nstate_, Nobservation_);
            K.Fill(Ts(0));
            GetInverse(P_z);
            MltAdd(Ts(1), P_xz, P_z, Ts(0), K);

            // Computes X_{n + 1}^+.
            MltAdd(Ts(1), K, observation_manager_.GetInnovation(x), Ts(1), x);

            model_.StateUpdated();

            // Computes P_{n + 1}^+.
            MltAdd(Ts(-1), SeldonNoTrans, K, SeldonTrans, P_xz, Ts(1),
                   background_error_variance_);
        }

        if (option_display_["show_time"])
            cout << " done." << endl;

        MessageHandler::Send(*this, "model", "analysis");
        MessageHandler::Send(*this, "observation_manager", "analysis");
        MessageHandler::Send(*this, "driver", "analysis");

        MessageHandler::Send(*this, "all", "::Analyze end");
    }


    //! Finalizes a step for the model.
    template <class Model, class ObservationManager>
    void UnscentedKalmanFilter<Model, ObservationManager>::FinalizeStep()
    {
        MessageHandler::Send(*this, "all", "::FinalizeStep begin");

        model_.FinalizeStep();

        MessageHandler::Send(*this, "all", "::FinalizeStep end");
    }


    //! Finalizes the model.
    template <class Model, class ObservationManager>
    void UnscentedKalmanFilter<Model, ObservationManager>
    ::Finalize()
    {
        MessageHandler::Send(*this, "all", "::Finalize begin");

        model_.Finalize();

        MessageHandler::Send(*this, "all", "::Finalize end");
    }


    //! Checks whether the model has finished.
    /*!
      \return True if no more data assimilation is required, false otherwise.
    */
    template <class Model, class ObservationManager>
    bool UnscentedKalmanFilter<Model, ObservationManager>::HasFinished()
    {
        return model_.HasFinished();
    }


    //! Returns the model.
    /*!
      \return The model.
    */
    template <class Model, class ObservationManager>
    Model&
    UnscentedKalmanFilter<Model, ObservationManager>::GetModel()
    {
        return model_;
    }


    //! Returns the observation manager.
    /*!
      \return The observation manager.
    */
    template <class Model, class ObservationManager>
    ObservationManager&
    UnscentedKalmanFilter<Model, ObservationManager>
    ::GetObservationManager()
    {
        return observation_manager_;
    }


    //! Returns the output saver.
    /*!
      \return The output saver.
    */
    template <class Model, class ObservationManager>
    OutputSaver&
    UnscentedKalmanFilter<Model, ObservationManager>::GetOutputSaver()
    {
        return output_saver_;
    }


    //! Returns the name of the class.
    /*!
      \return The name of the class.
    */
    template <class Model, class ObservationManager>
    string
    UnscentedKalmanFilter<Model, ObservationManager>::GetName() const
    {
        return "UnscentedKalmanFilter";
    }


    //! Receives and handles a message.
    /*
      \param[in] message the received message.
    */
    template <class Model, class ObservationManager>
    void UnscentedKalmanFilter<Model, ObservationManager>
    ::Message(string message)
    {
        if (message.find("initial condition") != string::npos)
        {
            output_saver_.Save(model_.GetTime(), model_.GetTime(),
                               "forecast_time");
            output_saver_.Save(model_.GetState(), model_.GetTime(),
                               "forecast_state");
        }
        if (message.find("forecast") != string::npos)
        {
            output_saver_.Save(model_.GetTime(), model_.GetTime(),
                               "forecast_time");
            output_saver_.Save(model_.GetState(), model_.GetTime(),
                               "forecast_state");
        }
        if (message.find("analysis") != string::npos)
        {
            output_saver_.Save(model_.GetTime(), model_.GetTime(),
                               "analysis_time");
            output_saver_.Save(model_.GetState(), model_.GetTime(),
                               "analysis_state");
        }
    }


} // namespace Verdandi.


#define VERDANDI_FILE_METHOD_UNSCENTEDKALMANFILTER_CXX
#endif
