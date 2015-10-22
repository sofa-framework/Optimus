// Copyright (C) 2008-2010 INRIA
// Author(s): Marc Fragu
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


#ifndef VERDANDI_FILE_METHOD_REDUCEDORDEREXTENDEDKALMANFILTER_CXX

#include "ReducedOrderExtendedKalmanFilter.hxx"

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
    ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::ReducedOrderExtendedKalmanFilter()
    {

        /*** Initializations ***/

#if defined(VERDANDI_WITH_MPI)
        int initialized;
        MPI_Initialized(&initialized);
        if (!initialized)
            MPI_Init(NULL, NULL);
        MPI_Comm_rank(MPI_COMM_WORLD, &rank_);
        MPI_Comm_size(MPI_COMM_WORLD, &Nprocess_);
        if (rank_ == 0)
        {
#endif

            MessageHandler::AddRecipient("model", model_,
                                         Model::StaticMessage);
            MessageHandler::AddRecipient("observation_manager",
                                         observation_manager_,
                                         ObservationManager::StaticMessage);
            MessageHandler::AddRecipient("driver", *this,
                                         ReducedOrderExtendedKalmanFilter
                                         ::StaticMessage);
#if defined(VERDANDI_WITH_MPI)
        }
#endif
    }


    //! Destructor.
    template <class Model, class ObservationManager>
    ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::~ReducedOrderExtendedKalmanFilter()
    {
    }


    /////////////
    // METHODS //
    /////////////


    //! Initializes the driver.
    /*! Initializes the model and the observation manager. Optionally computes
      the analysis of the first step. */
    template <class Model, class ObservationManager>
    void ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::Initialize(string configuration_file,
                 bool initialize_model, bool initialize_observation_manager)
    {
        VerdandiOps configuration(configuration_file);
        Initialize(configuration,
                   initialize_model, initialize_observation_manager);
    }


    //! Initializes the driver.
    /*! Initializes the model and the observation manager. Optionally computes
      the analysis of the first step. */
    template <class Model, class ObservationManager>
    void ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::Initialize(VerdandiOps& configuration,
                 bool initialize_model, bool initialize_observation_manager)
    {
#if defined(VERDANDI_WITH_MPI)
        if (rank_ == 0)
            MessageHandler::Send(*this, "all", "::Initialize begin");

        /***************************
         * Reads the configuration *
         ***************************/

        configuration_file_ = configuration.GetFilePath();
        configuration.SetPrefix("reduced_order_extended_kalman_filter.");

        /*** Model ***/

        configuration.Set("model.configuration_file", "", configuration_file_,
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


        /*** Ouput saver ***/

#if defined(VERDANDI_WITH_MPI)
        if (rank_ == 0)
        {
#endif
            configuration.
                SetPrefix("reduced_order_extended_kalman_filter"
                          ".output_saver.");
            output_saver_.Initialize(configuration);
            output_saver_.Empty("forecast_time");
            output_saver_.Empty("forecast_state");
            output_saver_.Empty("analysis_time");
            output_saver_.Empty("analysis_state");

            /*** Logger and read configuration ***/

            configuration.SetPrefix("reduced_order_extended_kalman_filter.");

            if (configuration.Exists("output.configuration"))
            {
                string output_configuration;
                configuration.Set("output.configuration",
                                  output_configuration);
                configuration.WriteLuaDefinition(output_configuration);
            }
#if defined(VERDANDI_WITH_MPI)
        }
#endif
        configuration.SetPrefix("reduced_order_extended_kalman_filter.");

        if (configuration.Exists("output.log"))
            Logger::SetFileName(configuration.Get<string>("output.log"));

        /*** Initializations ***/

        if (initialize_model)
            model_.Initialize(model_configuration_file_);
        if (initialize_observation_manager)
            observation_manager_.Initialize(model_,
                                            observation_configuration_file_);
        Nstate_ = model_.GetNstate();
        Nobservation_  = observation_manager_.GetNobservation();

        model_state_error_variance& L =
            model_.GetStateErrorVarianceProjector();
        model_state_error_variance_reduced& U =
            model_.GetStateErrorVarianceReduced();
        Copy(L, L_);
        Copy(U, U_);
        Nreduced_ = U.GetN();

        if (Nprocess_ > Nreduced_)
            throw ErrorProcessing("ReducedOrderExtendedKalmanFilter"
                                  "::Initialize(bool initialize_model,"
                                  " bool initialize_observation_manager)",
                                  "The number of processes ("
                                  + to_str(Nprocess_) + ") has to be in [2, "
                                  +  to_str(Nreduced_) + "].");

        /*** Local column indexes of L and U ***/

        Nlocal_reduced_ = int(Nreduced_ / Nprocess_);
        int r = Nreduced_ % Nprocess_;
        if (Nprocess_ - rank_ - 1 < r)
            Nlocal_reduced_++;

        Nlocal_reduced_column_sum_.Reallocate(Nprocess_ + 1);
        Nlocal_reduced_column_sum_(0) = 0;
        for (int i = 0; i < Nprocess_; i++)
        {
            Nlocal_reduced_column_sum_(i + 1) = int(Nreduced_ / Nprocess_);
            if (Nprocess_ - i - 1 < r)
                Nlocal_reduced_column_sum_(i + 1) += 1;
            Nlocal_reduced_column_sum_(i + 1)
                += Nlocal_reduced_column_sum_(i);
        }

        for (int i = Nlocal_reduced_column_sum_(rank_);
             i < Nlocal_reduced_column_sum_(rank_ + 1); i++)
            local_reduced_column_.PushBack(i);

        U_.Reallocate(Nreduced_, Nlocal_reduced_);
        Vector<Ts> col(Nreduced_);
        for (int i = 0; i < Nlocal_reduced_; i++)
        {
            GetCol(U, local_reduced_column_(i), col);
            SetCol(col, i, U_);
        }

        L_.Reallocate(Nstate_, Nlocal_reduced_);
        col.Reallocate(Nstate_);
        for (int i = 0; i < Nlocal_reduced_; i++)
        {
            GetCol(L, local_reduced_column_(i), col);
            SetCol(col, i, L_);
        }

        displacement_gather_1_ = new int[Nprocess_];
        recvcount_gather_1_ = new int[Nprocess_];
        for (int i = 0; i < Nprocess_; i++)
        {
            recvcount_gather_1_[i] = (Nlocal_reduced_column_sum_(i + 1)
                                      - Nlocal_reduced_column_sum_(i))
                * Nobservation_;
            displacement_gather_1_[i] = Nlocal_reduced_column_sum_(i)
                * Nobservation_;
        }

        displacement_gather_2_ = new int[Nprocess_];
        recvcount_gather_2_ = new int[Nprocess_];
        for (int i = 0; i < Nprocess_; i++)
        {
            recvcount_gather_2_[i] = (Nlocal_reduced_column_sum_(i + 1)
                                      - Nlocal_reduced_column_sum_(i))
                * Nreduced_;
            displacement_gather_2_[i] = Nlocal_reduced_column_sum_(i)
                * Nreduced_;
        }

        displacement_gather_3_ = new int[Nprocess_];
        recvcount_gather_3_ = new int[Nprocess_];
        for (int i = 0; i < Nprocess_; i++)
        {
            recvcount_gather_3_[i] = (Nlocal_reduced_column_sum_(i + 1)
                                      - Nlocal_reduced_column_sum_(i));
            displacement_gather_3_[i] = Nlocal_reduced_column_sum_(i);
        }

        /*** Assimilation ***/

        if (analyze_first_step_)
            Analyze();

        if (rank_ == 0)
        {
            if (initialize_model)
            {
                MessageHandler::Send(*this, "model", "initial condition");
                MessageHandler::Send(*this, "driver", "initial condition");
            }

            MessageHandler::Send(*this, "all", "::Initialize end");
        }
#else
        MessageHandler::Send(*this, "all", "::Initialize begin");

        /***************************
         * Reads the configuration *
         ***************************/

        configuration_file_ = configuration.GetFilePath();
        configuration.SetPrefix("reduced_order_extended_kalman_filter.");

        /*** Model ***/

        configuration.Set("model.configuration_file", "", configuration_file_,
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

        configuration.
            SetPrefix("reduced_order_extended_kalman_filter"
                      ".output_saver.");
        output_saver_.Initialize(configuration);
        output_saver_.Empty("forecast_state");
        output_saver_.Empty("analysis_state");

        /*** Logger and read configuration ***/

        configuration.SetPrefix("reduced_order_extended_kalman_filter.");

        if (configuration.Exists("output.configuration"))
        {
            string output_configuration;
            configuration.Set("output.configuration",
                              output_configuration);
            configuration.WriteLuaDefinition(output_configuration);
        }

        configuration.SetPrefix("reduced_order_extended_kalman_filter.");

        if (configuration.Exists("output.log"))
            Logger::SetFileName(configuration.Get<string>("output.log"));

        /*** Initializations ***/

        if (initialize_model)
            model_.Initialize(model_configuration_file_);
        if (initialize_observation_manager)
            observation_manager_.Initialize(model_,
                                            observation_configuration_file_);
        Nstate_ = model_.GetNstate();
        Nobservation_  = observation_manager_.GetNobservation();

        Copy(model_.GetStateErrorVarianceProjector(), L_);
        Copy(model_.GetStateErrorVarianceReduced(), U_);
        Nreduced_ = U_.GetN();

        /*** Assimilation ***/

        if (analyze_first_step_)
            Analyze();

        if (initialize_model)
        {
            MessageHandler::Send(*this, "model", "initial condition");
            MessageHandler::Send(*this, "driver", "initial condition");
        }

        MessageHandler::Send(*this, "all", "::Initialize end");

#endif
    }


    //! Initializes a step for the extended Kalman filter.
    /*! Initializes a step for the model.
     */
    template <class Model, class ObservationManager>
    void ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::InitializeStep()
    {
        model_.InitializeStep();
    }


    //! Performs a step forward, with optimal interpolation at the end.
    template <class Model, class ObservationManager>
    void ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::Forward()
    {
#if defined(VERDANDI_WITH_MPI)
        if (rank_ == 0)
#endif
            MessageHandler::Send(*this, "all", "::Forward begin");

        time_ = model_.GetTime();

        model_.Forward();

        PropagateCovarianceMatrix();

#if defined(VERDANDI_WITH_MPI)
        if (rank_ == 0)
        {
#endif
            MessageHandler::Send(*this, "model", "forecast");
            MessageHandler::Send(*this, "observation_manager", "forecast");
            MessageHandler::Send(*this, "driver", "forecast");

            MessageHandler::Send(*this, "all", "::Forward end");
#if defined(VERDANDI_WITH_MPI)
        }
#endif
    }


    //! Computes an analysis.
    /*! Whenever observations are available, it computes BLUE. */
    template <class Model, class ObservationManager>
    void ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::Analyze()
    {
#if defined(VERDANDI_WITH_MPI)
        if (rank_ == 0)
            MessageHandler::Send(*this, "all", "::Analyze begin");

        observation_manager_.SetTime(model_, model_.GetTime());

        if (observation_manager_.HasObservation())
        {
            if (rank_ == 0)
                if (option_display_["show_time"])
                    cout << "Performing Reduced Order EKF at time step ["
                         << model_.GetTime() << "]..." << endl;

            model_state& x = model_.GetState();
            Nstate_ = model_.GetNstate();

            /*** Updated matrix U ***/

            dense_matrix HL_local_trans(Nobservation_, Nlocal_reduced_),
                working_matrix_or(Nobservation_, Nlocal_reduced_),
                HL_global_trans(Nreduced_, Nobservation_),
                U_global(Nreduced_, Nreduced_);


            Mlt(observation_manager_.GetTangentLinearOperator(), L_,
                HL_local_trans);
            Transpose(HL_local_trans);
            MPI_Allgatherv(HL_local_trans.GetData(), Nlocal_reduced_ *
                           Nobservation_, MPI_DOUBLE,
                           HL_global_trans.GetData(), recvcount_gather_1_,
                           displacement_gather_1_, MPI_DOUBLE,
                           MPI_COMM_WORLD);
            Transpose(HL_local_trans);
            Mlt(observation_manager_.GetErrorVarianceInverse(),
                HL_local_trans, working_matrix_or);
            MltAdd(Ts(1), HL_global_trans, working_matrix_or, Ts(1), U_);

            Transpose(U_);
            MPI_Allgatherv(U_.GetData(), Nlocal_reduced_ * Nreduced_,
                           MPI_DOUBLE, U_global.GetData(),
                           recvcount_gather_2_, displacement_gather_2_,
                           MPI_DOUBLE, MPI_COMM_WORLD);
            Transpose(U_);

            observation & y = observation_manager_.GetInnovation(x);
            Nobservation_ = y.GetSize();

            model_state_error_variance_row state_innovation(Nreduced_),
                state_innovation_local(Nlocal_reduced_),
                x_local(Nreduced_);
            MltAdd(Ts(1), SeldonTrans, working_matrix_or, y,
                   Ts(0), state_innovation_local);

            MPI_Allgatherv(state_innovation_local.GetData(), Nlocal_reduced_,
                           MPI_DOUBLE, state_innovation.GetData(),
                           recvcount_gather_3_, displacement_gather_3_,
                           MPI_DOUBLE, MPI_COMM_WORLD);

            GetInverse(U_global);
            MltAdd(Ts(1), U_global, state_innovation, Ts(0), x_local);

            model_state_error_variance_row x_local_local(Nlocal_reduced_);
            for (int i = 0; i < Nlocal_reduced_; i++)
                x_local_local(i) =
                    x_local(Nlocal_reduced_column_sum_(rank_) + i);

            model_state_error_variance_row dx_local(Nstate_), dx(Nstate_);
            MltAdd(Ts(1), L_, x_local_local, Ts(0), dx_local);
            dx.Fill(Ts(0));
            MPI_Allreduce(dx_local.GetData(), dx.GetData(), Nstate_,
                          MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD);
            Add(Ts(1), dx, x);

            model_.StateUpdated();


            if (rank_ == 0)
            {
                if (option_display_["show_time"])
                    cout << " done." << endl;

                MessageHandler::Send(*this, "model", "analysis");
                MessageHandler::Send(*this, "observation_manager",
                                     "analysis");
                MessageHandler::Send(*this, "driver", "analysis");
            }
        }
        if (rank_ == 0)
            MessageHandler::Send(*this, "all", "::Analyze end");
#else
        MessageHandler::Send(*this, "all", "::Analyze begin");

        observation_manager_.SetTime(model_, model_.GetTime());

        if (observation_manager_.HasObservation())
        {
            if (option_display_["show_time"])
                cout << "Performing Reduced Order EKF at time step ["
                     << model_.GetTime() << "]..." << endl;

            model_state& x = model_.GetState();
            Nstate_ = model_.GetNstate();

            observation& y = observation_manager_.GetInnovation(x);
            Nobservation_ = y.GetSize();

            /*** Updated matrix U ***/

            dense_matrix HL(Nobservation_, Nreduced_),
                working_matrix_or(Nobservation_, Nreduced_);
            Mlt(observation_manager_.GetTangentLinearOperator(), L_, HL);
            Mlt(observation_manager_.GetErrorVarianceInverse(), HL,
                working_matrix_or);
            MltAdd(Ts(1), SeldonTrans, HL, SeldonNoTrans,
                   working_matrix_or, Ts(1), U_);

            /*** Updated K ***/

            dense_matrix U_inv(U_),
                working_matrix_ro2(Nreduced_, Nobservation_);

            Mlt(observation_manager_.GetErrorVarianceInverse(), HL,
                working_matrix_or);
            model_state_error_variance_row state_innovation(Nreduced_);
            MltAdd(Ts(1), SeldonTrans, working_matrix_or, y, Ts(0),
                   state_innovation);

            model_state_error_variance_row correction(Nreduced_);
            GetInverse(U_inv);
            MltAdd(Ts(1), U_inv, state_innovation, Ts(0), correction);
            MltAdd(Ts(1), L_, correction, Ts(1), x);

            model_.StateUpdated();

            if (option_display_["show_time"])
                cout << " done." << endl;

            MessageHandler::Send(*this, "model", "analysis");
            MessageHandler::Send(*this, "observation_manager", "analysis");
            MessageHandler::Send(*this, "driver", "analysis");
        }

        MessageHandler::Send(*this, "all", "::Analyze end");
#endif
    }


    //! Finalizes a step for the model.
    template <class Model, class ObservationManager>
    void ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::FinalizeStep()
    {
#if defined(VERDANDI_WITH_MPI)
        if (rank_ == 0)
#endif
            MessageHandler::Send(*this, "all", "::FinalizeStep begin");

        model_.FinalizeStep();

#if defined(VERDANDI_WITH_MPI)
        if (rank_ == 0)
#endif
            MessageHandler::Send(*this, "all", "::FinalizeStep end");
    }


    //! Finalizes the model.
    template <class Model, class ObservationManager>
    void ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::Finalize()
    {
#if defined(VERDANDI_WITH_MPI)
        if (rank_ == 0)
#endif
            MessageHandler::Send(*this, "all", "::Finalize begin");

        model_.Finalize();

#if defined(VERDANDI_WITH_MPI)
        if (rank_ == 0)
#endif
            MessageHandler::Send(*this, "all", "::Finalize end");
#if defined(VERDANDI_WITH_MPI)
        int finalized;
        MPI_Finalized(&finalized);
        if (!finalized)
            MPI_Finalize();
#endif
    }


    //! Computes Covariance.
    template <class Model, class ObservationManager>
    void ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::PropagateCovarianceMatrix()
    {
#ifdef VERDANDI_WITH_MPI
        double saved_time = model_.GetTime();
        model_.SetTime(time_);

        // One column of L.
        model_state_error_variance_row L_col(Nstate_);
        for (int j = 0; j < Nlocal_reduced_; j++)
        {
            GetCol(L_, j, L_col);
            model_.ApplyTangentLinearOperator(L_col);
            SetCol(L_col, j, L_);
        }

        model_.SetTime(saved_time);
#else
        double saved_time = model_.GetTime();
        model_.SetTime(time_);

        // One column of L.
        model_state_error_variance_row L_col(Nstate_);
        for (int j = 0; j < Nreduced_; j++)
        {
            GetCol(L_, j, L_col);
            model_.ApplyTangentLinearOperator(L_col);
            SetCol(L_col, j, L_);
        }

        model_.SetTime(saved_time);
#endif
    }



    //! Checks whether the model has finished.
    /*!
      \return True if no more data assimilation is required, false otherwise.
    */
    template <class Model, class ObservationManager>
    bool ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::HasFinished()
    {
        return model_.HasFinished();
    }


    //! Returns the model.
    /*!
      \return The model.
    */
    template <class Model, class ObservationManager>
    Model&
    ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::GetModel()
    {
        return model_;
    }


    //! Returns the observation manager.
    /*!
      \return The observation manager..
    */
    template <class Model, class ObservationManager>
    ObservationManager&
    ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
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
    ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::GetOutputSaver()
    {
        return output_saver_;
    }


    //! Returns the name of the class.
    /*!
      \return The name of the class.
    */
    template <class Model, class ObservationManager>
    string
    ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::GetName() const
    {
        return "ReducedOrderExtendedKalmanFilter";
    }


    //! Receives and handles a message.
    /*
      \param[in] message the received message.
    */
    template <class Model, class ObservationManager>
    void ReducedOrderExtendedKalmanFilter<Model, ObservationManager>
    ::Message(string message)
    {
#if defined(VERDANDI_WITH_MPI)
        if (rank_ == 0)
        {
#endif
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
#if defined(VERDANDI_WITH_MPI)
        }
#endif
    }


} // namespace Verdandi.


#define VERDANDI_FILE_METHOD_REDUCEDORDEREXTENDEDKALMANFILTER_CXX
#endif
