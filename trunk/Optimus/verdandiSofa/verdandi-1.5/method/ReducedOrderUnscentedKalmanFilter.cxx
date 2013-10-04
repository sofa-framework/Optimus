// Copyright (C) 2008-2010 INRIA
// Author(s): Marc Fragu, Philippe Moireau, Vivien Mallet
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


#ifndef VERDANDI_FILE_METHOD_REDUCEDORDERUNSCENTEDKALMANFILTER_CXX

#include "ReducedOrderUnscentedKalmanFilter.hxx"

#include "seldon/vector/VectorCollection.cxx"

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
    ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::ReducedOrderUnscentedKalmanFilter()
    {
#ifndef VERDANDI_WITH_MPI
        MessageHandler::AddRecipient("model", model_, Model::StaticMessage);
        MessageHandler::AddRecipient("observation_manager",
                                     observation_manager_,
                                     ObservationManager::StaticMessage);
        MessageHandler::AddRecipient("driver",
                                     *this, ReducedOrderUnscentedKalmanFilter
                                     ::StaticMessage);
#else
        MPI_Comm_rank(MPI_COMM_WORLD, &world_rank_);
        MPI_Comm_size(MPI_COMM_WORLD, &Nworld_process_);
        MessageHandler::AddRecipient("model" + to_str(world_rank_), model_,
                                     Model::StaticMessage);
        MessageHandler::AddRecipient("observation_manager"
                                     + to_str(world_rank_),
                                     observation_manager_,
                                     ObservationManager::StaticMessage);
        MessageHandler::AddRecipient("driver" + to_str(world_rank_), *this,
                                     ReducedOrderUnscentedKalmanFilter
                                     ::StaticMessage);
#endif
    }


    //! Destructor.
    template <class Model, class ObservationManager>
    ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::~ReducedOrderUnscentedKalmanFilter()
    {
    }


    /////////////
    // METHODS //
    /////////////


    //! Initializes the driver.
    /*! Initializes the model and the observation manager. Optionally computes
      the analysis of the first step. */
    template <class Model, class ObservationManager>
    void ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
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
    void ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::Initialize(VerdandiOps& configuration,
                 bool initialize_model, bool initialize_observation_manager)
    {
        MessageHandler::Send(*this, "all", "::Initialize begin");


        /***************************
         * Reads the configuration *
         ***************************/


        configuration_file_ = configuration.GetFilePath();
        configuration.SetPrefix("reduced_order_unscented_kalman_filter.");

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
#ifdef VERDANDI_WITH_MPI
        // Should the MPI grid be displayed on screen?
        configuration.Set("display.show_mpi_grid",
                          option_display_["show_mpi_grid"]);
#endif

        /*** Assimilation options ***/

        configuration.Set("data_assimilation.analyze_first_step",
                          analyze_first_step_);
        configuration.Set("data_assimilation.with_resampling",
                          with_resampling_);
        configuration.Set("data_assimilation.observation_error_variance",
                          "ops_in(v, {'matrix', 'matrix_inverse'})",
                          observation_error_variance_);

        /*** Sigma-points ***/

        configuration.Set("sigma_point.type",
                          "ops_in(v, {'canonical', 'star', 'simplex'})",
                          sigma_point_type_);

#if defined(VERDANDI_WITH_MPI)
        configuration.Set("mpi_grid.Nrow", Nrow_);
        configuration.Set("mpi_grid.Ncol", Ncol_);
        SetGridCommunicator(Nrow_, Ncol_, &row_communicator_,
                            &col_communicator_);
        MPI_Comm_rank(row_communicator_, &model_task_);
        MPI_Comm_size(row_communicator_, &Nprocess_);

        if (Nworld_process_ != Nrow_ * Ncol_)
            throw ErrorConfiguration("ObservationGenerator<Model>::Initialize"
                                     , "Wrong number of processes: " +
                                     to_str(Nprocess_)
                                     + ". The dimension of the MPI grid ("
                                     + to_str(Nrow_) + ", " + to_str(Ncol_) +
                                     ") requires " + to_str(Nrow_ * Ncol_)
                                     + " processes.");
#endif

        /*** Ouput saver ***/

#if defined(VERDANDI_WITH_MPI)
        if (model_task_ == 0)
        {
#endif
            configuration.
                SetPrefix("reduced_order_unscented_kalman_filter"
                          ".output_saver.");
            output_saver_.Initialize(configuration);
            output_saver_.Empty("forecast_time");
            output_saver_.Empty("forecast_state");
            output_saver_.Empty("analysis_time");
            output_saver_.Empty("analysis_state");

            /*** Logger and read configuration ***/

            configuration.SetPrefix("reduced_order_unscented_kalman_filter.");

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

        if (configuration.Exists("output.log"))
            Logger::SetFileName(configuration.Get<string>("output.log"));

#ifdef VERDANDI_WITH_MPI
        if (world_rank_ == 0)
        {
            if (option_display_["show_mpi_grid"])
                Logger::StdOut(*this, "world rank\tmodel task\tmodel rank");
            else
                Logger::Log<-3>(*this,
                                "world rank\tmodel task\tmodel rank");
        }

        MPI_Barrier(MPI_COMM_WORLD);
        int model_rank;
        MPI_Comm_rank(col_communicator_, &model_rank);
        if (option_display_["show_mpi_grid"])
            Logger::StdOut(*this, to_str(world_rank_) + "\t\t"
                           + to_str(model_task_) + "\t\t" +
                           to_str(model_rank));
        else
            Logger::Log<-3>(*this, to_str(world_rank_) + "\t\t"
                            + to_str(model_task_) + "\t\t" +
                            to_str(model_rank));
#endif


        /*** Initializations ***/

        if (initialize_model)
        {
#ifdef VERDANDI_WITH_MPI
            model_.SetMPICommunicator(col_communicator_);
#endif
            model_.Initialize(model_configuration_file_);
        }
        if (initialize_observation_manager)
        {
#ifdef VERDANDI_WITH_MPI
            observation_manager_.SetMPICommunicator(col_communicator_);
#endif
            observation_manager_.Initialize(model_,
                                            observation_configuration_file_);
            observation_manager_.DiscardObservation(false);
        }
        Nstate_ = model_.GetNstate();
        Nobservation_ = observation_manager_.GetNobservation();

        Copy(model_.GetStateErrorVarianceReduced(), U_);
        U_inv_.Copy(U_);

        GetInverse(U_inv_);

        Nreduced_ = U_.GetN();

        /*** Sigma-points ***/

        sigma_point_matrix V_trans;
        if (sigma_point_type_ == "canonical")
            ComputeCanonicalSigmaPoint(Nreduced_, V_trans, D_alpha_,
                                       alpha_constant_);
        else if (sigma_point_type_ == "star")
            ComputeStarSigmaPoint(Nreduced_, V_trans, D_alpha_,
                                  alpha_constant_);
        else if (sigma_point_type_ == "simplex")
            ComputeSimplexSigmaPoint(Nreduced_, V_trans, D_alpha_,
                                     alpha_constant_);
        if (alpha_constant_)
            alpha_ = D_alpha_(0);

        Nsigma_point_ = V_trans.GetM();

        // Initializes transpose of I.
        sigma_point_matrix P_alpha_v(Nreduced_, Nreduced_);
        I_trans_.Reallocate(Nsigma_point_, Nreduced_);

        if (alpha_constant_)
        {
            MltAdd(Ts(alpha_), SeldonTrans, V_trans, SeldonNoTrans, V_trans,
                   Ts(0), P_alpha_v);
            GetInverse(P_alpha_v);
            GetCholesky(P_alpha_v);
            MltAdd(Ts(1), SeldonNoTrans, V_trans, SeldonTrans, P_alpha_v,
                   Ts(0), I_trans_);
        }
        else
            throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                 "Initialize()", "Calculation not "
                                 "implemented for no constant alpha_i.");
        I_.Copy(I_trans_);
        Transpose(I_);

        // Initializes D_v.
        D_v_.Reallocate(Nsigma_point_, Nsigma_point_);
        if (alpha_constant_)
            MltAdd(Ts(alpha_ * alpha_), SeldonNoTrans, I_trans_, SeldonTrans,
                   I_trans_, Ts(0), D_v_);
        else
            throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                 "Initialize()", "Calculation not "
                                 "implemented for no constant alpha_i.");

#if defined(VERDANDI_WITH_MPI)

        /*** Local sigma-points ***/

        Nlocal_sigma_point_ = int(Nsigma_point_ / Nprocess_);
        int r = Nsigma_point_ % Nprocess_;
        if (Nprocess_ - model_task_ - 1 < r)
            Nlocal_sigma_point_++;

        Nlocal_sigma_point_sum_.Reallocate(Nprocess_ + 1);
        Nlocal_sigma_point_sum_(0) = 0;
        for (int i = 0; i < Nprocess_; i++)
        {
            Nlocal_sigma_point_sum_(i + 1) = int(Nsigma_point_ / Nprocess_);
            if (Nprocess_ - i - 1 < r)
                Nlocal_sigma_point_sum_(i + 1) += 1;
            Nlocal_sigma_point_sum_(i + 1) += Nlocal_sigma_point_sum_(i);
        }

        x_.SetCommunicator(col_communicator_);
        x_col_.SetCommunicator(col_communicator_);
        Reallocate(x_col_, model_.GetNstate(), model_);
        X_i_.SetCommunicator(col_communicator_);
        Reallocate(X_i_, Nstate_, Nsigma_point_, model_);
        X_i_local_.SetCommunicator(col_communicator_);
        Reallocate(X_i_local_, Nstate_, Nlocal_sigma_point_, model_);
        Z_i_trans_.Reallocate(Nsigma_point_, Nobservation_);
        HL_trans_.Reallocate(Nreduced_, Nobservation_);
        HL_trans_R_.Reallocate(Nreduced_, Nobservation_);
#endif

        /*** Assimilation ***/

        if (analyze_first_step_)
            Analyze();

        if (initialize_model)
        {
#ifndef VERDANDI_WITH_MPI
            MessageHandler::Send(*this, "model", "initial condition");
            MessageHandler::Send(*this, "driver", "initial condition");
#else
            MessageHandler::Send(*this, "model" + to_str(world_rank_),
                                 "initial condition");
            MessageHandler::Send(*this, "driver" + to_str(world_rank_),
                                 "initial condition");
#endif
        }
        MessageHandler::Send(*this, "all", "::Initialize end");
    }


    //! Initializes a step for the unscented Kalman filter.
    /*! Initializes a step for the model.
     */
    template <class Model, class ObservationManager>
    void ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::InitializeStep()
    {
#if defined(VERDANDI_WITH_MPI)
        if (model_task_ == 0)
#endif
            MessageHandler::Send(*this, "all", "::InitializeStep begin");

        model_.InitializeStep();

#if defined(VERDANDI_WITH_MPI)
        if (model_task_ == 0)
#endif
            MessageHandler::Send(*this, "all", "::InitializeStep end");
    }


    //! Performs a step forward, with optimal interpolation at the end.
    template <class Model, class ObservationManager>
    void ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::Forward()
    {
        MessageHandler::Send(*this, "all", "::Forward begin");

        if (sigma_point_type_ == "simplex")
        {
#if defined(VERDANDI_WITH_MPI)

            x_.Copy(model_.GetState());

            /*** Sampling ***/

            int Nlocal_state = model_.GetLocalNstate();
            // Local rows, global columns.
            Matrix<double> X_i_global_double;
            model_state_error_variance tmp;
            if (model_task_ == 0)
            {
                X_i_global_double.Reallocate(Nlocal_state, Nsigma_point_);
                GetCholesky(U_inv_);
                Copy(model_.GetStateErrorVarianceProjector(), tmp);
                MltAdd(Ts(1), tmp, U_inv_, Ts(0),
                       model_.GetStateErrorVarianceProjector());
                // Computes X_n^{(i)+}.
                for (int i = 0; i < Nsigma_point_; i++)
                    SetCol(x_, i, X_i_);
                MltAdd(Ts(1), model_.GetStateErrorVarianceProjector(),
                       I_, Ts(1), X_i_);
                Copy(X_i_, X_i_global_double);
                Transpose(X_i_global_double);
            }

            int displacement[Nprocess_],  sendcount[Nprocess_];
            for (int i = 0; i < Nprocess_; i++)
            {
                sendcount[i] = (Nlocal_sigma_point_sum_(i + 1)
                                - Nlocal_sigma_point_sum_(i)) * Nlocal_state;
                displacement[i] = Nlocal_sigma_point_sum_(i) * Nlocal_state;
            }

            // Local rows, local columns.
            Matrix<double> X_i_local_double(Nlocal_sigma_point_, Nlocal_state);
            MPI_Scatterv(X_i_global_double.GetData(),
                         sendcount, displacement, MPI_DOUBLE,
                         X_i_local_double.GetData(),
                         Nlocal_sigma_point_ * Nlocal_state,
                         MPI_DOUBLE, 0, row_communicator_);
            Transpose(X_i_local_double);

            Copy(X_i_local_double, X_i_local_);

            /*** Prediction ***/

            // Computes X_{n + 1}^-.
            x_.Zero();
            double new_time;
            for (int i = 0; i < Nlocal_sigma_point_; i++)
            {
                GetCol(X_i_local_, i, x_col_);
                new_time = model_.ApplyOperator(x_col_, false);
                Add(Ts(alpha_), x_col_, x_);
                SetCol(x_col_, i, X_i_local_);
            }
            model_.SetTime(new_time);

            Vector<double> x_double, working_vector;
            Copy(x_, x_double);
            if (model_task_ == 0)
            {
                working_vector.Reallocate(Nlocal_state);
                working_vector.Fill(0.);
            }

            MPI_Reduce(x_double.GetData(), working_vector.GetData(),
                       x_double.GetM(), MPI_DOUBLE, MPI_SUM,
                       0, row_communicator_);

            if (model_task_ == 0)
            {
                Copy(working_vector, model_.GetState());
                model_.StateUpdated();
            }

            /*** Resampling ***/

            Copy(X_i_local_, X_i_local_double);
            if (model_task_ == 0)
                X_i_global_double.Reallocate(Nsigma_point_, Nlocal_state);

            Transpose(X_i_local_double);

            MPI_Gatherv(X_i_local_double.GetData(),
                        Nlocal_sigma_point_ * Nlocal_state, MPI_DOUBLE,
                        X_i_global_double.GetData(),  sendcount,
                        displacement, MPI_DOUBLE, 0, row_communicator_);
            if (model_task_ == 0)
            {
                Transpose(X_i_global_double);
                Copy(X_i_global_double, X_i_);
            }

            if (with_resampling_)
                throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                     "Forward()", "'resampling 'option "
                                     "not supported yet.");
            // Computes L_{n + 1}.
            if (model_task_ == 0)
                MltAdd(Ts(alpha_), X_i_, I_trans_, Ts(0),
                       model_.GetStateErrorVarianceProjector());

#else
            model_state_error_variance_row x(Nstate_);
            Copy(model_.GetState(), x);

            /*** Sampling ***/

            model_state_error_variance tmp;
            GetCholesky(U_inv_);

            Copy(model_.GetStateErrorVarianceProjector(), tmp);
            MltAdd(Ts(1), tmp, U_inv_, Ts(0),
                   model_.GetStateErrorVarianceProjector());

            // Computes X_n^{(i)+}.
            Reallocate(X_i_, Nstate_, Nsigma_point_, model_);
            for (int i = 0; i < Nsigma_point_; i++)
                SetCol(x, i, X_i_);

            MltAdd(Ts(1), model_.GetStateErrorVarianceProjector(),
                   I_, Ts(1), X_i_);

            /*** Prediction ***/

            // Computes X_{n + 1}^-.
            x.Fill(Ts(0));
            model_state_error_variance_row x_col;
            Reallocate(x_col, x.GetM(), model_);
            double new_time(0);
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetCol(X_i_, i, x_col);
                new_time = model_.ApplyOperator(x_col, false);
                Add(Ts(alpha_), x_col, x);
                SetCol(x_col, i, X_i_);
            }
            model_.SetTime(new_time);

            /*** Resampling ***/

            if (with_resampling_)
            {
#if defined(VERDANDI_WITH_PETSC)
                throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                     "Forward()", "'resampling 'option "
                                     "not supported yet.");
#else
                MltAdd(Ts(alpha_), X_i_, I_trans_, Ts(0),
                       model_.GetStateErrorVarianceProjector());
                for(int i = 0; i < Nsigma_point_; i++)
                    SetCol(x, i, X_i_);
                MltAdd(Ts(1), model_.GetStateErrorVarianceProjector(),
                       I_, Ts(1), X_i_);
#endif
            }

            // Computes L_{n + 1}.
            MltAdd(Ts(alpha_), X_i_, I_trans_, Ts(0),
                   model_.GetStateErrorVarianceProjector());

            model_.GetState().Copy(x);
            model_.StateUpdated();
#endif
        }
        else
        {
#if defined(VERDANDI_WITH_MPI) || defined(VERDANDI_WITH_PETSC)
            throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                 "Forward()", "Parallel algorithm not "
                                 "implemented yet for the 'no"
                                 " simplex' cases.");
#else
            model_state_error_variance_row x(Nstate_);
            Copy(model_.GetState(), x);

            /*** Sampling ***/

            sigma_point_matrix tmp;
            GetCholesky(U_inv_);
            Copy(model_.GetStateErrorVarianceProjector(), tmp);
            MltAdd(Ts(1), tmp, U_inv_, Ts(0),
                   model_.GetStateErrorVarianceProjector());

            // Computes X_n^{(i)+}.
            X_i_trans_.Reallocate(Nsigma_point_, Nstate_);
            sigma_point x_col;
            for (int i = 0; i < Nsigma_point_; i++)
                SetRow(x, i, X_i_trans_);

            MltAdd(Ts(1), SeldonNoTrans, I_trans_, SeldonTrans,
                   model_.GetStateErrorVarianceProjector(),
                   Ts(1), X_i_trans_);

            /*** Prediction ***/

            // Computes X_{n + 1}^-.
            x.Fill(Ts(0));
            double new_time(0);
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRow(X_i_trans_, i, x_col);
                new_time = model_.ApplyOperator(x_col, false);
                Add(Ts(alpha_), x_col, x);
                SetRow(x_col, i, X_i_trans_);
            }
            model_.SetTime(new_time);

            model_.GetState().Copy(x);
            model_.StateUpdated();

            /*** Resampling with SVD ***/

            sigma_point_matrix M_trans(Nsigma_point_, Nstate_);
            for (int i = 0; i < Nsigma_point_; i++)
                SetRow(x, i, M_trans);
            Mlt(Ts(-1), M_trans);
            Add(Ts(1), X_i_trans_, M_trans);

            if (alpha_constant_)
                Mlt(sqrt(alpha_), M_trans);
            else
                throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                     "Forward()", "Calculation not "
                                     "implemented for no constant alpha_i.");

            sigma_point_matrix G(Nsigma_point_, Nsigma_point_);
            MltAdd(Ts(1), SeldonNoTrans, M_trans, SeldonTrans, M_trans,
                   Ts(0), G);

            Vector<Ts> lambda;
            Matrix<Ts> U, V;
            GetSVD(G, lambda, U, V);
            U.Resize(Nsigma_point_, Nreduced_);

            sigma_point_matrix
                working_matrix_rr(Nsigma_point_, Nsigma_point_),
                working_matrix_rN(X_i_trans_);

            MltAdd(Ts(sqrt(alpha_)), SeldonNoTrans, U, SeldonTrans, I_trans_,
                   Ts(0), working_matrix_rr);

            for(int i = 0; i < Nsigma_point_; i++)
                SetRow(x, i, X_i_trans_);
            Add(Ts(-1), X_i_trans_, working_matrix_rN);

            MltAdd(Ts(1), SeldonTrans, working_matrix_rr, SeldonNoTrans,
                   working_matrix_rN, Ts(1), X_i_trans_);

            // Computes L_{n + 1}.
            MltAdd(Ts(alpha_), SeldonTrans, X_i_trans_, SeldonNoTrans,
                   I_trans_, Ts(0), model_.GetStateErrorVarianceProjector());
#endif
        }

#ifndef VERDANDI_WITH_MPI
        MessageHandler::Send(*this, "model", "forecast");
        MessageHandler::Send(*this, "observation_manager", "forecast");
        MessageHandler::Send(*this, "driver", "forecast");
#else
        MessageHandler::Send(*this, "model" +
                             to_str(world_rank_), "forecast");
        MessageHandler::Send(*this, "observation_manager" +
                             to_str(world_rank_), "forecast");
        MessageHandler::Send(*this, "driver" + to_str(world_rank_),
                             "forecast");
#endif
        MessageHandler::Send(*this, "all", "::Forward end");
    }


    //! Computes an analysis.
    /*! Whenever observations are available, it computes BLUE. */
    template <class Model, class ObservationManager>
    void ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::Analyze()
    {

#if defined(VERDANDI_WITH_MPI)
        if (model_task_ == 0)
#endif
            MessageHandler::Send(*this, "all", "::Analyze begin");

        observation_manager_.SetTime(model_, model_.GetTime());

        if (!observation_manager_.HasObservation())
        {
#if defined(VERDANDI_WITH_MPI)
            if (model_task_ == 0)
#endif
                MessageHandler::Send(*this, "all", "::Analyze end");
            return;
        }

        Nobservation_  = observation_manager_.GetNobservation();

#if defined(VERDANDI_WITH_MPI)
        if (world_rank_ == 0)
        {
#endif
            if (option_display_["show_time"])
                cout << "Performing Reduced Order UKF at time step ["
                     << model_.GetTime() << "]..." << endl;
#if defined(VERDANDI_WITH_MPI)
        }
#endif

        if (sigma_point_type_ == "simplex")
        {
#if defined(VERDANDI_WITH_MPI)
            observation z_col(Nobservation_), z(Nobservation_);
            z.Fill(To(0));
            observation reduced_innovation(Nreduced_);
            if (model_task_ == 0)
            {
                for (int i = 0; i < Nsigma_point_; i++)
                {
                    GetCol(X_i_, i, x_col_);
                    observation& z_col =
                        observation_manager_.GetInnovation(x_col_);
                    Add(To(alpha_), z_col, z);
                    SetRow(z_col, i, Z_i_trans_);
                }
                MltAdd(Ts(alpha_), SeldonTrans, I_trans_, SeldonNoTrans,
                       Z_i_trans_, Ts(0), HL_trans_);
                observation_error_variance R_inv;
                if (observation_error_variance_ == "matrix_inverse")
                    Mlt(HL_trans_, observation_manager_.
                        GetErrorVarianceInverse(), HL_trans_R_);
                else
                {
                    observation_error_variance R_inv;
                    Copy(observation_manager_.GetErrorVariance(), R_inv);
                    GetInverse(R_inv);
                    Mlt(HL_trans_, R_inv, HL_trans_R_);
                }
                U_inv_.SetIdentity();
                MltAdd(Ts(1), SeldonNoTrans, HL_trans_R_,
                       SeldonTrans, HL_trans_, Ts(1), U_inv_);
                GetInverse(U_inv_);
                MltAdd(Ts(1), U_inv_, HL_trans_R_, Ts(0), HL_trans_);
                MltAdd(Ts(-1), HL_trans_, z, Ts(0), reduced_innovation);
            }
            MltAdd(Ts(1), model_.GetStateErrorVarianceProjector(),
                   reduced_innovation, Ts(1), model_.GetState());
            model_.StateUpdated();
#else
            // Computes [HX_{n+1}^{*}].
            sigma_point_matrix Z_i_trans(Nsigma_point_, Nobservation_);
            model_state_error_variance_row x_col;
            Reallocate(x_col, Nstate_, model_);
            model_.StateUpdated();

            observation z(Nobservation_);
            z.Fill(To(0));
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetCol(X_i_, i, x_col);
                observation& z_col =
                    observation_manager_.GetInnovation(x_col);
                Add(To(alpha_), z_col, z);
                SetRow(z_col, i, Z_i_trans);
            }

            sigma_point_matrix HL_trans(Nreduced_, Nobservation_);
            MltAdd(Ts(alpha_), SeldonTrans, I_trans_, SeldonNoTrans, Z_i_trans,
                   Ts(0), HL_trans);

            observation_error_variance R_inv;
            sigma_point_matrix working_matrix_po(Nreduced_, Nobservation_),
                tmp;

            if (observation_error_variance_ == "matrix_inverse")
                Mlt(HL_trans, observation_manager_.GetErrorVarianceInverse(),
                    working_matrix_po);
            else
            {
                observation_error_variance R_inv;
                Copy(observation_manager_.GetErrorVariance(), R_inv);
                GetInverse(R_inv);
                Mlt(HL_trans, R_inv, working_matrix_po);
            }

            U_inv_.SetIdentity();
            MltAdd(Ts(1), SeldonNoTrans, working_matrix_po,
                   SeldonTrans, HL_trans, Ts(1), U_inv_);
            GetInverse(U_inv_);

            tmp.Reallocate(Nreduced_, Nobservation_);
            tmp.Fill(Ts(0));

            observation reduced_innovation(Nreduced_);
            MltAdd(Ts(1), U_inv_, working_matrix_po, Ts(0), tmp);
            MltAdd(Ts(-1), tmp, z, Ts(0), reduced_innovation);

            // Updates.
            model_state& x =  model_.GetState();
            MltAdd(Ts(1), model_.GetStateErrorVarianceProjector(),
                   reduced_innovation, Ts(1), x);
            model_.StateUpdated();
#endif
        }
        else
        {
#if defined(VERDANDI_WITH_MPI) || defined(VERDANDI_WITH_PETSC)
            throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                 "Analyse()", "Parallel algorithm not"
                                 " implemented yet for the 'no"
                                 " simplex' cases.");
#else
            // Computes [HX_{n+1}^{*}].
            sigma_point_matrix Z_i_trans(Nsigma_point_, Nobservation_);
            sigma_point x_col;
            observation z(Nobservation_);
            z.Fill(To(0));
            if (!alpha_constant_)
                throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                     "Analyse()", "Calculation not "
                                     "implemented for non constant alpha_i.");

            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(X_i_trans_, i, x_col);
                observation& z_col =
                    observation_manager_.GetInnovation(x_col);
                Add(To(alpha_), z_col, z);
                SetRow(z_col, i, Z_i_trans);
                x_col.Nullify();
            }

            observation z_col(Nobservation_);
            // Computes [Z] = [HX_{n+1}^{*} - E(HX_{n+1}^{*})].
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRowPointer(Z_i_trans, i, z_col);
                Add(To(-1), z, z_col);
                z_col.Nullify();
            }

            sigma_point_matrix
                working_matrix_ro(Nsigma_point_, Nobservation_),
                D_m(Nsigma_point_, Nsigma_point_);
            sigma_point_matrix HL_trans;

            observation_error_variance R_inv;
            if (observation_error_variance_ == "matrix_inverse")
                Mlt(Z_i_trans, observation_manager_.GetErrorVarianceInverse(),
                    working_matrix_ro);
            else
            {
                Copy(observation_manager_.GetErrorVariance(), R_inv);
                GetInverse(R_inv);
                Mlt(Z_i_trans, R_inv, working_matrix_ro);
            }

            // Computes D_m.
            MltAdd(To(1), SeldonNoTrans, working_matrix_ro, SeldonTrans,
                   Z_i_trans, To(0), D_m);

            // Computes U_{n+1}.
            sigma_point_matrix
                working_matrix_rp(Nsigma_point_, Nreduced_),
                working_matrix_rr(Nsigma_point_, Nsigma_point_),
                working_matrix_rr2(Nsigma_point_, Nsigma_point_),
                working_matrix_rr3(Nsigma_point_, Nsigma_point_);

            Copy(D_v_, working_matrix_rr);
            Mlt(Ts(-1), working_matrix_rr);
            if (alpha_constant_)
            {
                for(int i = 0; i < Nsigma_point_; i++ )
                    working_matrix_rr(i, i) += alpha_;
                MltAdd(Ts(1), D_m, working_matrix_rr, Ts(0),
                       working_matrix_rr2);
                for(int i = 0; i < Nsigma_point_; i++ )
                    working_matrix_rr2(i, i) += 1;
                GetInverse(working_matrix_rr2);
                MltAdd(Ts(1), working_matrix_rr2, D_m, Ts(0),
                       working_matrix_rr);
                MltAdd(Ts(alpha_), working_matrix_rr, I_trans_, Ts(0),
                       working_matrix_rp);
                U_.SetIdentity();
                MltAdd(Ts(alpha_), SeldonTrans, I_trans_, SeldonNoTrans,
                       working_matrix_rp, Ts(1), U_);

                Copy(U_, U_inv_);
                GetInverse(U_inv_);

                // Computes {HL}_{n+1}.
                HL_trans.Reallocate(Nreduced_, Nobservation_);
                working_matrix_rr2.SetIdentity();
                MltAdd(Ts(1), D_v_, working_matrix_rr, Ts(1),
                       working_matrix_rr2);

                working_matrix_rr.SetIdentity();
                Add(Ts(alpha_), D_m, working_matrix_rr);
                GetInverse(working_matrix_rr);

                Mlt(working_matrix_rr, working_matrix_rr2,
                    working_matrix_rr3);

                MltAdd(Ts(alpha_), working_matrix_rr3, I_trans_, Ts(0),
                       working_matrix_rp);

                MltAdd(Ts(1), SeldonTrans, working_matrix_rp, SeldonNoTrans,
                       Z_i_trans, Ts(0), HL_trans);
            }
            else
                throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                     "Analyse()", "Calculation not "
                                     "implemented for no constant alpha_i.");

            // Computes K.
            sigma_point_matrix K(Nstate_, Nobservation_),
                working_matrix_po(Nreduced_, Nobservation_),
                working_matrix_po2(Nreduced_, Nobservation_);

            if (observation_error_variance_ == "matrix_inverse")
                Mlt(HL_trans, observation_manager_.GetErrorVarianceInverse(),
                    working_matrix_po);
            else
                Mlt(HL_trans, R_inv, working_matrix_po);
            Mlt(U_inv_, working_matrix_po, working_matrix_po2);
            Mlt(model_.GetStateErrorVarianceProjector(),
                working_matrix_po2, K);

            // Updates.
            model_state& x =  model_.GetState();
            MltAdd(Ts(-1), K, z, Ts(1), x);
            model_.StateUpdated();
#endif
        }

#ifndef VERDANDI_WITH_MPI
        MessageHandler::Send(*this, "model", "forecast");
        MessageHandler::Send(*this, "observation_manager", "forecast");
        MessageHandler::Send(*this, "driver", "forecast");
#else
        MessageHandler::Send(*this, "model" +
                             to_str(world_rank_), "analysis");
        MessageHandler::Send(*this, "observation_manager" +
                             to_str(world_rank_), "analysis");
        MessageHandler::Send(*this, "driver" + to_str(world_rank_),
                             "analysis");
#endif
        MessageHandler::Send(*this, "all", "::Analyze end");
    }


    //! Finalizes a step for the model.
    template <class Model, class ObservationManager>
    void ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::FinalizeStep()
    {
        MessageHandler::Send(*this, "all", "::FinalizeStep begin");

        model_.FinalizeStep();

        MessageHandler::Send(*this, "all", "::FinalizeStep end");
    }


    //! Finalizes the model.
    template <class Model, class ObservationManager>
    void ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
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
    bool ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
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
    ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
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
    ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
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
    ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
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
    ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::GetName() const
    {
        return "ReducedOrderUnscentedKalmanFilter";
    }


    //! Receives and handles a message.
    /*
      \param[in] message the received message.
    */
    template <class Model, class ObservationManager>
    void ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::Message(string message)
    {
#if defined(VERDANDI_WITH_MPI)
        if (model_task_ == 0)
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


#define VERDANDI_FILE_METHOD_REDUCEDORDERUNSCENTEDKALMANFILTER_CXX
#endif
