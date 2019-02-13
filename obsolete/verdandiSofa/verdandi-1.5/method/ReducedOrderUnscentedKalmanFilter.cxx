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
        observation_manager_ = new ObservationManager;
        step = 0;
#ifndef VERDANDI_WITH_MPI
        MessageHandler::AddRecipient("model", model_, Model::StaticMessage);
        MessageHandler::AddRecipient("observation_manager",
                                     *observation_manager_,
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
                                     *observation_manager_,
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
        timer = new CTime(); /// ADDIP
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

        configuration.Set("output_directory", "", configuration_file_, output_directory_);

        char comm[100];
        sprintf(comm, "rm %s/roukf*dat", output_directory_.c_str());        
        int sr = system(comm);
        if (sr == 0)
            std::cout << "Executed: " << comm << std::endl;

        configuration.SetPrefix("reduced_order_unscented_kalman_filter.");

        /*** Model ***/
        configuration.Set("output.saveVQ", saveVQ_);

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
            observation_manager_->SetMPICommunicator(col_communicator_);
#endif
            observation_manager_->Initialize(model_,
                                            observation_configuration_file_);
            observation_manager_->DiscardObservation(false);
        }
        Nstate_ = model_.GetNstate();
        Nobservation_ = observation_manager_->GetNobservation();

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


    /** ADJUSTED **/
    //! Performs a step forward, with optimal interpolation at the end.
    template <class Model, class ObservationManager>
    void ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::Forward()
    {
        MessageHandler::Send(*this, "all", "::Forward begin");

        if (sigma_point_type_ == "simplex")
        {
            MessageHandler::Send(*this, "all", "::Simplex");
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
            TIC;
            model_state_error_variance_row x(Nstate_);

            Copy(model_.GetState(), x); // copy former to latter, former is a verdandi state propagated from sofa state (result of last step)

            /*** Sampling ***/

            model_state_error_variance tmp;            
            GetCholesky(U_inv_); // sqrt U_inv_ ... Cn
            TOCTIC("== pre-forward 1== ");


            Copy(model_.GetStateErrorVarianceProjector(), tmp); // projector is copied to tmp (created if necessary) ... Ln
            //TOCTIC("== pre-forward2a == ");
            MltAdd(Ts(1), tmp, U_inv_, Ts(0),
                   model_.GetStateErrorVarianceProjector()); // projector*Uinv->projector ... projector returns REFERENCE and can be modified ... Ln CnT

            //TOCTIC("== pre-forward2b (mltadd) == ");
            // Computes X_n^{(i)+}.
            Reallocate(X_i_, Nstate_, Nsigma_point_, model_); // allocate  ... Nstate*Nsigma_point_ size to X_i
            //TOCTIC("== pre-forward3a == ");
            for (int i = 0; i < Nsigma_point_; i++)
                SetCol(x, i, X_i_); // all columns set to the verdandi state x
            //TOCTIC("== pre-forward3b == ");
            
            model_state_error_variance& temp = model_.GetStateErrorVarianceProjector();            
            //TOCTIC("== pre-forward3c == ");
            MltAdd(Ts(1), temp, I_, Ts(1), X_i_); // sigma points ready ... projector*I_ + X_i -> X_i            
            //TOCTIC("== pre-forward3d (mltadd) == ");

            /*** Prediction ***/

            // Computes X_{n + 1}^-.

            x.Fill(Ts(0));
            double new_time(0);
            TOCTIC("== pre-forward4 == ");

            /**
             * -----------------------------START OF ADJUSTED SECTION---------------------------------
             * The following of code has been modified for parallelization.
             */            
            bool parallel_mode = false;


            sigma_point x_col;
            Reallocate(x_col, x.GetM(), model_);

            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetCol(X_i_, i, x_col);
                //std::cout << "Apply operator start..." << std::endl;
                new_time = model_.ApplyOperator(x_col, false);
                //std::cout << "Apply operator done!" << std::endl;
                if (new_time<0.0f)
                {
                    parallel_mode=true;
                    break;
                }
                Add(Ts(alpha_), x_col, x); // B<- alpha A + B
                SetCol(x_col, i, X_i_);
            }
            //asumSVec("summPred",x);
            //TOCTIC("== seq mode ==");
            if (parallel_mode)
            {

                MessageHandler::Send(*this, "all", "::Parallel Mode");
                // generate sigma points for ROUKF
                // new allocation every time so as not to disrupt the code in overly many locations

                x.Fill(Ts(0));
                sigma_point* verdandi_states = new sigma_point[Nsigma_point_]; // dynamic allocation - verdandi_states
                for (int i = 0; i < Nsigma_point_; i++)
                {
                    verdandi_states[i].Nullify();
                    verdandi_states[i].Resize(x.GetM());
                }
                for (int i = 0; i < Nsigma_point_; i++)
                {
                    GetCol(X_i_, i, verdandi_states[i]);
                }

                // compute the simulation step for all sigma points in parallel
                new_time = model_.ApplyOperatorParallel(verdandi_states, false);

                // generate apriori state estimate
                for (int i = 0; i < Nsigma_point_; i++)
                {

                    Add(Ts(alpha_), verdandi_states[i], x);  // B<- alpha A + B, x will be apriori state estimate (apriori mean)
                    SetCol(verdandi_states[i], i, X_i_); // generating Xn+1 [*]
                }


                //
                delete[] verdandi_states; // free the memory - verdandi_states
            }
            //std::cout << "X = " << x << std::endl;
            TOCTIC("== forward ==");
            /**
             * -----------------------------END OF ADJUSTED SECTION---------------------------------
             */            
            model_.SetTime(new_time);
            /*** Resampling ***/

            if (with_resampling_)
            {std::cout<<"\nRESAMPLING!!\n";
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
            //TOCTIC("== post-forward1 ==");

            // Computes L_{n + 1}.
            MltAdd(Ts(alpha_), X_i_, I_trans_, Ts(0),
                   model_.GetStateErrorVarianceProjector());

            model_.GetState().Copy(x);
            //asumSVec("sumFinPred",x);
            model_.StateUpdated();
            ////asumSVec("state endpred.: ", x);
#endif
            TOC("== post-forward2 ==");
        }
        else
        {
#if defined(VERDANDI_WITH_MPI) || defined(VERDANDI_WITH_PETSC)
            throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                 "Forward()", "Parallel algorithm not "
                                 "implemented yet for the 'no"
                                 " simplex' cases.");
#else
            TIC;
            MessageHandler::Send(*this, "all", "::nonSimplex");
            model_state_error_variance_row x(Nstate_);
            Copy(model_.GetState(), x);
            
            /*** Sampling ***/
            
            sigma_point_matrix tmp;            
            GetCholesky(U_inv_);
            TOCTIC(" == nsx pre-forward1 ==");

            if (saveVQ_){
                char name[100];
                sprintf(name, "%s/roukf-forecast_uinv.dat", output_directory_.c_str());
                output_saver_.WriteText(U_inv_, name);
            }
            
            TOCTIC(" == nsx pre-forward2 ==");

            Copy(model_.GetStateErrorVarianceProjector(), tmp);
            TOCTIC(" == nsx pre-forward3 ==");
            MltAdd(Ts(1), tmp, U_inv_, Ts(0),
                   model_.GetStateErrorVarianceProjector());
            TOCTIC(" == nsx pre-forward4 (mltadd) ==");

            // Computes X_n^{(i)+}.
            X_i_trans_.Reallocate(Nsigma_point_, Nstate_);
            sigma_point x_col;
            for (int i = 0; i < Nsigma_point_; i++)
                SetRow(x, i, X_i_trans_);
            TOCTIC(" == nsx pre-forward5 ==");

            //// print error variance projector begin
            if (0) {
                model_state_error_variance& m1 = model_.GetStateErrorVarianceProjector();
                size_t errVarM = model_.GetStateErrorVarianceProjector().GetM();
                size_t errVarN = model_.GetStateErrorVarianceProjector().GetN();
                for (size_t i = 0; i < 6; i++) {
                    for (size_t j = 0; j < errVarN; j++)
                        std::cout << m1(i,j) << " ";
                    std::cout << std::endl;
                }
                std::cout << " ... FWD BEGIN ... " << std::endl;
                for (size_t i = errVarM-errVarN; i < errVarM; i++) {
                    for (size_t j = 0; j < errVarN; j++)
                        std::cout << m1(i,j) << " ";
                    std::cout << std::endl;
                }
            }
            //// print error variance projector end


            MltAdd(Ts(1), SeldonNoTrans, I_trans_, SeldonTrans,
                   model_.GetStateErrorVarianceProjector(),
                   Ts(1), X_i_trans_);
            TOCTIC(" == nsx pre-forward6 (mltadd) ==");

            /*** Prediction ***/

            // Computes X_{n + 1}^-.

            x.Fill(Ts(0));
            double new_time(0);            

            /**
             * -----------------------------START OF ADJUSTED SECTION---------------------------------
             * The following of code has been modified for parallelization.
             */            
            bool parallel_mode = false;

            //Reallocate(x_col, x.GetM(), model_);

            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetRow(X_i_trans_, i, x_col);
                if (saveVQ_){
                    char name[100];
                    sprintf(name, "%s/roukf-forecast-sigmastate_%02d.dat", output_directory_.c_str(), i);
                    output_saver_.WriteText(x_col, name);
                }

                new_time = model_.ApplyOperator(x_col, false);
                if (new_time<0.0f)
                {
                    parallel_mode=true;
                    break;
                }

                Add(Ts(alpha_), x_col, x);
                SetRow(x_col, i, X_i_trans_);
            }
            TOCTIC("== seq mode nsx == ");            
            if (parallel_mode)
            {

                //model_.SetTime(new_time);
                //model_.GetState().Copy(x);
                //model_.StateUpdated();


                MessageHandler::Send(*this, "all", "::Parallel Mode");
                cout<<"Sigma points count "<<Nsigma_point_<<std::endl;
                // generate sigma points for ROUKF
                // new allocation every time so as not to disrupt the code in too many locations
                sigma_point* verdandi_states = new sigma_point[Nsigma_point_]; // dynamic allocation - verdandi_states
                for (int i = 0; i < Nsigma_point_; i++)
                {
                    verdandi_states[i].Nullify();
                    verdandi_states[i].Resize(x.GetM());
                }

                for (int i = 0; i < Nsigma_point_; i++)
                {
                    // GetCol(X_i_, i, verdandi_states[i]);
                    GetRow(X_i_trans_, i, verdandi_states[i]);
                    if (saveVQ_){
                        char name[100];
                        sprintf(name, "%s/roukf-forecast-sigmastate_%02d.dat", output_directory_.c_str(), i);
                        output_saver_.WriteText(verdandi_states[i], name);
                    }
                }

                // compute the simulation step for all sigma points in parallel
                new_time = model_.ApplyOperatorParallel(verdandi_states, false);

                for (int i = 0; i < Nsigma_point_; i++)
                {

                    Add(Ts(alpha_), verdandi_states[i], x);  // B<- alpha A + B, x will be apriori state estimate (apriori mean)
                    // SetCol(verdandi_states[i], i, X_i_);
                    SetRow(verdandi_states[i], i, X_i_trans_); // generating Xn+1 [*]
                }
                delete[] verdandi_states; // free the memory - verdandi_states
            }
            TOCTIC("== par mode nsx ==");
            /**
             * -----------------------------END OF ADJUSTED SECTION---------------------------------
             */            
            model_.SetTime(new_time);

            model_.GetState().Copy(x);
            model_.StateUpdated();
            TOCTIC(" == nsx post-forward1 ==");

            /*** Resampling with SVD ***/

            sigma_point_matrix M_trans(Nsigma_point_, Nstate_);
            for (int i = 0; i < Nsigma_point_; i++)
                SetRow(x, i, M_trans);
            Mlt(Ts(-1), M_trans);
            Add(Ts(1), X_i_trans_, M_trans);
            TOCTIC(" == nsx post-forward2 ==");


            if (alpha_constant_)
                Mlt(sqrt(alpha_), M_trans);
            else
                throw ErrorUndefined("ReducedOrderUnscentedKalmanFilter::"
                                     "Forward()", "Calculation not "
                                     "implemented for no constant alpha_i.");
            TOC(" == nsx post-forward3 ==");
            
            sigma_point_matrix G(Nsigma_point_, Nsigma_point_);
            
            //M_trans.WriteText("Mt.txt");
            TIC
            MltAdd(Ts(1), SeldonNoTrans, M_trans, SeldonTrans, M_trans,
                   Ts(0), G);
            TOC(" == nsx post-forward4 (mltadd) ==");
            //G.WriteText("preSvdG.txt");
            
            Vector<Ts> svdLambda;
            Matrix<Ts> svdU, svdV;
            TIC
            GetSVD(G, svdLambda, svdU, svdV);
            TOC(" == nsx post-forward5 (SVD) ==");
            
            //svdU.WriteText("svdU.txt");
            //svdV.WriteText("svdV.txt");
            //svdLambda.WriteText("lambda.txt");
            //G.WriteText("postSvdG.txt");
            
            TIC

            //std::cout << "Lambda = " << lambda << std::endl;
            //std::cout << "U = " << U << std::endl;
            //std::cout << "V = " << V << std::endl;

            svdU.Resize(Nsigma_point_, Nreduced_);
            //svdU.WriteText("svdU2.txt");
            //// print error variance projector begin
            if (0) {
                model_state_error_variance& m1 = model_.GetStateErrorVarianceProjector();
                size_t errVarM = model_.GetStateErrorVarianceProjector().GetM();
                size_t errVarN = model_.GetStateErrorVarianceProjector().GetN();
                for (size_t i = 0; i < 6; i++) {
                    for (size_t j = 0; j < errVarN; j++)
                        std::cout << m1(i,j) << " ";
                    std::cout << std::endl;
                }
                std::cout << " ... FWD 2 ... " << std::endl;
                for (size_t i = errVarM-errVarN; i < errVarM; i++) {
                    for (size_t j = 0; j < errVarN; j++)
                        std::cout << m1(i,j) << " ";
                    std::cout << std::endl;
                }
            }
            //// print error variance projector end

            //std::cout << "Ures = " << U << std::endl;

            sigma_point_matrix
                working_matrix_rr(Nsigma_point_, Nsigma_point_),
                working_matrix_rN(X_i_trans_);
            TOCTIC(" == nsx post-forward6 ==");            
                          
            MltAdd(Ts(sqrt(alpha_)), SeldonNoTrans, svdU, SeldonTrans, I_trans_,
                   Ts(0), working_matrix_rr);
            TOCTIC(" == nsx post-forward7 (mltadd) ==");

            for(int i = 0; i < Nsigma_point_; i++)
                SetRow(x, i, X_i_trans_);
            Add(Ts(-1), X_i_trans_, working_matrix_rN);

            //std::cout << "Wrn = " << working_matrix_rN << std::endl;

            MltAdd(Ts(1), SeldonTrans, working_matrix_rr, SeldonNoTrans,
                   working_matrix_rN, Ts(1), X_i_trans_);

            // Computes L_{n + 1}.
                                                            
            MltAdd(Ts(alpha_), SeldonTrans, X_i_trans_, SeldonNoTrans,
                   I_trans_, Ts(0), model_.GetStateErrorVarianceProjector());
                                                
            TOCTIC(" == nsx post-forward8 (mltadd) ==")
            //std::cout << "VarProj = " << model_.GetStateErrorVarianceProjector() << std::endl;

            /*std::cout << "END FORWARD: ";
            for (size_t i = 0; i < 12; i++)
                std::cout << model_.state_(i) << " ";
            std::cout << std::endl;
            std::cout << "ITRANS = " << I_trans_ << std::endl;*/

            //// print error variance projector begin
            if (0) {
                model_state_error_variance& m1 = model_.GetStateErrorVarianceProjector();
                size_t errVarM = model_.GetStateErrorVarianceProjector().GetM();
                size_t errVarN = model_.GetStateErrorVarianceProjector().GetN();
                for (size_t i = 0; i < 6; i++) {
                    for (size_t j = 0; j < errVarN; j++)
                        std::cout << m1(i,j) << " ";
                    std::cout << std::endl;
                }
                std::cout << " ... FWD END ... " << std::endl;
                for (size_t i = errVarM-errVarN; i < errVarM; i++) {
                    for (size_t j = 0; j < errVarN; j++)
                        std::cout << m1(i,j) << " ";
                    std::cout << std::endl;
                }
            }
            TOC("== post-forward nsx ==");
            //// print error variance projector end

            if (saveVQ_){
                //typename Model::state_error_variance LL = model_.GetStateErrorVarianceProjector();
                //Seldon::Vector<Ts> LLv(Nreduced_* Nreduced_);
                //for (int i = 0; i < Nreduced_; i++)
                //    for (int j = 0; j < Nreduced_; j++)
                //        LLv(3*i+j) = LL(LL.GetM()-3+i,j);

                //std::cout << "SIZES" << LL.GetN() << " x " << LL.GetM() << " " << Nreduced_ << std::endl;
                //char name[100];
                //sprintf(name, "%s/roukf-forecast-Lmat.dat", output_directory_.c_str());
                //output_saver_.WriteText(LL, name);
                //sprintf(name, "%s/roukf-forecast-Lvec.dat", output_directory_.c_str());
                //output_saver_.WriteText(LLv, name);
            }

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

        observation_manager_->SetTime(model_, model_.GetTime());

        if (!observation_manager_->HasObservation())
        {
            std::cout << "ANALYZE: NO OBSERVATION ------------------" << std::endl;
#if defined(VERDANDI_WITH_MPI)
            if (model_task_ == 0)
#endif
                MessageHandler::Send(*this, "all", "::Analyze end");
            return;
        }

        Nobservation_  = observation_manager_->GetNobservation();


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
                        observation_manager_->GetInnovation(x_col_);
                    Add(To(alpha_), z_col, z);
                    SetRow(z_col, i, Z_i_trans_);
                }
                MltAdd(Ts(alpha_), SeldonTrans, I_trans_, SeldonNoTrans,
                       Z_i_trans_, Ts(0), HL_trans_);
                observation_error_variance R_inv;
                if (observation_error_variance_ == "matrix_inverse")
                    Mlt(HL_trans_, observation_manager_->
                        GetErrorVarianceInverse(), HL_trans_R_);
                else
                {
                    observation_error_variance R_inv;
                    Copy(observation_manager_->GetErrorVariance(), R_inv);
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
            TIC
            // *correction step* ?
            // Computes [HX_{n+1}^{*}].
            sigma_point_matrix Z_i_trans(Nsigma_point_, Nobservation_);
            model_state_error_variance_row x_col;
            Reallocate(x_col, Nstate_, model_);
            model_.StateUpdated();

            observation z(Nobservation_);
            z.Fill(To(0));
            //asumSMat("correction input mat",X_i_);
            for (int i = 0; i < Nsigma_point_; i++)
            {
                GetCol(X_i_, i, x_col);
                observation& z_col =
                    observation_manager_->GetInnovation(x_col);
                Add(To(alpha_), z_col, z);
                SetRow(z_col, i, Z_i_trans);
            }
            //asumSVec("correction accumInnov",z);
            TOCTIC("== an1sx == ");            
            sigma_point_matrix HL_trans(Nreduced_, Nobservation_);
            MltAdd(Ts(alpha_), SeldonTrans, I_trans_, SeldonNoTrans, Z_i_trans,
                   Ts(0), HL_trans);
            //asumSMat("HL_trans", HL_trans);
            //observation_error_variance R_inv;
            sigma_point_matrix working_matrix_po(Nreduced_, Nobservation_),
                tmp;

            if (observation_error_variance_ == "matrix_inverse")
                Mlt(HL_trans, observation_manager_->GetErrorVarianceInverse(),
                    working_matrix_po);
            else
            {
                observation_error_variance R_inv;
                Copy(observation_manager_->GetErrorVariance(), R_inv);
                GetInverse(R_inv);
                Mlt(HL_trans, R_inv, working_matrix_po);
            }
            //TOCTIC("== an2sx == ");
            U_inv_.SetIdentity();
            MltAdd(Ts(1), SeldonNoTrans, working_matrix_po,
                   SeldonTrans, HL_trans, Ts(1), U_inv_);            
            TOCTIC("== an3sx == ");
            GetInverse(U_inv_);
            TOCTIC("== an4sx == (inv) ");            
            tmp.Reallocate(Nreduced_, Nobservation_);
            tmp.Fill(Ts(0));

            observation reduced_innovation(Nreduced_);
            MltAdd(Ts(1), U_inv_, working_matrix_po, Ts(0), tmp);
            MltAdd(Ts(-1), tmp, z, Ts(0), reduced_innovation);
            TOC("== an5sx == ");
            //asumSMat("matUinv", U_inv_);
            //asumSMat("matWorkingPO", working_matrix_po);
            //asumSVec("reduced innovation", reduced_innovation);

            // Updates.
            model_state& x =  model_.GetState();
            MltAdd(Ts(1), model_.GetStateErrorVarianceProjector(),
                   reduced_innovation, Ts(1), x);
            model_.StateUpdated();

            /*std::cout << "New state = " << std::endl;
            for (size_t i = Nstate_-20; i < Nstate_; i++)
                std::cout << x(i) << " ";
            std::cout << std::endl;*/
            asumSVec("############# final state", x);


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
            TIC
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
                    observation_manager_->GetInnovation(x_col);
                Add(To(alpha_), z_col, z);
                //std::cout << "Innovation: " << z_col << std::endl;
                SetRow(z_col, i, Z_i_trans);
                x_col.Nullify();
            }
            TOCTIC("== an1nsx == ");            
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
            TOCTIC("== an2nsx == ");            
            observation_error_variance R_inv;
            if (observation_error_variance_ == "matrix_inverse")
                Mlt(Z_i_trans, observation_manager_->GetErrorVarianceInverse(),
                    working_matrix_ro);
            else
            {
                Copy(observation_manager_->GetErrorVariance(), R_inv);
                GetInverse(R_inv);
                Mlt(Z_i_trans, R_inv, working_matrix_ro);
            }
            TOCTIC("== an3nsx == ");            
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
            TOCTIC("== an4nsx == ");            
            if (alpha_constant_)
            {
                for(int i = 0; i < Nsigma_point_; i++ )
                    working_matrix_rr(i, i) += alpha_;
                MltAdd(Ts(1), D_m, working_matrix_rr, Ts(0),
                       working_matrix_rr2);
                TOCTIC("  == an5nsx a == (multadd) ");
                for(int i = 0; i < Nsigma_point_; i++ )
                    working_matrix_rr2(i, i) += 1;
                GetInverse(working_matrix_rr2);
                TOCTIC("  == an5nsx b == (getInverse) ");
                MltAdd(Ts(1), working_matrix_rr2, D_m, Ts(0),
                       working_matrix_rr);
                TOCTIC("  == an5nsx c == (multadd) ");
                MltAdd(Ts(alpha_), working_matrix_rr, I_trans_, Ts(0),
                       working_matrix_rp);
                TOCTIC("  == an5nsx d == (multadd) ");
                U_.SetIdentity();
                MltAdd(Ts(alpha_), SeldonTrans, I_trans_, SeldonNoTrans,
                       working_matrix_rp, Ts(1), U_);
                TOCTIC("  == an5nsx e == ");

                Copy(U_, U_inv_);
                GetInverse(U_inv_);
                TOCTIC("  == an5nsx f == ");

                /// added P = L Uint Lt;
                /*sigma_point_matrix LU(Nstate_, Nreduced_);
                model_state_error_variance tmp, tmp2, tmp3;

                Copy(model_.GetStateErrorVarianceProjector(), tmp);
                MltAdd(Ts(1), tmp, U_inv_, Ts(0), tmp2);
                MltAdd(Ts(1), SeldonNoTrans, tmp2, SeldonTrans, tmp, Ts(0), tmp3);
                std::cout << "P = " << tmp3 << std::endl;*/

                ///

                // Computes {HL}_{n+1}.
                HL_trans.Reallocate(Nreduced_, Nobservation_);
                working_matrix_rr2.SetIdentity();
                MltAdd(Ts(1), D_v_, working_matrix_rr, Ts(1),
                       working_matrix_rr2);
                TOCTIC("  == an5nsx g == ");

                working_matrix_rr.SetIdentity();
                Add(Ts(alpha_), D_m, working_matrix_rr);
                GetInverse(working_matrix_rr);
                TOCTIC("  == an5nsx h == (getinverse)");

                Mlt(working_matrix_rr, working_matrix_rr2,
                    working_matrix_rr3);
                TOCTIC("  == an5nsx i == (mlt)");

                MltAdd(Ts(alpha_), working_matrix_rr3, I_trans_, Ts(0),
                       working_matrix_rp);
                TOCTIC("  == an5nsx j == (multadd) ");

                MltAdd(Ts(1), SeldonTrans, working_matrix_rp, SeldonNoTrans,
                       Z_i_trans, Ts(0), HL_trans);
                TOCTIC("  == an5nsx k == ");
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
                Mlt(HL_trans, observation_manager_->GetErrorVarianceInverse(),
                    working_matrix_po);
            else
                Mlt(HL_trans, R_inv, working_matrix_po);
            Mlt(U_inv_, working_matrix_po, working_matrix_po2);
            Mlt(model_.GetStateErrorVarianceProjector(),
                working_matrix_po2, K);

            /*{
                char name[100];
                sprintf(name, "out/K_%03d.txt", step);
                std::ofstream f;
                f.open(name);
                f << K << std::endl;
                f.close();
                sprintf(name, "out/z_%03d.txt", step);
                f.open(name);
                f << z << std::endl;
                f.close();
            }*/

            // Updates.
            model_state& x =  model_.GetState();


            MltAdd(Ts(-1), K, z, Ts(1), x);

            /*model_state yy(x.GetM());
            yy.Zero();
            MltAdd(Ts(-1), K, z, Ts(1), yy);
            std::cout << "KalmanGain = \n" << K << std::endl;
            std::cout << "innovation = " << z << std::endl;
            std::cout << "correction = " << yy << std::endl;*/

            model_.StateUpdated();
            /*std::cout << "END ANALYZE:  ";
            for (size_t i = 0; i < 12; i++)
                std::cout << model_.state_(i) << " ";
            std::cout << std::endl;*/
            TOC("== an6nsx == ");            

            //// print error variance projector begin
            if (0) {
                model_state_error_variance& m1 = model_.GetStateErrorVarianceProjector();
                size_t errVarM = model_.GetStateErrorVarianceProjector().GetM();
                size_t errVarN = model_.GetStateErrorVarianceProjector().GetN();
                for (size_t i = 0; i < 6; i++) {
                    for (size_t j = 0; j < errVarN; j++)
                        std::cout << m1(i,j) << " ";
                    std::cout << std::endl;
                }
                std::cout << " ... ANA END ... " << std::endl;
                for (size_t i = errVarM-errVarN; i < errVarM; i++) {
                    for (size_t j = 0; j < errVarN; j++)
                        std::cout << m1(i,j) << " ";
                    std::cout << std::endl;
                }
            }            
            //// print error variance projector end
#endif
        }

#ifndef VERDANDI_WITH_MPI
        MessageHandler::Send(*this, "model", "analysis");
        MessageHandler::Send(*this, "observation_manager", "analysis");
        MessageHandler::Send(*this, "driver", "analysis");
#else
        MessageHandler::Send(*this, "model" + to_str(world_rank_), "analysis");
        MessageHandler::Send(*this, "observation_manager" + to_str(world_rank_), "analysis");
        MessageHandler::Send(*this, "driver" + to_str(world_rank_), "analysis");
#endif
        MessageHandler::Send(*this, "all", "::Analyze end");
        step++;
    }


    //! Finalizes a step for the model.
    template <class Model, class ObservationManager>
    void ReducedOrderUnscentedKalmanFilter<Model, ObservationManager>
    ::FinalizeStep()
    {
        MessageHandler::Send(*this, "all", "::FinalizeStep begin");

        model_.FinalizeStep();

        MessageHandler::Send(*this, "all", "::FinalizeStep end");
    } // FinalizeStep


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
        return *observation_manager_;
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