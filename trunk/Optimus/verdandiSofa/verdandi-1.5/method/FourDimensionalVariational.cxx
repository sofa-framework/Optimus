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


#ifndef VERDANDI_FILE_METHOD_FOURDIMENSIONALVARIATIONAL_CXX

#include "FourDimensionalVariational.hxx"

#include "TrajectoryManager.cxx"

#include "seldon/vector/VectorCollection.cxx"


namespace Verdandi
{


    ////////////////////////////////
    // CONSTRUCTOR AND DESTRUCTOR //
    ////////////////////////////////


    //! Main constructor.
    /*! Builds the driver and reads option keys in the configuration file.
      \param[in] configuration_file configuration file.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    FourDimensionalVariational<Model, ObservationManager,  Optimization>
    ::FourDimensionalVariational()
    {

        /*** Initializations ***/

        MessageHandler::AddRecipient("model", model_,
                                     Model::StaticMessage);
        MessageHandler::AddRecipient("observation_manager",
                                     observation_manager_,
                                     ObservationManager::StaticMessage);
        MessageHandler::AddRecipient("driver", *this,
                                     FourDimensionalVariational
                                     ::StaticMessage);
    }


    //! Destructor.
    template <class Model, class ObservationManager,
              class Optimization>
    FourDimensionalVariational<Model, ObservationManager,  Optimization>
    ::~FourDimensionalVariational()
    {
    }


    /////////////
    // METHODS //
    /////////////


    //! Initializes the driver.
    /*! Initializes the model and the observation manager. Optionally computes
      the analysis of the first step. */
    template <class Model, class ObservationManager,
              class Optimization>
    void FourDimensionalVariational<
                                    Model, ObservationManager, Optimization>
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
    template <class Model, class ObservationManager,
              class Optimization>
    void FourDimensionalVariational<
                                    Model, ObservationManager, Optimization>
    ::Initialize(VerdandiOps& configuration,
                 bool initialize_model, bool initialize_observation_manager)
    {


        /***************************
         * Reads the configuration *
         ***************************/


        configuration_file_ = configuration.GetFilePath();
        configuration.SetPrefix("four_dimensional_variational.");

        /*** Model ***/

        configuration.Set("model.configuration_file", "", configuration_file_,
                          model_configuration_file_);

        /*** Observation manager ***/

        configuration.Set("observation_manager.configuration_file", "",
                          configuration_file_,
                          observation_configuration_file_);

        /*** Display options ***/

        // Should optimization iterations be displayed on screen?
        configuration.Set("display.show_optimization_iteration",
                          option_display_["show_optimization_iteration"]);
        // Should optimized parameters be displayed on screen?
        configuration.Set("display.show_optimized_parameter",
                          option_display_["show_optimized_parameter"]);
        // Should iterations be displayed on screen?
        configuration.Set("display.show_iteration",
                          option_display_["show_iteration"]);
        // Should current time be displayed on screen?
        configuration.Set("display.show_time", option_display_["show_time"]);

        /*** Ouput saver ***/

        configuration.SetPrefix("four_dimensional_variational"
                                ".output_saver.");
        output_saver_.Initialize(configuration);
        output_saver_.Empty("forecast_time");
        output_saver_.Empty("forecast_state");
        output_saver_.Empty("analysis_time");
        output_saver_.Empty("analysis_state");

        /*** Logger and read configuration ***/

        configuration.SetPrefix("four_dimensional_variational.");

        if (configuration.Exists("output.log"))
            Logger::SetFileName(configuration.Get<string>("output.log"));

        if (configuration.Exists("output.configuration"))
        {
            string output_configuration;
            configuration.Set("output.configuration",
                              output_configuration);
            configuration.WriteLuaDefinition(output_configuration);
        }

        Ncall_cost_ = 0;

        /*** Model and observation manager initialization ***/

        if (initialize_model)
            model_.Initialize(model_configuration_file_);
        initial_time_ = model_.GetTime();
        if (initialize_observation_manager)
        {
            observation_manager_.Initialize(model_,
                                            observation_configuration_file_);
            observation_manager_.DiscardObservation(false);
        }
        Nstate_ = model_.GetNstate();
        Nobservation_  = observation_manager_.GetNobservation();

        /*** Optimization initialization ***/

        configuration.SetPrefix("four_dimensional_variational.nlopt.");
        string algorithm;
        configuration.Set("algorithm", algorithm);
        double parameter_tolerance, cost_function_tolerance;
        configuration.Set("parameter_tolerance", parameter_tolerance);
        configuration.Set("cost_function_tolerance", cost_function_tolerance);
        int Niteration_max;
        configuration.Set("Niteration_max", Niteration_max);
        optimization_.Initialize(Nstate_, algorithm, parameter_tolerance,
                                 cost_function_tolerance, Niteration_max);

#ifdef VERDANDI_WITH_TRAJECTORY_MANAGER

        /*** Trajectory manager initialization ***/

        configuration.SetPrefix("four_dimensional_variational."
                                "trajectory_manager.");
        string checkpoint_recording_mode;
        configuration.Set("checkpoint_recording_mode",
                          checkpoint_recording_mode);
        string checkpoint_recording_file;
        configuration.Set("checkpoint_recording_file",
                          checkpoint_recording_file);
        string trajectory_recording_mode;
        configuration.Set("trajectory_recording_mode",
                          trajectory_recording_mode);
        string trajectory_recording_file;
        configuration.Set("trajectory_recording_file",
                          trajectory_recording_file);
        int Nskip_step;
        configuration.Set("Nskip_step", Nskip_step);
        trajectory_manager_.Initialize(checkpoint_recording_mode,
                                       checkpoint_recording_file,
                                       trajectory_recording_mode,
                                       trajectory_recording_file,
                                       Nskip_step);
#endif

        optimization_.SetLowerBound(model_.GetStateLowerBound());
        optimization_.SetUpperBound(model_.GetStateUpperBound());

        state_first_guess_.Reallocate(model_.GetNstate());
        Copy(model_.GetState(), state_first_guess_);

        if (initialize_model)
        {
            MessageHandler::Send(*this, "model", "initial condition");
            MessageHandler::Send(*this, "driver", "initial condition");
        }

        configuration.SetPrefix("four_dimensional_variational.");

        configuration.Set("observation_tangent_linear_operator_access",
                          "ops_in(v, {'element', 'matrix'})",
                          observation_tangent_linear_operator_access_);
    }


    //! Initializes a step for the extended Kalman filter.
    /*! Initializes a step for the model.
     */
    template <class Model, class ObservationManager,
              class Optimization>
    void FourDimensionalVariational<Model, ObservationManager,
                                    Optimization>::InitializeStep()
    {
        model_.InitializeStep();
    }


    //! Performs a step forward, with optimal interpolation at the end.
    template <class Model, class ObservationManager,
              class Optimization>
    void FourDimensionalVariational<Model, ObservationManager,
                                    Optimization>::Forward()
    {
        MessageHandler::Send(*this, "all", "::Forward begin");

        model_.Forward();

        MessageHandler::Send(*this, "model", "forecast");
        MessageHandler::Send(*this, "observation_manager", "forecast");
        MessageHandler::Send(*this, "driver", "forecast");

        MessageHandler::Send(*this, "all", "::Forward end");
    }


    //! Computes an analysis.
    template <class Model, class ObservationManager,
              class Optimization>
    void FourDimensionalVariational<Model, ObservationManager,
                                    Optimization>::Analyze()
    {
        MessageHandler::Send(*this, "all", "::Analyze begin");

        model_state& x = model_.GetState();;
        optimization_.SetParameter(x);
        Ncall_cost_ = 0;
        optimization_.Optimize(StaticCost,
                               reinterpret_cast<void*>(this));
        optimization_.GetParameter(x);
        model_.StateUpdated();
        model_.SetTime(initial_time_);

        MessageHandler::Send(*this, "model", "analysis");
        MessageHandler::Send(*this, "observation_manager", "analysis");
        MessageHandler::Send(*this, "driver", "analysis");

        MessageHandler::Send(*this, "all", "::Analyze end");
    }


    //! Finalizes a step for the model.
    template <class Model, class ObservationManager,
              class Optimization>
    void FourDimensionalVariational<Model, ObservationManager,
                                    Optimization>::FinalizeStep()
    {
        MessageHandler::Send(*this, "all", "::FinalizeStep begin");

        model_.FinalizeStep();

        MessageHandler::Send(*this, "all", "::FinalizeStep end");
    }


    //! Finalizes the model.
    template <class Model, class ObservationManager,
              class Optimization>
    void FourDimensionalVariational<Model, ObservationManager,
                                    Optimization>::Finalize()
    {
        MessageHandler::Send(*this, "all", "::Finalize begin");

        model_.Finalize();

        MessageHandler::Send(*this, "all", "::Finalize end");
    }


    //! Checks whether the model has finished.
    /*!
      \return True if no more data assimilation is required, false otherwise.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    bool FourDimensionalVariational<Model, ObservationManager,
                                    Optimization>::HasFinished()
    {
        return model_.HasFinished();
    }


    //! Returns the model.
    /*!
      \return The model.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    Model&
    FourDimensionalVariational<Model, ObservationManager,  Optimization>
    ::GetModel()
    {
        return model_;
    }


    //! Returns the observation manager.
    /*!
      \return The observation manager.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    ObservationManager&
    FourDimensionalVariational<Model, ObservationManager, Optimization>
    ::GetObservationManager()
    {
        return observation_manager_;
    }


    //! Returns the output saver.
    /*!
      \return The output saver.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    OutputSaver&
    FourDimensionalVariational<Model, ObservationManager, Optimization>
    ::GetOutputSaver()
    {
        return output_saver_;
    }


    //! Returns the name of the class.
    /*!
      \return The name of the class.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    string
    FourDimensionalVariational<Model, ObservationManager,  Optimization>
    ::GetName() const
    {
        return "FourDimensionalVariational";
    }


    //! Receives and handles a message.
    /*
      \param[in] message the received message.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    void FourDimensionalVariational<Model, ObservationManager,
                                    Optimization>::Message(string message)
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


    //! Cost function.
    /*
      \param[in] x vector that stores parameters values.
      \param[in, out] gradient vector that stores gradient values, on final
      exit, it returns the gradient vector for optimized parameters.
      \return the cost value at x.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    typename FourDimensionalVariational<Model, ObservationManager,
    Optimization>::Ts
    FourDimensionalVariational<Model, ObservationManager,  Optimization>
    ::Cost(const model_state& x, model_state& gradient)
    {
        Vector<double> time;
        if (option_display_["show_optimization_iteration"])
            Logger::StdOut(*this,
                           "Optimization iteration: " + to_str(Ncall_cost_));
        if (option_display_["show_optimized_parameter"])
            Logger::StdOut(*this,
                           "Optimized parameter: " + to_str(x));
        Ncall_cost_++;
        model_state delta(x);

        bool with_gradient = gradient.GetM() != 0;

        /*** Background contribution ***/

        model_state x_b(state_first_guess_);
        Add(Ts(-1), x_b, delta);

        model_state_error_variance B_inv(model_.GetStateErrorVariance());
        GetInverse(B_inv);
        MltAdd(Ts(1), B_inv, delta, Ts(0), x_b);

        Ts cost_background;
        cost_background = DotProd(delta, x_b);

        /*** Observation contribution ***/

#ifndef VERDANDI_WITH_TRAJECTORY_MANAGER

        vector<model_state> trajectory;

        Ts cost_observation(0);
        Copy(x, model_.GetState());
        model_.StateUpdated();
        model_.SetTime(initial_time_);
        while (!model_.HasFinished())
        {
            model_state& state = model_.GetState();
            observation y, Rinv_y;
            observation_manager_.SetTime(model_, model_.GetTime());
            if (observation_manager_.HasObservation())
            {
                observation& y = observation_manager_.GetInnovation(state);
                Nobservation_ = y.GetSize();
                Rinv_y.Reallocate(Nobservation_);
                observation_error_variance
                    Rinv = observation_manager_.GetErrorVarianceInverse();
                MltAdd(Ts(1), Rinv, y, Ts(0), Rinv_y);
                cost_observation += DotProd(y, Rinv_y);
            }

            time.PushBack(model_.GetTime());
            trajectory.push_back(state);

            model_.InitializeStep();
            model_.Forward();
        }

        if (!with_gradient)
            return Ts(0.5) * (cost_background + cost_observation
                              + model_.GetAdditionalCostTerm());

        /*** Backward loop ***/

        model_state adjoint_source(Nstate_);
        adjoint_source.Fill(Ts(0));
        model_.GetAdjointState().Fill(Ts(0));
        model_.AdjointStateUpdated();
        for (int t = time.GetM() - 1; t >= 0; t--)
        {
            model_.SetTime(time(t));
            observation Rinv_y;
            observation_manager_.SetTime(model_, model_.GetTime());
            if (observation_manager_.HasObservation())
            {
                observation& y =
                    observation_manager_.GetInnovation(trajectory[t]);
                Nobservation_ = y.GetSize();
                Rinv_y.Reallocate(Nobservation_);
                observation_error_variance
                    Rinv(observation_manager_.GetErrorVarianceInverse());

                MltAdd(Ts(1), Rinv, y, Ts(0), Rinv_y);

                if (observation_tangent_linear_operator_access_ == "matrix")
                    MltAdd(Ts(1), SeldonTrans, observation_manager_.
                           GetTangentLinearOperator(), Rinv_y, Ts(0),
                           adjoint_source);
                else // "element".
                {
                    adjoint_source.Fill(Ts(0));
                    for (int i = 0; i < Nstate_; i++)
                        for (int j = 0; j < Nobservation_; j++)
                            adjoint_source(i) +=
                                observation_manager_.
                                GetTangentLinearOperator(j, i) * Rinv_y(j);
                }
            }
            else
                adjoint_source.Fill(Ts(0));

            Copy(trajectory[t], model_.GetState());
            model_.StateUpdated();

            model_.InitializeStep();
            model_.BackwardAdjoint(adjoint_source);
        }

        Copy(model_.GetAdjointState(), gradient);
        Add(Ts(1), model_.GetAdditionalAdjointTerm(), gradient)
        Mlt(Ts(-1), gradient);
        Add(Ts(1), x_b, gradient);

        return Ts(0.5) * (cost_background + cost_observation +
                          model_.GetAdditionalCostTerm());

#else

        Ts cost_observation(0);
        Copy(x, model_.GetState());
        model_.StateUpdated();
        model_.SetTime(initial_time_);
        while (!model_.HasFinished())
        {
            model_state& state = model_.GetState();
            observation Rinv_y;
            observation_manager_.SetTime(model_, model_.GetTime());
            if (observation_manager_.HasObservation())
            {
                observation& y = observation_manager_.GetInnovation(state);
                Nobservation_ = y.GetSize();
                Rinv_y.Reallocate(Nobservation_);
                observation_error_variance
                    Rinv = observation_manager_.GetErrorVarianceInverse();
                MltAdd(Ts(1), Rinv, y, Ts(0), Rinv_y);
                cost_observation += DotProd(y, Rinv_y);
            }

            time.PushBack(model_.GetTime());
            trajectory_manager_.Save(state, model_.GetTime());

            model_.InitializeStep();
            model_.Forward();
        }

        if (!with_gradient)
            return Ts(0.5) * (cost_background + cost_observation +
                              model_.GetAdditionalCostTerm());

        /*** Backward loop ***/

        model_state adjoint_source(Nstate_);
        adjoint_source.Fill(Ts(0));
        model_.GetAdjointState().Fill(Ts(0));
        model_.AdjointStateUpdated();
        for (int t = time.GetM() - 1; t >= 0; t--)
        {
            model_.SetTime(time(t));
            trajectory_manager_.SetTime(model_, model_.GetTime());
            observation Rinv_y;
            observation_manager_.SetTime(model_, model_.GetTime());
            if (observation_manager_.HasObservation())
            {
                observation& y = observation_manager_.
                    GetInnovation(trajectory_manager_.GetState());
                Nobservation_ = y.GetSize();
                Rinv_y.Reallocate(Nobservation_);
                observation_error_variance
                    Rinv(observation_manager_.GetErrorVarianceInverse());

                MltAdd(Ts(1), Rinv, y, Ts(0), Rinv_y);

                if (observation_tangent_linear_operator_access_ == "matrix")
                    MltAdd(Ts(1), SeldonTrans, observation_manager_.
                           GetTangentLinearOperator(), Rinv_y, Ts(0),
                           adjoint_source);
                else // "element".
                {
                    adjoint_source.Fill(Ts(0));
                    for (int i = 0; i < Nstate_; i++)
                        for (int j = 0; j < Nobservation_; j++)
                            adjoint_source(i) +=
                                observation_manager_.
                                GetTangentLinearOperator(j, i) * Rinv_y(j);
                }
            }
            else
                adjoint_source.Fill(Ts(0));

            Copy(trajectory_manager_.GetState(), model_.GetState());
            model_.StateUpdated();

            model_.InitializeStep();
            model_.BackwardAdjoint(adjoint_source);
        }

        Copy(model_.GetAdjointState(), gradient);
        Add(Ts(1), model_.GetAdditionalAdjointTerm(), gradient);
        Mlt(Ts(-1), gradient);
        Add(Ts(1), x_b, gradient);
        trajectory_manager_.Deallocate();

        return Ts(0.5) * (cost_background + cost_observation +
                          model_.GetAdditionalCostTerm());

#endif
    }


    //! Constraint function.
    /*
      \param[in] x vector that stores constraints parameters values.
      \param[in] gradient vector that stores constaint gradient values.
      \return the constraint value at x.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    typename FourDimensionalVariational<Model, ObservationManager,
    Optimization>::Ts
    FourDimensionalVariational<Model, ObservationManager,  Optimization>
    ::Constraint(const model_state& x, model_state& gradient)
    {
        throw ErrorUndefined("FourDimensionalVariational::Constraint");
    }


    //! Sets initial time.
    /*
      \param[in] time the initial time value to be set.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    void FourDimensionalVariational<Model, ObservationManager,
                                    Optimization>
    ::SetInitialTime(double time)
    {
        initial_time_ = time;
    }


    //! Static cost function.
    /*
      \param[in] x vector that stores parameters values.
      \param[in,out] gradient vector that stores gradient values, on final
      exit, it returns the gradient vector for optimized parameters.
      \param[in] parameter the current FourDimensionalVariational<Model,
      ObservationManager,  Optimization> object.
      \return the cost value at x.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    typename FourDimensionalVariational<Model, ObservationManager,
    Optimization>::Ts
    FourDimensionalVariational<Model, ObservationManager,  Optimization>
    ::StaticCost(const model_state& x, model_state& gradient,
                 void* this_object)
    {
        return reinterpret_cast<FourDimensionalVariational<Model,
            ObservationManager, Optimization>* >(this_object)
            ->Cost(x, gradient);
    }


    //! Constraints.
    /*
      \param[in] x vector that stores constraints parameters values.
      \param[in] gradient vector that stores constaint gradient values.
      \param[in] parameter the current FourDimensionalVariational<Model,
      ObservationManager,  Optimization> object.
      \return the constraint value at x.
    */
    template <class Model, class ObservationManager,
              class Optimization>
    typename FourDimensionalVariational<Model, ObservationManager,
    Optimization>::Ts
    FourDimensionalVariational<Model, ObservationManager,  Optimization>
    ::StaticConstraint(const model_state& x, model_state& gradient,
                       void* parameter)
    {
        throw ErrorUndefined("FourDimensionalVariational::StaticConstraint");
    }



} // namespace Verdandi.


#define VERDANDI_FILE_METHOD_FOURDIMENSIONALVARIATIONAL_CXX
#endif
