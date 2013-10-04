// Copyright (C) 2012 INRIA
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


#ifndef VERDANDI_FILE_METHOD_OBSERVATIONGENERATOR_CXX


#include "ObservationGenerator.hxx"


namespace Verdandi
{


    ////////////////////////////////
    // CONSTRUCTOR AND DESTRUCTOR //
    ////////////////////////////////


    //! Main constructor.
    /*! Builds the driver and reads option keys in the configuration file.
      \param[in] configuration configuration file.
    */
    template <class Model, class ObservationManager>
    ObservationGenerator<Model, ObservationManager>::ObservationGenerator():
        iteration_(-1)
    {

        /*** Initializations ***/

        MessageHandler::AddRecipient("model", model_, Model::StaticMessage);
        MessageHandler::AddRecipient("observation_manager",
                                     observation_manager_,
                                     ObservationManager::StaticMessage);
        MessageHandler::AddRecipient("driver", *this,
                                     ObservationGenerator::StaticMessage);
    }


    //! Destructor.
    template <class Model, class ObservationManager>
    ObservationGenerator<Model, ObservationManager>::~ObservationGenerator()
    {
    }


    /////////////
    // METHODS //
    /////////////


    //! Initializes the simulation.
    /*! Initializes the model.
      \param[in] configuration_file configuration file to be given to the
      model initialization method.
    */
    template <class Model, class ObservationManager>
    void ObservationGenerator<Model, ObservationManager>
    ::Initialize(string configuration_file, bool initialize_model,
                 bool initialize_observation_manager)
    {
        VerdandiOps configuration(configuration_file);
        Initialize(configuration, initialize_model);
    }


    //! Initializes the simulation.
    /*! Initializes the model.
      \param[in] configuration configuration file to be given to the
      model initialization method.
    */
    template <class Model, class ObservationManager>
    void ObservationGenerator<Model, ObservationManager>
    ::Initialize(VerdandiOps& configuration, bool initialize_model,
                 bool initialize_observation_manager)
    {
        MessageHandler::Send(*this, "all", "::Initialize begin");

        configuration_file_ = configuration.GetFilePath();
        configuration.SetPrefix("observation_generator.");

#ifdef VERDANDI_WITH_MPI
        MPI_Comm_rank(MPI_COMM_WORLD, &world_rank_);
        MPI_Comm_size(MPI_COMM_WORLD, &Nprocess_);
        configuration.Set("mpi_grid.Nrow", Nrow_);
        configuration.Set("mpi_grid.Ncol", Ncol_);

        if (Nprocess_ != Nrow_ * Ncol_)
            throw ErrorConfiguration("ObservationGenerator<Model>::Initialize"
                                     , "Wrong number of processes: " +
                                     to_str(Nprocess_)
                                     + ". The dimension of the MPI grid ("
                                     + to_str(Nrow_) + ", " + to_str(Ncol_) +
                                     ") requires " + to_str(Nrow_ * Ncol_)
                                     + " processes.");

        SetGridCommunicator(Nrow_, Ncol_, &row_communicator_,
                            &col_communicator_);
        MPI_Comm_rank(row_communicator_, &model_task_);
#endif


        /*********
         * Model *
         *********/


        configuration.Set("model.configuration_file", "",
                          configuration_file_,
                          model_configuration_file_);
        if (initialize_model)
        {
#ifdef VERDANDI_WITH_MPI
            model_.SetMPICommunicator(col_communicator_);
#endif
            model_.Initialize(model_configuration_file_);
        }


        configuration.Set("observation_manager.configuration_file", "",
                          configuration_file_,
                          observation_configuration_file_);
        if (initialize_observation_manager)
        {
#ifdef VERDANDI_WITH_MPI
            observation_manager_.SetMPICommunicator(col_communicator_);
#endif

            observation_manager_.Initialize(model_,
                                            observation_configuration_file_);
        }

        /***************************
         * Reads the configuration *
         ***************************/


        /*** Display options ***/

        // Should the iteration be displayed on screen?
        configuration.Set("display.show_iteration", show_iteration_);
        // Should the time be displayed on screen?
        configuration.Set("display.show_time", show_time_);
#ifdef VERDANDI_WITH_MPI
        // Should the MPI grid be displayed on screen?
        configuration.Set("display.show_mpi_grid", show_mpi_grid_);
#endif

        if (show_iteration_)
            Logger::StdOut(*this, "Initialization");
        else
            Logger::Log<-3>(*this, "Initialization");

        iteration_ = 0;

#ifdef VERDANDI_WITH_MPI
        if (world_rank_ == 0)
        {
            if (show_mpi_grid_)
                Logger::StdOut(*this, "world rank\tmodel task\tmodel rank");
            else
                Logger::Log<-3>(*this,
                                "world rank\tmodel task\tmodel rank");
        }

        MPI_Barrier(MPI_COMM_WORLD);
        int model_rank;
        MPI_Comm_rank(col_communicator_, &model_rank);
        if (show_mpi_grid_)
            Logger::StdOut(*this, to_str(world_rank_) + "\t\t"
                           + to_str(model_task_) + "\t\t" +
                           to_str(model_rank));
        else
            Logger::Log<-3>(*this, to_str(world_rank_) + "\t\t"
                            + to_str(model_task_) + "\t\t" +
                            to_str(model_rank));
#endif


        /*** Ouput saver ***/

        configuration.SetPrefix("observation_generator.output_saver.");
        output_saver_.Initialize(configuration);
        output_saver_.Empty("forecast_state");
        output_saver_.Empty("observation_time");
        output_saver_.Empty("observation");
        configuration.SetPrefix("observation_generator.");

        /*** Logger and read configuration ***/

        if (configuration.Exists("output.log"))
            Logger::SetFileName(configuration.Get<string>("output.log"));

        if (configuration.Exists("output.configuration"))
        {
            string output_configuration;
            configuration.Set("output.configuration", output_configuration);
            configuration.WriteLuaDefinition(output_configuration);
        }

        if (initialize_model)
        {
            MessageHandler::Send(*this, "model", "initial condition");
            MessageHandler::Send(*this, "driver", "initial condition");
        }

        MessageHandler::Send(*this, "all", "::Initialize end");
    }


    //! Initializes the model before a time step.
    template <class Model, class ObservationManager>
    void ObservationGenerator<Model, ObservationManager>::InitializeStep()
    {
        MessageHandler::Send(*this, "all", "::InitializeStep begin");

        model_.InitializeStep();

        MessageHandler::Send(*this, "all", "::InitializeStep end");
    }


    //! Performs a step forward without optimal interpolation.
    template <class Model, class ObservationManager>
    void ObservationGenerator<Model, ObservationManager>::Forward()
    {
        time_.PushBack(model_.GetTime());

        MessageHandler::Send(*this, "all", "::Forward begin");

#ifdef VERDANDI_WITH_MPI
        if (world_rank_ == 0)
        {
#endif
            if (show_time_)
                Logger::StdOut(*this, "Time: " + to_str(model_.GetTime()));
            else
                Logger::Log<-3>(*this,
                                "Time: " + to_str(model_.GetTime()));
            if (show_iteration_)
                Logger::StdOut(*this, "Iteration " + to_str(iteration_)
                               + " -> " + to_str(iteration_ + 1));
            else
                Logger::Log<-3>(*this, "Iteration " + to_str(iteration_)
                                + " -> " + to_str(iteration_ + 1));
#ifdef VERDANDI_WITH_MPI
        }
#endif

        model_.Forward();
        iteration_++;

        MessageHandler::Send(*this, "model", "forecast");
        MessageHandler::Send(*this, "driver", "forecast");

        MessageHandler::Send(*this, "all", "::Forward end");
    }


    //! Finalizes a step for the model.
    template <class Model, class ObservationManager>
    void ObservationGenerator<Model, ObservationManager>::FinalizeStep()
    {
        MessageHandler::Send(*this, "all", "::FinalizeStep begin");

        model_.FinalizeStep();

        MessageHandler::Send(*this, "model", "final condition");
        MessageHandler::Send(*this, "driver", "final condition");

        MessageHandler::Send(*this, "all", "::FinalizeStep end");
    }


    //! Finalizes the model.
    template <class Model, class ObservationManager>
    void ObservationGenerator<Model, ObservationManager>::Finalize()
    {
        MessageHandler::Send(*this, "all", "::Finalize begin");

        model_.Finalize();

        MessageHandler::Send(*this, "model", "final condition");
        MessageHandler::Send(*this, "driver", "final condition");

        MessageHandler::Send(*this, "all", "::Finalize end");
    }


    //! Checks whether the model has finished.
    /*!
      \return True if no more data assimilation is required, false otherwise.
    */
    template <class Model, class ObservationManager>
    bool ObservationGenerator<Model, ObservationManager>::HasFinished()
    {
        return model_.HasFinished();
    }


    //! Returns the model.
    /*!
      \return The model.
    */
    template <class Model, class ObservationManager>
    Model& ObservationGenerator<Model, ObservationManager>::GetModel()
    {
        return model_;
    }


    //! Returns the output saver.
    /*!
      \return The output saver.
    */
    template <class Model, class ObservationManager>
    OutputSaver&
    ObservationGenerator<Model, ObservationManager>::GetOutputSaver()
    {
        return output_saver_;
    }


    //! Returns the name of the class.
    /*!
      \return The name of the class.
    */
    template <class Model, class ObservationManager>
    string ObservationGenerator<Model, ObservationManager>::GetName() const
    {
        return "ObservationGenerator";
    }


    //! Receives and handles a message.
    /*
      \param[in] message the received message.
    */
    template <class Model, class ObservationManager>
    void  ObservationGenerator<Model, ObservationManager>
    ::Message(string message)
    {
        if (message.find("initial condition") != string::npos)
        {
            output_saver_.Save(model_.GetTime(), model_.GetTime(),
                               "observation_time");
            output_saver_.Save(model_.GetState(), model_.GetTime(),
                               "forecast_state");
            observation y(observation_manager_.GetNobservation());
            observation_manager_.ApplyOperator(model_.GetState(), y);
            output_saver_.Save(y, model_.GetTime(), "observation");
        }
        if (message.find("forecast") != string::npos)
        {
            output_saver_.Save(model_.GetTime(), model_.GetTime(),
                               "observation_time");
            output_saver_.Save(model_.GetState(), model_.GetTime(),
                               "forecast_state");
            observation y(observation_manager_.GetNobservation());
            observation_manager_.ApplyOperator(model_.GetState(), y);
            output_saver_.Save(y, model_.GetTime(), "observation");
        }
    }


} // namespace Verdandi.


#define VERDANDI_FILE_METHOD_OBSERVATIONGENERATOR_CXX
#endif
