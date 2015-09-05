----------------------------------- GLOBAL -----------------------------------


Delta_t_model = 0.0015
Nskip_save = 100

output_directory = "result/"
output_file_string = ""
observation_file = output_directory .. "truth-forecast_state.bin"


----------------------------------- MODEL ------------------------------------


dofile("configuration/quadratic_model.lua")

-- In order to demonstrate the assimilation, errors are introduced in the
-- model.
quadratic_model.definition.initial_state = {0., 0.}
quadratic_model.definition.with_linear_term = false


python_model = {

   module = "QuadraticModel",
   directory = "../../model/",
   class_name = "QuadraticModel"

}


-------------------------------- OBSERVATION ---------------------------------


dofile("configuration/observation.lua")


python_observation_manager = {

   module = "LinearObservationManager",
   directory = "../../observation_manager/",
   class_name = "LinearObservationManager"

}


------------------------------ PERTURBATION-----------------------------------


dofile("configuration/perturbation_manager.lua")


----------------------------------- METHOD -----------------------------------


-- Simulation with assimilation using optimal interpolation.
optimal_interpolation = {

   -- Computation mode for BLUE: "vector" or "matrix".
   BLUE_computation = "vector",
   -- Should the diagonal of the analysis variance be computed?
   with_analysis_variance_diagonal = false,

   data_assimilation = {

      analyze_first_step = false,

   },

   display = {

      show_iteration = false,
      show_time = true

   },

   output_saver = {

      variable_list = {"forecast_time", "forecast_state",
                       "analysis_time", "analysis_state"},
      file = output_directory .. "oi-%{name}.%{extension}",
      time = "step " .. Delta_t_model * Nskip_save .. " 1.e-6"

   },

   output = {

      configuration = output_directory .. "oi.lua",
      log = output_directory .. "oi.log"

   }

}


-- Simulation with assimilation using EKF.
extended_kalman_filter = {

   -- Computation mode for BLUE: "vector" or "matrix".
   BLUE_computation = "matrix",
   -- Computation mode for covariance: "vector" or "matrix".
   covariance_computation = "vector",

   data_assimilation = {

      analyze_first_step = false,

   },

   display = {

      show_iteration = false,
      show_time = true
   },

   output_saver = {

      variable_list = {"forecast_time", "forecast_state",
                       "analysis_time", "analysis_state"},
      file = output_directory .. "ekf-%{name}.%{extension}",
      time = "step " .. Delta_t_model * Nskip_save .. " 1.e-6"

   },

   output = {

     configuration = output_directory .. "ekf.lua",
     log = output_directory .. "ekf.log"

  }

}


-- Simulation with assimilation using UKF.
unscented_kalman_filter = {

   data_assimilation = {

      analyze_first_step = false

   },

   sigma_point = {

      -- Choice of sigma-points: "canonical", "star" or "simplex".
      type = "simplex"

   },

   display = {

      show_iteration = false,
      show_time = true

   },

   output_saver = {

      variable_list = {"forecast_time", "forecast_state",
                       "analysis_time", "analysis_state"},
      file = output_directory .. "ukf-%{name}.%{extension}",
      time = "step " .. Delta_t_model * Nskip_save .. " 1.e-6"

   },

   output = {

     configuration = output_directory .. "ukf.lua",
     log = output_directory .. "ukf.log"

  }

}


-- Monte Carlo simulation.
monte_carlo = {

   display = {

      show_iteration = false,
      show_time = false

   },

   output_saver = {
      file_string = output_file_string,
      variable_list = {"perturbation", "forecast_time", "forecast_state"},
      file = output_directory  .. output_file_string
         .. "mc-%{name}.%{extension}",
      time = "step " .. Delta_t_model * Nskip_save .. " 1.e-6"

   },

   output = {

      configuration = output_directory .. "mc.lua",
      log = output_directory .. "mc.log"

   },

}


-- Simulation with assimilation using ensemble Kalman filter.
ensemble_kalman_filter = {

   Nmember = 100,

   -- How the tangent linear operator is accessed: "element" or "matrix".
   observation_tangent_linear_operator_access = "matrix",

   data_assimilation = {

      analyze_first_step = false,

   },

   display = {

      show_iteration = true,
      show_time = false

   },

   output_saver = {

      variable_list = {"forecast_time", "forecast_state",
                       "analysis_time", "analysis_state"},
      file = output_directory .. "enkf-%{name}.%{extension}",
      time = "step " .. Delta_t_model * Nskip_save .. " 1.e-6",

   },

   output = {

     configuration = output_directory .. "enkf.lua",
     log = output_directory .. "enkf.log"

  }

}

for i = 0, 1 do
   table.insert(ensemble_kalman_filter.output_saver.variable_list,
                "forecast_state-" .. i)
   table.insert(ensemble_kalman_filter.output_saver.variable_list,
                "analysis_state-" .. i)
end


-- Forward simulation.
forward = {

   display = {

      show_iteration = false,
      show_time = true

   },

   output_saver = {

      variable_list = {"forecast_time", "forecast_state"},
      file = output_directory .. "forward-%{name}.%{extension}",
      time = "step " .. Delta_t_model * Nskip_save .. " 1.e-6"

   },

   output = {

      configuration = output_directory .. "forward.lua",
      log = output_directory .. "forward.log"

   }

}


-- Hamilton-Jacobi-Bellman.

Delta_t_hjb = 0.0003

hjb = {

   domain = {

      discretization = {-1., 0.01, 201,
                        -1., 0.01, 201},
      initial_time = 0.,
      Delta_t = Delta_t_hjb,
      Nt = math.floor(0.1 / Delta_t_hjb)

   },

   equation_coefficient = {

      with_quadratic_term = true,
      with_advection_term = true,
      with_source_term = true,

      -- Is the model time-dependent?
      model_time_dependent = false,

      Q_0 = {1., 0.,
             0., 1.},

      -- Position of the minimum of the initial parabola.
      x_0 = {0., 0.},

      Q = {1., 0.,
           0., 1.},

      R = {10., 0.,
           0., 10.}

   },

   solver = {

      -- Numerical scheme: LxF (for first-order Lax-Friedrichs), BrysonLevy
      -- (second-order central scheme) and Godunov.
      scheme = "Godunov"

   },

   boundary_condition = {

      -- "Dirichlet", "Extrapolation" and "Periodic" are supported.
      type = "Extrapolation",
      -- Constant value imposed outside the domain.
      value = 2.

   },

   lax_friedrichs = {

      -- Is the model time-dependent?
      model_time_dependent = false,
      -- Upper bound on the absolute value of the model operator, in every
      -- direction.
      Upper_bound_model = {150., 150.}

   },

   display = {

      show_iteration = false,
      show_time = true

   },

   output_saver = {

      variable_list = {"time_step", "time", "value_function"},
      file = output_directory .. "hjb-%{name}.%{extension}",
      time = "step " .. Delta_t_hjb * 50 .. " 1.e-8"

   }

}
