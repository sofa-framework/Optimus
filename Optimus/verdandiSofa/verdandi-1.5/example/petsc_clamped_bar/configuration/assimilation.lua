----------------------------------- GLOBAL -----------------------------------


Delta_t_petsc_clamped_bar = 0.01
final_time_petsc_clamped_bar = 10
-- Saving period.
Nskip_save = 1

output_mode = "text"
output_directory = "result/"
output_mode_scalar = "text"

observation_file = output_directory .. "truth-forecast_state.bin"


----------------------------------- MODEL ------------------------------------


dofile("configuration/petsc_clamped_bar.lua")

-- In order to demonstrate the assimilation, errors are introduced in the
-- model.
petsc_clamped_bar.physics.theta_force = {1., 1.}


-------------------------------- OBSERVATION ---------------------------------


dofile("configuration/observation.lua")


----------------------------------- METHOD -----------------------------------


-- Simulation with assimilation using ROUKF.
reduced_order_unscented_kalman_filter = {

   data_assimilation = {

      analyze_first_step = false,
      with_resampling = false,
      -- Indicates how R is stored: "matrix", "matrix_inverse".
      observation_error_variance = "matrix_inverse",

   },

   sigma_point = {

      -- Choice of sigma-points: "canonical", "star" or "simplex".
      type = "simplex"

   },

   display = {

      show_iteration = false,
      show_time = true,
      show_mpi_grid = true

   },

   output_saver = {

      variable_list = {"forecast_time", "forecast_state",
                       "analysis_time", "analysis_state"},
      file = output_directory .. "roukf-%{name}.%{extension}",
      time = "step " .. Delta_t_petsc_clamped_bar * Nskip_save .. " 1.e-6",
      mode = output_mode,
      mode_scalar = output_mode_scalar

   },

   output = {

     configuration = output_directory .. "roukf.lua",
     log = output_directory .. "roukf_%{rank}.log"

   },

   mpi_grid = {

      -- The number of processes for each model task.
      Nrow = 2,
      -- The number of model tasks.
      Ncol = 3
   }

}


-- Forward simulation.
forward = {

   display = {

      show_iteration = false,
      show_time = true,
      show_mpi_grid = true

   },

   output_saver = {

      variable_list = {"forecast_time", "forecast_state"},
      file = output_directory .. "forward-%{name}.%{extension}",
      time = "step " .. Delta_t_petsc_clamped_bar * Nskip_save .. " 1.e-6",
      mode = output_mode,
      mode_scalar = output_mode_scalar

   },

   output = {

      configuration = output_directory .. "forward.lua",
      log = output_directory .. "forward.log"

   },

   mpi_grid = {

      -- The number of processes for each model task.
      Nrow = 3,
      -- The number of model tasks.
      Ncol = 1
   }

}
