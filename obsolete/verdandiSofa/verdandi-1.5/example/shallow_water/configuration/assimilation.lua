----------------------------------- GLOBAL -----------------------------------


Delta_t_shallow_water = 0.03
final_time_shallow_water = 1500 * Delta_t_shallow_water
-- Saving period.
Nskip_save = 10

output_directory = "result/"
output_mode = "binary"
output_mode_scalar = "text"

observation_file = output_directory .. "truth-forecast_state.bin"


----------------------------------- MODEL ------------------------------------


dofile("configuration/shallow_water.lua")

-- The configuration of the model is modified.
shallow_water.initial_condition.value = 1.
shallow_water.error.standard_deviation_bc = 0.


-------------------------------- OBSERVATION ---------------------------------


dofile("configuration/observation.lua")


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

      analyze_first_step = true,

   },

   display = {

      show_iteration = false,
      show_time = false
   },

   output_saver = {

      variable_list = {"forecast_time", "forecast_state",
                       "analysis_time", "analysis_state"},
      file = output_directory .. "oi-%{name}.%{extension}",
      time = "step " .. Delta_t_shallow_water * Nskip_save .. " 1.e-6",
      mode = output_mode,
      mode_scalar = output_mode_scalar

   },

   output = {

      configuration = output_directory .. "oi.lua",
      log = output_directory .. "oi.log"

   }

}


-- Simulation with assimilation using ensemble Kalman filter.
ensemble_kalman_filter = {

   Nmember = 10,

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
      time = "step " .. Delta_t_shallow_water * Nskip_save .. " 1.e-6",

   },

   output = {

     configuration = output_directory .. "enkf.lua",
     log = output_directory .. "enkf.log"

  }
}

for i = 0, ensemble_kalman_filter.Nmember do
   table.insert(ensemble_kalman_filter.output_saver.variable_list,
                "forecast_state-" .. i)
   table.insert(ensemble_kalman_filter.output_saver.variable_list,
                "analysis_state-" .. i)
end


-- Forward simulation.
forward = {

   display = {

      show_iteration = false,
      show_time = false

   },

   output_saver = {

      variable_list = {"forecast_time", "forecast_state"},
      file = output_directory .. "forward-%{name}.%{extension}",
      time = "step " .. Delta_t_shallow_water * Nskip_save .. " 1.e-6",
      mode = output_mode,
      mode_scalar = output_mode_scalar

   },

   output = {

      configuration = output_directory .. "forward.lua",
      log = output_directory .. "forward.log"

   }
}
