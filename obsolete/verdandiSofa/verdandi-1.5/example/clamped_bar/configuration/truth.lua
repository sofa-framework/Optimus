----------------------------------- GLOBAL -----------------------------------


Delta_t_clamped_bar = 0.01
final_time_clamped_bar = 10.0
-- Saving period.
Nskip_save = 1

output_directory = "result/"
output_mode = "binary"
output_mode_scalar = "text"

observation_file = output_directory .. "truth-observation.bin"


----------------------------------- MODEL ------------------------------------


dofile("configuration/clamped_bar.lua")


----------------------------------- OBSERVATION ------------------------------


dofile("configuration/observation.lua")
-- In order to deactivate all observations.
observation.option.with_observation = false


----------------------------------- METHOD -----------------------------------


forward = {

   output_saver = {

      variable_list = {"forecast_time", "forecast_state"},
      file = output_directory .. "truth-%{name}.%{extension}",
      time = "step " .. Delta_t_clamped_bar * Nskip_save .. " 1.e-6",
      mode = output_mode,
      mode_scalar = output_mode_scalar

   },

   display = {

      show_iteration = false,
      show_time = true

   },

   output = {

      configuration = output_directory .. "truth.lua",
      log = output_directory .. "truth.log"

   }

}


observation_generator = forward
observation_generator.output_saver.variable_list
    = {"observation_time", "observation", "forecast_state"}
