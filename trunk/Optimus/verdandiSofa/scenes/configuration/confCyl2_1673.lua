Delta_t_clamped_bar = 0.001
final_time_clamped_bar = 3.0
Nskip_save = 1

output_directory = "result2_1673/"

--observation_file = output_directory .. "truth-observation.bin"
--observation.option.
--observation_generator = forward
--observation_generator.output_saver.variable_list = {"observation_time", "observation", "forecast_state"}

forward = {
   output_saver = {
      variable_list = {"forecast_time", "forecast_state"},
      file = output_directory .. "truth-%{name}.%{extension}",
      time = "step " .. Delta_t_clamped_bar * Nskip_save .. " 1.e-6",
      mode = "binary",
      mode_scalar = "text"
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

reduced_order_unscented_kalman_filter = {
   data_assimilation = {
      analyze_first_step = false,
      with_resampling = false,
      -- Indicates how R is stored: "matrix", "matrix_inverse".
      observation_error_variance = "matrix_inverse",
   },

   sigma_point = {      
      type = "star" -- Choice of sigma-points: "canonical", "star" or "simplex".
   },

   display = {
      show_iteration = false,
      show_time = true
   },

   output_saver = {
      variable_list = {"forecast_time", "forecast_state", "analysis_time", "analysis_state"},
      file = output_directory .. "roukf-%{name}.%{extension}",
      time = "step " .. Delta_t_clamped_bar * Nskip_save .. " 1.e-6",
      mode = "text",
      mode_scalar = "text"
   },
   output = {
     configuration = output_directory .. "roukf.lua",
     log = output_directory .. "roukf_%{rank}.log"
   },
}

observation = {   
   with_observation = false,
   file = output_directory .. "truth-forecast_state.bin",   
   type = "state",   
   Delta_t_constant = true,
   -- If the period with which observations are available is non constant
   -- one should define the observation time file.
   observation_time_file = output_directory .. "time.dat",
   -- Else  one should define the period with which
   -- observations are available.
   Delta_t = Delta_t_clamped_bar * Nskip_save,
   -- Period with which available observations are actually assimilated.
   Nskip = 1,
   -- Duration during which observations are assimilated.
   final_time = final_time_clamped_bar,

   -- In case of triangles widths defined in a file.
   width_file = "configuration/width.bin",

   aggregator = {

      -- The interpolation type may be "step", "triangle" or "interpolation".
      type = "step",
      width_left = 0.005,
      width_right = 0.005,

      -- If the type is "triangle", the triangles widths may be the same for
      -- all observations ("constant") or not ("per-observation").
      width_property = "constant",

      -- If the triangles widths are not constant, or in the case of
      -- "interpolation", one should define an observation interval. It is
      -- assumed that the observations outside this interval have no
      -- contribution.
      width_left_upper_bound = 1.,
      width_right_upper_bound = 1.,

      -- If the value is true, each observation can be used only one time.
      discard_observation = true
   },

   error = {
      -- Variance of observational errors.
      variance = 1.0e-2
   },

   operator = {      
      scaled_identity = false,      
      diagonal_value = 1.,      
      value = {}
   },

   option = {      
      with_observation = true
   },

   location = {
      observation_location = {1, 0}
   }
}

Nstate = 1556
Nobservation = 1554
for i = 1, Nobservation * Nstate do
    observation.operator.value[i] = 0.
end
for i = 1, Nobservation do
    for j = 1, Nobservation do
        if i == j then
            observation.operator.value[Nstate * (i - 1) + j] = 1.0
        end
    end
end

