// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001
// and is subject to Federal Aviation Administration Acquisition Management System
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE
// Corporation and do not necessarily reflect the views of the Federal Aviation
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <string>
#include <filesystem>

#include "scalar/Length.h"
#include "scalar/Temperature.h"
#include "public/Atmosphere.h"

namespace aaesim {
struct ScenarioConfiguration final {
   int number_of_iterations;
   double start_time_rng_seed;
   std::filesystem::path bada_data_path;
   std::filesystem::path wind_truth_file;
   std::filesystem::path wind_forecast_file;
   std::filesystem::path ttv_file;
   bool use_wind;
   int predicted_wind_option;
   Units::Length adsb_reception_range_threshold;
   Units::CelsiusTemperature true_temperature_offset;
   Atmosphere::AtmosphereType atmosphere_type;
   std::string scenario_simple_name;
   std::filesystem::path scenario_path;
   std::filesystem::path output_path;
   Units::Time mean_interdelivery_time;
   Units::Time standard_deviation_interdelivery_time;
   bool start_time_is_overridden;
   bool enable_verbose_output_data;
};
}  // namespace aaesim
