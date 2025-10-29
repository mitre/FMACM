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

#include <scalar/Angle.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Temperature.h>
#include <scalar/Time.h>

#include <map>
#include <vector>

#include "public/AircraftState.h"
#include "public/Logging.h"
#include "public/OutputHandler.h"
#include "public/VerticalPath.h"

struct PredictionFileBase {
  public:
   PredictionFileBase() = default;
   ~PredictionFileBase() = default;

   struct PredictionData {
      enum DataSource {
         INVALID_SOURCE = -1,
         FMS_DESCENT = 0,
         IM_ALGO_OWNSHIP,
         IM_ALGO_TARGET,
         FMS_ASCENT,
      };

      PredictionData() = default;

      int iteration_number{-1};
      Units::Time simulation_time{Units::SecondsTime(-1.0)};
      std::string acid{};
      DataSource source{PredictionData::INVALID_SOURCE};
      Units::Length altitude{Units::MetersLength(-1.0)};
      Units::Speed IAS{Units::MetersPerSecondSpeed(-1.0)};
      double mach{-1};
      Units::Speed GS{Units::MetersPerSecondSpeed(-1.0)};
      Units::Speed TAS{Units::MetersPerSecondSpeed(-1.0)};
      Units::Time time_to_go{Units::SecondsTime(-1.0)};
      Units::Length distance_to_go{Units::MetersLength(-1.0)};
      Units::MetersPerSecondSpeed VwePred{Units::MetersPerSecondSpeed(-1.0)};
      Units::MetersPerSecondSpeed VwnPred{Units::MetersPerSecondSpeed(-1.0)};
      VerticalPath::PredictionAlgorithmType algorithm{VerticalPath::PredictionAlgorithmType::UNDETERMINED};
      aaesim::open_source::bada_utils::FlapConfiguration flap_setting{
            aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED};
      Units::MetersPerSecondSpeed vertical_rate{Units::MetersPerSecondSpeed(-1.0)};
      Units::DegreesAngle flight_path_angle{Units::DegreesAngle(-1)};
   };

   std::vector<PredictionData> ExtractPredictionDataFromVerticalPath(const unsigned int &iteration,
                                                                     const Units::Time simulation_time,
                                                                     const std::string &acid,
                                                                     const VerticalPath &vertical_path,
                                                                     const PredictionData::DataSource &source) {
      std::vector<PredictionFileBase::PredictionData> prediction_data;

      for (auto m = 0; m < vertical_path.along_path_distance_m.size(); ++m) {
         PredictionFileBase::PredictionData pdata;

         pdata.iteration_number = iteration;
         pdata.source = source;
         pdata.acid = acid;
         pdata.simulation_time = simulation_time;

         pdata.altitude = Units::MetersLength(vertical_path.altitude_m[m]);
         pdata.IAS = Units::MetersPerSecondSpeed(vertical_path.cas_mps[m]);
         pdata.mach = vertical_path.mach[m];
         pdata.GS = Units::MetersPerSecondSpeed(vertical_path.gs_mps[m]);
         pdata.TAS = Units::MetersPerSecondSpeed(vertical_path.true_airspeed[m]);
         pdata.time_to_go = Units::SecondsTime(vertical_path.time_to_go_sec[m]);
         pdata.distance_to_go = Units::MetersLength(vertical_path.along_path_distance_m[m]);
         pdata.VwePred = vertical_path.wind_velocity_east[m];
         pdata.VwnPred = vertical_path.wind_velocity_north[m];
         pdata.algorithm = vertical_path.algorithm_type[m];
         pdata.flap_setting = vertical_path.flap_setting[m];
         pdata.vertical_rate = Units::MetersPerSecondSpeed(vertical_path.altitude_rate_mps[m]);
         pdata.flight_path_angle = Units::DegreesAngle(Units::RadiansAngle(vertical_path.theta_radians[m]));

         prediction_data.push_back(pdata);
      }
      return prediction_data;
   }
};
