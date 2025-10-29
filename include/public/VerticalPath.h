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

#include <scalar/Speed.h>

#include <iostream>
#include <vector>

#include "public/BadaUtils.h"

class VerticalPath final {
  public:
   enum PredictionAlgorithmType {
      UNDETERMINED = 0,
      LEVEL,
      LEVEL_DECEL1,
      LEVEL_DECEL2,
      CONSTANT_CAS,
      CONSTANT_MACH,
      CONSTANT_DECEL,
      IDLE1,
      IDLE2,
      FPA,
      FPA_DECEL,
      FPA_TO_CURRENT_POS,
      TAKEOFF_ROLL,
      ESF_CLIMB,
      CONSTANT_CAS_CLIMB,
      CONSTANT_MACH_CLIMB,
      LEVEL_ACCEL,
      LEVEL_FLIGHT
   };

   VerticalPath();

   virtual ~VerticalPath();

   void Append(const VerticalPath &in);

   void operator+=(const VerticalPath &in);

   bool operator==(const VerticalPath &obj) const;

   std::vector<double> along_path_distance_m;
   std::vector<double> altitude_m;
   std::vector<double> cas_mps;
   std::vector<double> mach;
   std::vector<double> altitude_rate_mps;
   std::vector<Units::Speed> true_airspeed;
   std::vector<double> tas_rate_mps;
   std::vector<double> theta_radians;
   std::vector<double> gs_mps;
   std::vector<double> time_to_go_sec;
   std::vector<double> mass_kg;
   std::vector<Units::Speed> wind_velocity_east;
   std::vector<Units::Speed> wind_velocity_north;
   std::vector<PredictionAlgorithmType> algorithm_type;
   std::vector<aaesim::open_source::bada_utils::FlapConfiguration> flap_setting;
};
