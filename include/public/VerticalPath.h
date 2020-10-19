// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <vector>
#include <iostream>
#include "Speed.h"

class VerticalPath
{

public:

   enum PredictionAlgorithmType
   {
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
      FPA_TO_CURRENT_POS
   };

   VerticalPath();

   virtual ~VerticalPath();

   void Append(const VerticalPath &in);

   void operator+=(const VerticalPath &in);

   bool operator==(const VerticalPath &obj) const;

   // These methods will convert to doubles and return a
   // new vector of doubles (this is a convenience for lab-related integration).
   const std::vector<double> GetWindVelocityEast() const;
   const std::vector<double> GetWindVelocityNorth() const;

   // data member lists for the vertical path.  NOTE that the values are normally in METERS from descent predictors.
   std::vector<double> along_path_distance_m;
   std::vector<double> altitude_m;
   std::vector<double> cas_mps;
   std::vector<double> altitude_rate_mps;
   std::vector<Units::Speed> true_airspeed;
   std::vector<double> tas_rate_mps;
   std::vector<double> theta_radians;
   std::vector<double> gs_mps;
   std::vector<double> time_to_go_sec;
   std::vector<double> mass_kg;
   std::vector<Units::MetersPerSecondSpeed> wind_velocity_east;
   std::vector<Units::MetersPerSecondSpeed> wind_velocity_north;
   std::vector<PredictionAlgorithmType> algorithm_type;
};
