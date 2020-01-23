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

   // Since you can't send a vector of Units::XXX over SDDF, these methods will convert to doubles and return a
   // new vector that we can send. This is lab related only.
   const std::vector<double> GetWindVelocityEast() const;

   const std::vector<double> GetWindVelocityNorth() const;

   // data member lists for the vertical path.  NOTE that the values are normally in METERS from descent predictors.
   std::vector<double> x;
   std::vector<double> h;
   std::vector<double> v;
   std::vector<double> h_dot;
   std::vector<double> v_dot;
   std::vector<double> theta;
   std::vector<double> gs;
   std::vector<double> time;
   std::vector<double> mass;
   std::vector<Units::MetersPerSecondSpeed> wind_velocity_east;
   std::vector<Units::MetersPerSecondSpeed> wind_velocity_north;
   std::vector<PredictionAlgorithmType> algorithm_type;
};
