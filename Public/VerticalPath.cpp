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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/VerticalPath.h"
#include "utility/constants.h"

using namespace std;

VerticalPath::VerticalPath() = default;

VerticalPath::~VerticalPath() = default;

void VerticalPath::Append(const VerticalPath &in) {
   for (int index = 0; index < in.along_path_distance_m.size(); index++) {
      along_path_distance_m.push_back(in.along_path_distance_m[index]);
      altitude_m.push_back(in.altitude_m[index]);
      cas_mps.push_back(in.cas_mps[index]);
      altitude_rate_mps.push_back(in.altitude_rate_mps[index]);
      true_airspeed.push_back(in.true_airspeed[index]);
      tas_rate_mps.push_back(in.tas_rate_mps[index]);
      theta_radians.push_back(in.theta_radians[index]);
      gs_mps.push_back(in.gs_mps[index]);
      time_to_go_sec.push_back(in.time_to_go_sec[index]);
      mass_kg.push_back(in.mass_kg[index]);
      wind_velocity_east.push_back(in.wind_velocity_east[index]);
      wind_velocity_north.push_back(in.wind_velocity_north[index]);
      algorithm_type.push_back(in.algorithm_type[index]);
   }
}

void VerticalPath::operator+=(const VerticalPath &in) {
   Append(in);
}

bool VerticalPath::operator==(const VerticalPath &obj) const {

   bool match = (along_path_distance_m.size() == obj.along_path_distance_m.size());
   match = match && (altitude_m.size() == obj.altitude_m.size());
   match = match && (cas_mps.size() == obj.cas_mps.size());
   match = match && (altitude_rate_mps.size() == obj.altitude_rate_mps.size());
   match = match && (true_airspeed.size() == obj.true_airspeed.size());
   match = match && (tas_rate_mps.size() == obj.tas_rate_mps.size());
   match = match && (theta_radians.size() == obj.theta_radians.size());
   match = match && (gs_mps.size() == obj.gs_mps.size());
   match = match && (time_to_go_sec.size() == obj.time_to_go_sec.size());
   match = match && (mass_kg.size() == obj.mass_kg.size());
   match = match && (wind_velocity_east.size() == obj.wind_velocity_east.size());
   match = match && (wind_velocity_north.size() == obj.wind_velocity_north.size());
   match = match && (algorithm_type.size() == obj.algorithm_type.size());


   for (auto ix = 0; match && (ix < along_path_distance_m.size()); ix++) {
      match = match && (along_path_distance_m[ix] == obj.along_path_distance_m[ix]);
   }


   for (auto ix = 0; match && (ix < altitude_m.size()); ix++) {
      match = match && (altitude_m[ix] == obj.altitude_m[ix]);
   }


   for (auto ix = 0; match && (ix < cas_mps.size()); ix++) {
      match = match && (cas_mps[ix] == obj.cas_mps[ix]);
   }


   for (auto ix = 0; match && (ix < altitude_rate_mps.size()); ix++) {
      match = match && (altitude_rate_mps[ix] == obj.altitude_rate_mps[ix]);
   }


   for (auto ix = 0; match && (ix < tas_rate_mps.size()); ix++) {
      match = match && (tas_rate_mps[ix] == obj.tas_rate_mps[ix]);
   }


   for (auto ix = 0; match && (ix < theta_radians.size()); ix++) {
      match = match && (theta_radians[ix] == obj.theta_radians[ix]);
   }


   for (auto ix = 0; match && (ix < gs_mps.size()); ix++) {
      match = match && (gs_mps[ix] == obj.gs_mps[ix]);
   }

    for (auto ix = 0; match && (ix < true_airspeed.size()); ix++) {
        match = match && (true_airspeed[ix] == obj.true_airspeed[ix]);
    }

   for (auto ix = 0; match && (ix < time_to_go_sec.size()); ix++) {
      match = match && (time_to_go_sec[ix] == obj.time_to_go_sec[ix]);
   }


   for (auto ix = 0; match && (ix < mass_kg.size()); ix++) {
      match = match && (mass_kg[ix] == obj.mass_kg[ix]);
   }

   for (auto ix = 0; match && (ix < wind_velocity_east.size()); ix++) {
      match = match && (wind_velocity_east[ix] == obj.wind_velocity_east[ix]);
   }

   for (auto ix = 0; match && (ix < wind_velocity_north.size()); ix++) {
      match = match && (wind_velocity_north[ix] == obj.wind_velocity_north[ix]);
   }

   for (auto ix = 0; match && (ix < algorithm_type.size()); ix++) {
      match = match && (algorithm_type[ix] == obj.algorithm_type[ix]);
   }

   return match;

}

const std::vector<double> VerticalPath::GetWindVelocityEast() const {
   std::vector<double> vwe_mps_vector;
   for (auto i = wind_velocity_east.begin(); i != wind_velocity_east.end(); ++i) {
      vwe_mps_vector.push_back(Units::MetersPerSecondSpeed(*i).value());
   }

   return vwe_mps_vector;
}

const std::vector<double> VerticalPath::GetWindVelocityNorth() const {
   std::vector<double> vwn_mps_vector;
   for (auto i = wind_velocity_north.begin(); i != wind_velocity_north.end(); ++i) {
      vwn_mps_vector.push_back(Units::MetersPerSecondSpeed(*i).value());
   }

   return vwn_mps_vector;
}
