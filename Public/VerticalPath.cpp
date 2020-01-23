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

#include "public/VerticalPath.h"
#include "utility/constants.h"

using namespace std;

VerticalPath::VerticalPath() = default;

VerticalPath::~VerticalPath() = default;

void VerticalPath::Append(const VerticalPath &in) {
   for (int index = 0; index < in.x.size(); index++) {
      x.push_back(in.x[index]);
      h.push_back(in.h[index]);
      v.push_back(in.v[index]);
      h_dot.push_back(in.h_dot[index]);
      v_dot.push_back(in.v_dot[index]);
      theta.push_back(in.theta[index]);
      gs.push_back(in.gs[index]);
      time.push_back(in.time[index]);
      mass.push_back(in.mass[index]);
      wind_velocity_east.push_back(in.wind_velocity_east[index]);
      wind_velocity_north.push_back(in.wind_velocity_north[index]);
      algorithm_type.push_back(in.algorithm_type[index]);
   }
}

void VerticalPath::operator+=(const VerticalPath &in) {
   Append(in);
}

bool VerticalPath::operator==(const VerticalPath &obj) const {

   bool match = (x.size() == obj.x.size());
   match = match && (h.size() == obj.h.size());
   match = match && (v.size() == obj.v.size());
   match = match && (h_dot.size() == obj.h_dot.size());
   match = match && (v_dot.size() == obj.v_dot.size());
   match = match && (theta.size() == obj.theta.size());
   match = match && (gs.size() == obj.gs.size());
   match = match && (time.size() == obj.time.size());
   match = match && (mass.size() == obj.mass.size());
   match = match && (wind_velocity_east.size() == obj.wind_velocity_east.size());
   match = match && (wind_velocity_north.size() == obj.wind_velocity_north.size());
   match = match && (algorithm_type.size() == obj.algorithm_type.size());


   for (auto ix = 0; match && (ix < x.size()); ix++) {
      match = match && (x[ix] == obj.x[ix]);
   }


   for (auto ix = 0; match && (ix < h.size()); ix++) {
      match = match && (h[ix] == obj.h[ix]);
   }


   for (auto ix = 0; match && (ix < v.size()); ix++) {
      match = match && (v[ix] == obj.v[ix]);
   }


   for (auto ix = 0; match && (ix < h_dot.size()); ix++) {
      match = match && (h_dot[ix] == obj.h_dot[ix]);
   }


   for (auto ix = 0; match && (ix < v_dot.size()); ix++) {
      match = match && (v_dot[ix] == obj.v_dot[ix]);
   }


   for (auto ix = 0; match && (ix < theta.size()); ix++) {
      match = match && (theta[ix] == obj.theta[ix]);
   }


   for (auto ix = 0; match && (ix < gs.size()); ix++) {
      match = match && (gs[ix] == obj.gs[ix]);
   }


   for (auto ix = 0; match && (ix < time.size()); ix++) {
      match = match && (time[ix] == obj.time[ix]);
   }


   for (auto ix = 0; match && (ix < mass.size()); ix++) {
      match = match && (mass[ix] == obj.mass[ix]);
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
