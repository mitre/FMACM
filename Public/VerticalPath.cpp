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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/VerticalPath.h"
#include "utility/constants.h"

using namespace std;

VerticalPath::VerticalPath(void) {
}

VerticalPath::~VerticalPath(void) {
}

// method to append another Trajectory to this one
void VerticalPath::append(const VerticalPath &in) {
   // loop to add the given list to the end of the current list
   for (int index = 0; index < (int) in.x.size(); index++) {
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
   }
}

// operator+= to act as mathematical append operation
void VerticalPath::operator+=(const VerticalPath &in) {
   this->append(in);
}

// Generic == implementation.
bool VerticalPath::operator==(const VerticalPath &obj) const {

   bool match = (this->x.size() == obj.x.size());
   match = match && (this->h.size() == obj.h.size());
   match = match && (this->v.size() == obj.v.size());
   match = match && (this->h_dot.size() == obj.h_dot.size());
   match = match && (this->v_dot.size() == obj.v_dot.size());
   match = match && (this->theta.size() == obj.theta.size());
   match = match && (this->gs.size() == obj.gs.size());
   match = match && (this->time.size() == obj.time.size());
   match = match && (this->mass.size() == obj.mass.size());
   match = match && (this->wind_velocity_east.size() == obj.wind_velocity_east.size());
   match = match && (this->wind_velocity_north.size() == obj.wind_velocity_north.size());


   for (auto ix = 0; match && (ix < x.size()); ix++) {
      match = match && (this->x[ix] == obj.x[ix]);
   }


   for (auto ix = 0; match && (ix < h.size()); ix++) {
      match = match && (this->h[ix] == obj.h[ix]);
   }


   for (auto ix = 0; match && (ix < v.size()); ix++) {
      match = match && (this->v[ix] == obj.v[ix]);
   }


   for (auto ix = 0; match && (ix < h_dot.size()); ix++) {
      match = match && (this->h_dot[ix] == obj.h_dot[ix]);
   }


   for (auto ix = 0; match && (ix < v_dot.size()); ix++) {
      match = match && (this->v_dot[ix] == obj.v_dot[ix]);
   }


   for (auto ix = 0; match && (ix < theta.size()); ix++) {
      match = match && (this->theta[ix] == obj.theta[ix]);
   }


   for (auto ix = 0; match && (ix < gs.size()); ix++) {
      match = match && (this->gs[ix] == obj.gs[ix]);
   }


   for (auto ix = 0; match && (ix < time.size()); ix++) {
      match = match && (this->time[ix] == obj.time[ix]);
   }


   for (auto ix = 0; match && (ix < mass.size()); ix++) {
      match = match && (this->mass[ix] == obj.mass[ix]);
   }

   for (auto ix = 0; match && (ix < wind_velocity_east.size()); ix++) {
      match = match && (this->wind_velocity_east[ix] == obj.wind_velocity_east[ix]);
   }

   for (auto ix = 0; match && (ix < wind_velocity_north.size()); ix++) {
      match = match && (this->wind_velocity_north[ix] == obj.wind_velocity_north[ix]);
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
