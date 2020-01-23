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

#include "public/IMCommandObserver.h"

IMCommandObserver::IMCommandObserver(void) {
   id = -1;
   time = 0.0;
   distance_to_go = 0.0;
   state_altitude = 0.0;
   state_TAS = 0.0;
   state_groundspeed = 0.0;
   IAS_command = 0.0;
   unmodified_IAS = 0.0;
   TAS_command = 0.0;
   reference_velocity = 0.0;
   reference_distance = 0.0;
   predictedDistance = 0.0;
   distance_difference = 0.0;
   trueDistance = 0.0;
   iteration = 0;
}

IMCommandObserver::~IMCommandObserver(void) {
}

// operator < for sort algorithm
bool IMCommandObserver::operator<(const IMCommandObserver &im_in) const {
   bool result = false;

   // if a.id < b.id OR a.id == b.id && a.time < b.time then a is < than b
   if (this->id < im_in.id) {
      result = true;
   } else if (this->id == im_in.id && this->time < im_in.time) {
      result = true;
   }

   return result;
}
