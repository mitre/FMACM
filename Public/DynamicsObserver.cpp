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

#include "public/DynamicsObserver.h"

DynamicsObserver::DynamicsObserver(void) {
   iter = -1;
   id = -1;
   time = -99999.0;
   achieved_groundspeed = 0.0;
   speed_command = 0.0;
   IAS_command = 0.0;
}

DynamicsObserver::~DynamicsObserver(void) {
}

// < operator to enable sorting of Dynamic Observer objects
bool DynamicsObserver::operator<(const DynamicsObserver &dyn_in) const {
   bool result = false;

   if (this->iter <= dyn_in.iter && this->id < dyn_in.id) {
      result = true;
   }

   return result;
}
