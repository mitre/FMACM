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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/CrossTrackObserver.h"


CrossTrackObserver::CrossTrackObserver(void) {
   time = -99999.0;
   x = 0.0;
   y = 0.0;
   dynamic_cross = 0.0;
   commanded_cross = 0.0;
   unmodified_cross = 0.0;
   psi_command = 0.0;
   phi = 0.0;
   limited_phi = 0.0;
   reported_distance = 0.0;
}


CrossTrackObserver::~CrossTrackObserver(void) {
}
