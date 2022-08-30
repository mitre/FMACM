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

#include "public/NMObserver.h"


NMObserver::NMObserver(void) {
   curr_NM = -2;
}


NMObserver::~NMObserver(void) {
}

void NMObserver::output_NM_values(double predictedDistance,
                                  double trueDistance,
                                  double time,
                                  double currIAS,
                                  double currGS,
                                  double targetGS,
                                  double minIAS,
                                  double maxIAS,
                                  double minTAS,
                                  double maxTAS) {
   // Creates and adds a new entry to the nautical mile observer report.
   //
   // predictedDistance:predicted distance (current distance from IM algorithms).
   // trueDistance:true distance.
   // time:time.
   // currIAS:current aircraft indicated airspeed.
   // currGS:current aircraft ground speed.
   // targetGS:target aircraft ground speed.
   // minIAS:minimum aircraft indicated airspeed.
   // maxIAS:maximum aircraft indicated airspeed.
   // minTAS:mininum aircraft true airspeed.
   // maxTAS:maximum aircraft true airspeed.

   NMObserverEntry new_entry;

   new_entry.predictedDistance = predictedDistance;
   new_entry.trueDistance = trueDistance;
   new_entry.time = time;
   new_entry.acIAS = currIAS;
   new_entry.acGS = currGS;
   new_entry.targetGS = targetGS;
   new_entry.minIAS = minIAS;
   new_entry.maxIAS = maxIAS;
   new_entry.minTAS = minTAS;
   new_entry.maxTAS = maxTAS;

   entry_list.push_back(new_entry);
}

void NMObserver::initialize_stats() {
   // sets the stats size
   while (predictedDistance.size() < entry_list.size()) {
      Statistics temp_value;
      predictedDistance.push_back(0.0);
      trueDistance.push_back(0.0);
      ac_IAS_stats.push_back(temp_value);
      ac_GS_stats.push_back(temp_value);
      target_GS_stats.push_back(temp_value);
      min_IAS_stats.push_back(temp_value);
      max_IAS_stats.push_back(temp_value);
   }
}
