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
