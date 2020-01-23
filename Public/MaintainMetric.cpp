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

#include "public/MaintainMetric.h"
#include <math.h>


MaintainMetric::MaintainMetric(void) {

   // Constructor

   achieveByTime = -1.0;
   totalMaintainTime = 0.0;
   numCyclesOutsideThreshold = 0;
   m_output_enabled = false;
}


MaintainMetric::~MaintainMetric(void) {

   // Destructor
}


void MaintainMetric::AddSpacingErrorSec(double err) {

   // Adds data to be added for each pass through an IM::update method.
   // Also increments number of cycles if outside of threshold.
   //
   // err:input spacing error.

   spacingError.Insert(err);

   // TODO:Need to include time step in this if.

   if (fabs(err) > CYCLE_THRESHOLD) {
      numCyclesOutsideThreshold++;
   }
}


void MaintainMetric::SetTimeAtAbp(double aTime) {

   // Sets time aircraft went by achieve by point.
   //
   // aTime:achieve by time.

   achieveByTime = aTime;
}


void MaintainMetric::ComputeTotalMaintainTime(double cTime) {

   // Computes total maintain time subtracting the achieveByTime
   // from the current time.
   //
   // cTime:current time.

   totalMaintainTime = cTime - achieveByTime;
}


bool MaintainMetric::TimeAtAbpRecorded() {

   // Boolean to determine if achieveBy set.
   //
   // return:true if achieve by has valid time.
   //        false if achieve by does not have valid time.

   return (achieveByTime >= 0.0);
}


double MaintainMetric::getMeanErr() {

   // Gets mean spacing error.
   //
   // returns mean error.

   return spacingError.GetMean();
}


double MaintainMetric::getStdErr() {

   // Gets standard deviation of spacing error.
   //
   // returns standard deviation of error.

   return spacingError.ComputeStandardDeviation();
}


double MaintainMetric::getBound95() {

   // Gets 95th bound of spacing error.
   //
   // returns 95th bound of spacing error.

   return spacingError.Get95thBounds();
}


double MaintainMetric::getTotMaintain() {

   // Gets total maintain time.
   //
   // returns total maintain time.

   return totalMaintainTime;
}


int MaintainMetric::getNumCycles() {

   // Gets number of cycles with spacing errors > cycle threshold
   //
   // returns number of cycles.

   return numCyclesOutsideThreshold;
}


bool MaintainMetric::hasSamples() {

   // Determines whether there are any samples collected.
   //
   // returns true if there are samples
   //		   else false.

   return (spacingError.GetNumberOfSamples() > 0);
}

bool MaintainMetric::IsOutputEnabled() const {
   return m_output_enabled;
}

void MaintainMetric::SetOutputEnabled(bool output_enabled) {
   m_output_enabled = output_enabled;
}
