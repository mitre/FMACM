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

#pragma once

#include "math/Statistics.h"

#ifndef CYCLE_THRESHOLD
#define CYCLE_THRESHOLD 10
#endif


// Storage metrics class for statistics gathered during the maintain phase
// of the flight.

class MaintainMetric
{
public:
   MaintainMetric(void);

   ~MaintainMetric(void);

   // Adds data to be added for each pass through an IM::update method.
   void AddSpacingErrorSec(double err);

   // Sets time aircraft went by achieve by point.
   void SetTimeAtAbp(double time);

   // Boolean to determine if achieveBy set (achieveBy < 0.0)
   bool TimeAtAbpRecorded();

   // Computes total maintain time subtracting the achieveByTime
   // from the current time.
   void ComputeTotalMaintainTime(double cTime);

   // Gets mean spacing error.
   double getMeanErr();

   // Gets standard deviation of spacing error.
   double getStdErr();

   // Gets 95th bound of spacing error.
   double getBound95();

   // Gets total maintain time.
   double getTotMaintain();

   // Gets number of cycles with spacing errors > cycle threshold
   int getNumCycles();

   // Returns whether there are data samples to collect metrics from.
   bool hasSamples();
   bool IsOutputEnabled() const;
   void SetOutputEnabled(bool output_enabled);

private:
   // Running sum of time spacing errors between IM and target ac.
   Statistics spacingError;

   // Time went by achieve by point.
   double achieveByTime;

   // Time spent in maintain stage, (current time - achieve by time)
   double totalMaintainTime;

   // Number of cycles with a spacing error > 10 secs.
   int numCyclesOutsideThreshold;

   /** Output should be enabled for IM aircraft */
   bool m_output_enabled;
};
