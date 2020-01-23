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

#pragma once

#include "public/AircraftIntent.h"
#include <Length.h>

// Class to compute distance between IM aircraft and target aircraft when
// IM aircraft is at the merge point of the IM and target route.

class MergePointMetric
{


public:
   MergePointMetric(void);

   ~MergePointMetric(void);

   // Determines and stores the merge point.
   void determineMergePoint(const AircraftIntent &IMIntent,
                            const AircraftIntent &targIntent);

   void determineMetricsLocation(const AircraftIntent &IMIntent,
                                 const AircraftIntent &targIntent,
                                 const std::string &waypointName);

   // Updates IM and target position.
   void update(double imXNew,
               double imYNew,
               double targXNew,
               double targYNew);

   // Gets merge point (waypoint name).
   std::string getMergePoint();

   // Gets computed distance.
   Units::Length getDist();

   // Returns whether merge point is set or not.
   bool mergePointFound();

   bool willReportMetrics() const;
   int GetImAcId() const;
   int GetTargetAcId() const;

private:
   static log4cplus::Logger logger;

   // Checks if newest IM position closer to waypoint than the stored IM position.
   bool newPointCloser(double x,
                       double y);

   int m_im_ac_id;
   int m_target_ac_id;

   std::string mMergePointName;
   Units::Length mMergePointX;
   Units::Length mMergePointY;

   double mIMX;  // ft
   double mIMY;  // ft

   Units::Length mIMDist;

   double mTargX;  // ft
   double mTargY;  // ft

   Units::Length mMergeDist;

   bool mReportMetrics;

};
