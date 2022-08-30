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

#include "public/AircraftIntent.h"
#include <scalar/Length.h>

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
