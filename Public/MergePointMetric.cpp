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

#include "public/MergePointMetric.h"
#include "public/AircraftCalculations.h"
#include <stdexcept>
// #include <AchievePointCalcs.h>

using namespace std;
log4cplus::Logger MergePointMetric::logger = log4cplus::Logger::getInstance("MergePointMetric");

MergePointMetric::MergePointMetric(void) {
   m_im_ac_id = 0;
   m_target_ac_id = 0;
   mReportMetrics = false;
   mMergePointName = "";
   mIMDist = mMergeDist = Units::infinity();
   mTargY = mTargX = mIMY = mIMX = 0;
   mMergePointY = mMergePointX = Units::ZERO_LENGTH;
}


MergePointMetric::~MergePointMetric(void) {
}

void MergePointMetric::determineMergePoint(const AircraftIntent &imintent,
                                           const AircraftIntent &targintent) {

   // Determines and sets the merge point between the IM and target aircraft.
   // The merge point is defined as the first waypoint in the route where the
   // remainder of the IM and target route match
   // If no merge point is found, this class will not report any metrics.
   //
   // imintent:intent of IM aircraft containing the list of waypoints.
   // targintent:intent of target aircraft containing the list of waypoints.

   m_im_ac_id = imintent.GetId();
   m_target_ac_id = targintent.GetId();

   // overwrite these if merge point is found
   mReportMetrics = false;
   mMergePointName = "";

   std::pair<const int, const int> indices = imintent.FindCommonWaypoint(targintent);
   const int index = indices.first;

   if (m_im_ac_id == m_target_ac_id) {
      LOG4CPLUS_DEBUG(logger, "Target ID matches own for acid " << m_im_ac_id
            << ". No merge metrics will be reported.");
   }
   else if (index == -1) {
      LOG4CPLUS_DEBUG(logger, "No merge point found for im acid " << imintent.GetId() <<
            " and target acid " << targintent.GetId()
            << ". No merge metrics will be reported.");
   } else {
      mMergePointName = imintent.GetWaypointName(index);
      mMergePointX = imintent.GetWaypointX(index);
      mMergePointY = imintent.GetWaypointY(index);
      mReportMetrics = true;
   }

}


void MergePointMetric::update(double imXNew,
                              double imYNew,
                              double targXNew,
                              double targYNew) {

   // Replaces the current IM and target position if the new IM position is closer to the
   // merge point than the current IM point.  Distances are computed in nmi.
   //
   // imXNew,imYNew:the new IM position.
   // targXNew,targYNew:the  new target position.

   if (!mReportMetrics) {
      return;
   } // nothing to do

   if (newPointCloser(imXNew, imYNew)) {
      // Replace the current information with the new information.

      mIMX = imXNew;
      mIMY = imYNew;
      mIMDist = AircraftCalculations::PtToPtDist(
            mMergePointX,
            mMergePointY,
            Units::FeetLength(mIMX),
            Units::FeetLength(mIMY));

      mTargX = targXNew;
      mTargY = targYNew;
      mMergeDist = AircraftCalculations::PtToPtDist(
            Units::FeetLength(mIMX),
            Units::FeetLength(mIMY),
            Units::FeetLength(mTargX),
            Units::FeetLength(mTargY));
   }
}


string MergePointMetric::getMergePoint() {

   // Gets merge point.
   //
   // returns name of waypoint which is the merge point.

   return mMergePointName;
}


Units::Length MergePointMetric::getDist() {

   // Gets computed distance when IM aircraft was at merge point.
   //
   // returns distance in nmi.

   return mMergeDist;
}


bool MergePointMetric::newPointCloser(double x,
                                      double y) {

   // Checks if newest IM position closer to waypoint than the stored IM position.
   //
   // x,y:new IM position in feet.
   //
   // returns true if the new closer to the merge point.
   //         else false.

   return (AircraftCalculations::PtToPtDist(
         mMergePointX, mMergePointY,
         Units::FeetLength(x), Units::FeetLength(y))
           < mIMDist);
}


bool MergePointMetric::mergePointFound() {

   // Determines whether merge point already found.
   //
   // returns true if merge point found.
   //         else false

   return (mMergePointName.length() > 0);
}

void MergePointMetric::determineMetricsLocation(const AircraftIntent &imintent,
                                                const AircraftIntent &targintent,
                                                const string &waypointName) {

   bool foundWaypointIm = false;
   for (int i = 0; i < imintent.GetNumberOfWaypoints(); ++i) {
      foundWaypointIm = imintent.GetWaypointName(i).compare(waypointName) == 0;
      if (foundWaypointIm) {
         mMergePointName = imintent.GetWaypointName(i);
         mMergePointX = imintent.GetWaypointX(i);
         mMergePointY = imintent.GetWaypointY(i);
         mReportMetrics = true;
         break;
      }
   }

   if (!foundWaypointIm) {
      LOG4CPLUS_DEBUG(logger, "Cannot find waypoint in aircraft intents. Merge Point Metrics will not be reported.");
      mReportMetrics = false;
      mMergePointName = "";
   }

}

bool MergePointMetric::willReportMetrics() const {
   return mReportMetrics;
}

int MergePointMetric::GetImAcId() const {
   return m_im_ac_id;
}

int MergePointMetric::GetTargetAcId() const {
   return m_target_ac_id;
}
