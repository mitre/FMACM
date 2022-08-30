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

#include "public/ClosestPointMetric.h"
#include "public/AircraftCalculations.h"


ClosestPointMetric::ClosestPointMetric(void) {
   m_im_ac_id = 0;
   m_target_ac_id = 0;
   m_report_metrics = false;
   mMinDist = Units::infinity();
}


ClosestPointMetric::~ClosestPointMetric(void) {
}


void ClosestPointMetric::update(double imx,
                                double imy,
                                double targx,
                                double targy) {

   // Computes the distance between im and target aircraft based on the input
   // positions and replaces the minimum distance if the new distance closer.
   // Distance is in nmi.
   //
   // imx,imy:position of IM aircraft.
   // targx,targy:position of target aircraft.

   Units::Length dist = AircraftCalculations::PtToPtDist(
         Units::FeetLength(imx),
         Units::FeetLength(imy),
         Units::FeetLength(targx),
         Units::FeetLength(targy));

   if (dist < mMinDist) {
      mMinDist = dist;
   }
}


Units::Length ClosestPointMetric::getMinDist() {

   // Gets minimum distance.
   //
   // returns minimum distance.

   return mMinDist;
}

void ClosestPointMetric::SetAcIds(int im_ac_id, int target_ac_id) {
   m_im_ac_id = im_ac_id;
   m_target_ac_id = target_ac_id;
   m_report_metrics = (im_ac_id != target_ac_id && target_ac_id >= 0);
}

int ClosestPointMetric::GetImAcId() const {
   return m_im_ac_id;
}

bool ClosestPointMetric::IsReportMetrics() const {
   return m_report_metrics;
}

int ClosestPointMetric::GetTargetAcId() const {
   return m_target_ac_id;
}
