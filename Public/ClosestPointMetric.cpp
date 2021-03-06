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
