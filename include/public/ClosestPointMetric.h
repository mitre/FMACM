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

#include <scalar/Length.h>


// Class used to compute the closest point metric, which is used to determine
// closest point between the IM and target aircrafts.

class ClosestPointMetric
{
public:
   ClosestPointMetric(void);

   ~ClosestPointMetric(void);

   // Computes distance for input position and updates minimum
   // position if less than minimum distance.
   void update(double imx,
               double imy,
               double targx,
               double targy);

   Units::Length getMinDist();

   void SetAcIds(int im_ac_id, int target_ac_id);
   int GetImAcId() const;
   bool IsReportMetrics() const;
   int GetTargetAcId() const;

private:
   int m_im_ac_id;
   int m_target_ac_id;
   bool m_report_metrics;

   // Minimum distance between IM and target aircraft.
   Units::Length mMinDist;

};
