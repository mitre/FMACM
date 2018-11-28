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

#include "public/ClosestPointMetric.h"
#include "public/AircraftCalculations.h"


ClosestPointMetric::ClosestPointMetric(void) {
   mMinDist = Units::NauticalMilesLength(1000000000.0);
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
