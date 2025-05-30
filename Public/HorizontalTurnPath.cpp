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
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/HorizontalTurnPath.h"
#include "public/HorizontalPath.h"

/**
 * Determine whether this is a right or left turn by using the
 * two previous points in the trajectory to establish a line
 * and finding whether the turn center is on the left or right.
 * The HorizontalPath object which owns this HorizontalTurnPath
 * would be p2.
 */
aaesim::open_source::HorizontalTurnPath::TURN_DIRECTION aaesim::open_source::HorizontalTurnPath::GetTurnDirection(
      const HorizontalPath &p0, const HorizontalPath &p1) const {

   if (turn_type == UNKNOWN) return NO_TURN;

   double dx1 = p1.GetXPositionMeters() - p0.GetXPositionMeters();
   double dy1 = p1.GetYPositionMeters() - p0.GetYPositionMeters();
   double dx2 = x_position_meters - p1.GetXPositionMeters();
   double dy2 = y_position_meters - p1.GetYPositionMeters();

   double cross_product = dx1 * dy2 - dy1 * dx2;

   return (cross_product > 0) ? LEFT_TURN : RIGHT_TURN;
}
