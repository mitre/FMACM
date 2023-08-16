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

#include "framework/WaypointSequenceReader.h"

using namespace fmacm;

std::vector<WaypointSequenceReader::WaypointSequenceRow> WaypointSequenceReader::ReadFile() {
   std::vector<WaypointSequenceRow> data;
   while (Advance()) {
      Units::DegreesAngle latitude(GetDouble(0));
      Units::DegreesAngle longitude(GetDouble(1));
      Units::DegreesAngle bank_angle(GetDouble(3));
      Units::MetersLength turn_radius(GetDouble(2));

      WaypointSequenceReader::WaypointSequenceRow data_row;
      data_row.latitude = latitude;
      data_row.longitude = longitude;
      data_row.bank_angle = bank_angle;
      data_row.turn_radius = turn_radius;

      data.push_back(data_row);
   }

   static Units::MetersLength tolerance(1);
   const bool invalid_first_row = data.front().turn_radius > tolerance;
   const bool invalid_last_row = data.back().turn_radius > tolerance;
   if (invalid_first_row || invalid_last_row) {
      throw std::runtime_error(
            "Waypoint sequence is not allowed to start or end with a turn segment. Please remove the non-zero turn "
            "radius from the first and last row.");
   }

   return data;
}
