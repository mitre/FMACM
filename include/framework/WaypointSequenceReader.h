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

#pragma once

#include "public/DataReader.h"

#include <vector>

#include "utility/CustomUnits.h"
#include "scalar/Angle.h"
#include "scalar/Length.h"

namespace fmacm {
class WaypointSequenceReader final : aaesim::open_source::DataReader {
  public:
   struct WaypointSequenceRow {
      WaypointSequenceRow()
         : latitude(Units::ZERO_ANGLE),
           longitude(Units::ZERO_ANGLE),
           bank_angle(Units::ZERO_ANGLE),
           turn_radius(Units::ZERO_LENGTH){};
      Units::Angle latitude;
      Units::Angle longitude;
      Units::Angle bank_angle;
      Units::Length turn_radius;
   };
   WaypointSequenceReader(std::string file_name) : DataReader(file_name, 1, 4){};
   std::vector<WaypointSequenceRow> ReadFile();
};
}  // namespace fmacm