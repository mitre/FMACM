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

#include "public/HorizontalPath.h"
#include "public/AircraftIntent.h"
#include <vector>

namespace aaesim {
namespace test {
namespace utils {
static const double TOLERANCE_RADIANS = 1e-15;
static const double TOLERANCE_METERS = 1.0;
static const double TOLERANCE_METERS_TIGHT = 1e-8;

enum Quadrant { FIRST, SECOND, THIRD, FOURTH };

class PublicUtils {

  public:
   static std::vector<HorizontalPath> CreateStraightHorizontalPath(Quadrant quadrant);
   static AircraftIntent LoadAircraftIntent(std::string parmsfile);
   static AircraftIntent PrepareAircraftIntent(std::string parmsfile);
};
}  // namespace utils
}  // namespace test
}  // namespace aaesim
