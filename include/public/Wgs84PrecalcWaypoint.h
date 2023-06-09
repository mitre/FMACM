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

#include "public/LoggingLoadable.h"
#include "public/PrecalcConstraint.h"
#include "public/LatitudeLongitudePoint.h"
#include "public/AircraftIntent.h"
#include <scalar/UnsignedAngle.h>
#include <scalar/SignedAngle.h>

namespace aaesim {

class Wgs84PrecalcWaypoint {
  public:
   Wgs84PrecalcWaypoint();

   virtual ~Wgs84PrecalcWaypoint() = default;

   bool operator==(const Wgs84PrecalcWaypoint &obj) const;

   std::string m_name;
   AircraftIntent::Arinc424LegType m_leg_type;
   Units::Length m_leg_length;
   Units::SignedRadiansAngle m_enu_course_out_angle;
   Units::SignedRadiansAngle m_enu_course_in_angle;
   LatitudeLongitudePoint m_position;
   LatitudeLongitudePoint m_rf_leg_center;
   Units::MetersLength m_radius_rf_leg;
   Units::RadiansAngle m_bank_angle;
   Units::MetersPerSecondSpeed m_ground_speed;
   PrecalcConstraint m_precalc_constraints;
};

}  // namespace aaesim