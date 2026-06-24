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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <scalar/UnsignedAngle.h>

#include <string>

#include "public/PrecalcConstraint.h"

class PrecalcWaypoint final {
  public:
   PrecalcWaypoint() = default;

   ~PrecalcWaypoint() = default;

   bool operator==(const PrecalcWaypoint &obj) const;

   std::string m_name{};

   Units::Length m_leg_length{Units::zero()};
   Units::UnsignedRadiansAngle m_course_angle{Units::zero()};

   Units::MetersLength m_x_pos_meters{Units::zero()};
   Units::MetersLength m_y_pos_meters{Units::zero()};

   Units::MetersLength m_rf_leg_center_x{Units::zero()};
   Units::MetersLength m_rf_leg_center_y{Units::zero()};
   Units::MetersLength m_radius_rf_leg{Units::zero()};

   Units::RadiansAngle m_bank_angle{Units::zero()};
   Units::MetersPerSecondSpeed m_ground_speed{Units::zero()};

   aaesim::open_source::PrecalcConstraint m_precalc_constraints{};
};
