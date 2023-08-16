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
namespace aaesim {
namespace open_source {
namespace constants {
const double PI = 3.14159265358979323846;

const double DEGREES_PER_RADIAN = 180 / PI;
const double TWO_PI = 2 * PI;
const double RADIAN_TO_DEGREES = DEGREES_PER_RADIAN;
const double DEGREES_TO_RADIAN = 1.0 / RADIAN_TO_DEGREES;

// exact conversion constants
const double NAUTICAL_MILES_TO_METERS = 1852.0;
const double FEET_TO_METERS = 0.3048;
const double KNOTS_TO_METERS_PER_SECOND = (NAUTICAL_MILES_TO_METERS / 3600.0);
const double NAUTICAL_MILES_TO_FEET = 6076.115486;
const double KNOTS_TO_FEET_PER_SECOND = 1.687809857;

// Gravitational acceleration is defined by
// International Committee on Weights and Measures (1901)
// and International Bureau of Weights and Measures (current)
// to be 9.80665 m/s^2
const double GRAVITY_METERS_PER_SECOND = 9.80665;
const double GRAVITY_FEET_PER_SECOND = GRAVITY_METERS_PER_SECOND / FEET_TO_METERS;

}  // namespace constants
}  // namespace open_source
}  // namespace aaesim
