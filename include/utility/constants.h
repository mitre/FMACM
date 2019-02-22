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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once
namespace aaesim {
   namespace constants {
      const int MAX_NUM_WAYPOINTS = 20;
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
      const double FEET_PER_SECOND_TO_KNOTS = 0.592483801;
      const double KNOTS_TO_FEET_PER_SECOND = 1.687809857;
      const double BIGNUM = 9.9E99;
// FIXME imprecise; METER2FOOT = 1/FT_M
      const double METERS_TO_FEET = 3.2808399;

      const double BARO_ALT_SIG = 6.0;
      const double BARO_ALT_RATE_SIG = 2.5;
      const int BARO_ALT_ERR_STEPS = 121;

      const extern double BARO_ALT_ERR_GRAD[BARO_ALT_ERR_STEPS];
      const extern double BARO_ALT_ERR_ALTITUDES[BARO_ALT_ERR_STEPS];
      const extern double BARO_ALT_ERROR[BARO_ALT_ERR_STEPS];

      const double BARO_GRADIENT_ERR = 0;
      const double SA_NORTH_STD_DEV = 128.0;
      const double SA_EAST_STD_DEV = 105.0;
      const double SA_DOWN_STD_DEV = 220.0;

      const double OMG0 = 0.012;
      const double QC = 0.002585;
      const double BETA = 0.707106781;

      const int TRACKING = 1;
      const int TURNING = 2;


      const double MAX_BANK_ANGLE_RADIAN = (25. * DEGREES_TO_RADIAN);
      const double ROLL_RATE_RADIAN_PER_SECOND = (3.0 * DEGREES_TO_RADIAN);

      // Gravitational acceleration is defined by
      // International Committee on Weights and Measures (1901)
      // and International Bureau of Weights and Measures (current)
      // to be 9.80665 m/s^2
      const double GRAVITY_METERS_PER_SECOND = 9.80665;
      const double GRAVITY_FEET_PER_SECOND = GRAVITY_METERS_PER_SECOND / FEET_TO_METERS;

      /* The following constants are used for converting from geodetic to */
      /* conformal latitude.  They are found in NAS-MD-312 Appendix D. */
      const double GEOD_CONST_A = 0.9932773;
      const double GEOD_CONST_B = 0.0066625;
   }
}
