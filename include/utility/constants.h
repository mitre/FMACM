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

#pragma once
namespace aaesim {
   namespace constants {
      const int MAX_NUM_WAYPOINTS = 20;
      const double PI = 3.14159265358979323846;

      const double DEG_PER_RAD = 180 / PI;
      const double TWOPI = 2 * PI;
      // const double PI_OVER_TWO = 0.5 * PI;
      const double RADTOD = DEG_PER_RAD;
      const double DTORAD = 1.0 / RADTOD;

      // exact conversion constants
      const double NM_M = 1852.0;
      const double FT_M = 0.3048;
      const double KTS_MPS = (NM_M / 3600.0);

      const double NM2FT = 6076.115486;
      const double FPS2KT = 0.592483801;
      const double KT2FPS = 1.687809857;
//#define FPS2FPM (double)60.0
//#define FPM2FPS (double)0.016666667
      const double BIGNUM = 9.9E99;
//#define BIGINTNUM (int)1000000
// FIXME imprecise; METER2FOOT = 1/FT_M
      const double METER2FOOT = 3.2808399;

      // defines taken from trajectory_class_approach_forward
//#define FPM_MPS (double)0.00508
//#define KG_LB (double)2.204622
//#define LAPSE_RATE (double)0.0065
//#define LB_KG (double)0.453592
//#define LB_NEW (double)4.4497375
//#define NEW_LB (double)0.2247323
//#define MPS_FPM (double)196.850394
//#define REARTH_FT (double)20907363.55

//#define EQUIT_RAD_NM (double)3443.918467
//#define EQUIT_RAD_FT (double)20925646.33
//#define ECCENTRICITY (double)0.081819191

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

      //radian:
      const double MAX_BANK_ANGLE = (25. * DTORAD);
      //radian/second:
      const double ROLL_RATE = (3.0 * DTORAD);

      // Gravitational acceleration is defined by
      // International Committee on Weights and Measures (1901)
      // and International Bureau of Weights and Measures (current)
      // to be 9.80665 m/s^2
      const double GRAV_MPS = 9.80665;
      const double GRAV = GRAV_MPS / FT_M;

      /* The following constants are used for converting from geodetic to */
      /* conformal latitude.  They are found in NAS-MD-312 Appendix D. */
      const double GEOD_CONST_A = 0.9932773;
      const double GEOD_CONST_B = 0.0066625;
   }
}
