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

enum nav_type { LAAS_APPROACH, WAAS_APPROACH, ILS, WAAS_ENROUTE, SPS_ENROUTE, DME, RNP1, RNP2, RNP4 };

// Glide-slope error is expressed in terms of an angle for the ILS.  Vertical errors
// is proportional to the distance from the runway threshold.  Radians
const double LOCALIZER_GLIDE_SLOPE_ERROR_SIGMA = 0.001;

class TSE {
  public:
   TSE();

   ~TSE() = default;

   // Input Data:
   nav_type nav_sys_type;  // The type of navigation system this is

   // Other Data:
   double rw_threshold_x,  // Coordinates of the runway threshold (at
         rw_threshold_y,   // centerline), ft

         // Note, all navigation system errors are modelled as a second order linear filter
         // with constant coefficients.  The system response is determined by the noise level
         // as well as the filter bandwith and damping ratio.  The following six variables
         // are used to model the control system response.

         turn_omega,  // Filter response frequency for nav system,
         alt_omega,   // hz for turn, altitude, speed control
         spd_omega,

         turn_damping_ratio,  // Filter damping ratio for nav system
         spd_damping_ratio,   // (unitless) for turn, altitude, speed control
         alt_damping_ratio,

         sig_delta_turn,  // Standard deviation of the cross-track position
                          // error (ft) for determining control filter response
         sig_delta_alt,   // Standard deviation of the altitude error (ft)

         dt,
         d_xtrk_last,  // last value for offset in cross-track direction
                       // (for turn process noise)
         d_hd_last,    // last value for heading offset (for turn process noise)

         del_h_last,    // last value for the vertical position
         del_gam_last,  // last value for the pitch in rad

         along_track_wind_gust, intermed_gust_state, cross_track_wind_gust, in_track_wind, x_track_wind,
         windspeed,       // initialized from run file
         wind_direction;  //  converted from runfile's degrees to radians

   void init(double rw_x,  // runway threshold x coordinate
             double rw_y   // runway threshold y coordiante
   );

   void generate_speed_process_noise(double nom_speed,         // nominal speed, ft/s
                                     double cur_speed,         // current speed, ft/s
                                     double in_track_dev,      // deviation in position along track,ft
                                     double *speed_chg_rate);  // output speed change rate, ft/s/s

   void generate_turn_process_noise(double v0, double *d_hd, double *d_xtrk, double sigma_delta, double k1, double w1,
                                    double air_spd, double *xtrk);

   void generate_alt_process_noise(double x,  // distance from runway
                                   double hr, double refang,
                                   double va,        // horizontal speed, ft/s
                                   double cur_z,     // current altitude, ft
                                   double cur_zd,    // current altitude rate, ft/s
                                   double *del_h,    // difference in height between ref. path and actual path
                                   double *del_gam,  // difference in pitch angle between ref & actual, radian
                                   double sig_h);    // standard deviation of altitude TSE, ft

   void generate_tse(double cur_x,  // current state, ft, s
                     double cur_y, double cur_z, double cur_xd, double cur_yd, double cur_zd,
                     double nom_x,  // nominal state ft, s
                     double nom_y, double nom_z, double nom_xd, double nom_yd, double nom_zd, double *dh, double *dy,
                     double *del_spd,  // output linear (speed change) accel, ft/s/s
                     double *del_h,
                     double *del_gam,  // output altitude accel, ft/s/s
                     double *xtrk);

   //  These 3 routines are the latest version of Tony Warren's paper (rev 3.1 from 10/27/99) - wind adjustments
   double update_alongtrack_groundspeed(double gspeed_adj, double alt, double airspeed,
                                        double windspeed);  //  updates speed adjustment for wind
   double update_intermed_gust_state(double intermed_gust_state, double alt, double airspeed, double windspeed);

   double update_crosstrack_groundspeed(double intermed_gust_state, double ct_gspeed_adj, double alt, double airspeed,
                                        double windspeed);

   //  This routine is a test caller for the previous 3 routines
   void update_wind_gust_states(double windspeed, double airspeed, double alt);
};
