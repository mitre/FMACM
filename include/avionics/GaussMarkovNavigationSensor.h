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

#include "avionics/NavigationSensor.h"

#include "avionics/FirstOrderGaussMarkovProcess.h"
#include "avionics/Track.h"
#include "public/AircraftState.h"
#include "public/Logging.h"

namespace aaesim {
class GaussMarkovNavigationSensor : public NavigationSensor {
  public:
   GaussMarkovNavigationSensor() : baro_meas_to_assap(2, 0., 1) {
      use_position_bias = false;
      use_velocity_bias = false;
      mModelLoaded = false;
      nacp = 0;
      nacv = 0;
      nicp = 0;
      nicv = 0;
      x = 0.0;
      y = 0.0;
      z = 0.0;
      xd = 0.0;
      yd = 0.0;
      zd = 0.0;
      x_bias = 0.0;
      y_bias = 0.0;
      z_bias = 0.0;
      xd_bias = 0.0;
      yd_bias = 0.0;
      zd_bias = 0.0;
   };

   ~GaussMarkovNavigationSensor() = default;

   GaussMarkovNavigationSensor(const GaussMarkovNavigationSensor &in);

   GaussMarkovNavigationSensor &operator=(const GaussMarkovNavigationSensor &in);

   void init(double nacp, double nacv, double nicp, double nicv, bool use_position_bias, bool use_velocity_bias);

   void update(const aaesim::open_source::AircraftState &aircraft_truth_state_vector_new,
               aaesim::open_source::AircraftState &aircraft_navigation_measurement_state_vector_new) override;

  private:
   void get_sa_values();

   // Input Data:
   int nacp;
   int nacv;
   int nicp, nicv;
   bool use_position_bias;
   bool use_velocity_bias;

   // Other Data:
   double x, y, z, xd, yd, zd;
   FirstOrderGaussMarkovProcess x_error, y_error;
   FirstOrderGaussMarkovProcess xd_error, yd_error;
   double x_bias, y_bias, z_bias;
   double xd_bias, yd_bias, zd_bias;
   Track baro_meas_to_assap;

   struct {
      // selective availability model parameters

      double dt{},  // update time interval

            sigmap{},  // position std. dev., SA on
            sigmav{},  // velocity std. dev., SA on

            p_err_north_out{},  // position error NED output
            p_err_east_out{}, p_err_down_out{},

            v_err_north_out{},  // velocity error NED output
            v_err_east_out{}, v_err_down_out{},

            tcorr{},  // correlation time constant, SA
            omg1{},   // natural frequency, SA

            phi11{},  // phi matrix
            phi12{}, phi21{}, phi22{},

            q11{},  // Q matrix
            q12{}, q21{}, q22{},

            u11{},                                      // U matrix
            u12{}, u21{}, u22{}, p_err_north{},         // position error NED
            p_err_east{}, p_err_down{}, v_err_north{},  // velocity error NED
            v_err_east{}, v_err_down{};

      double w_north[2], w_east[2], w_down[2];

   } sa_errors;

   void copy(const GaussMarkovNavigationSensor &in);

   bool mModelLoaded;

   static double BARO_ALT_ERR_GRAD[121];
   static double BARO_ALT_ERR_ALTITUDES[121];
   static double BARO_ALT_ERROR[121];
   static double BARO_GRADIENT_ERR;

   static log4cplus::Logger logger;
};
}  // namespace aaesim