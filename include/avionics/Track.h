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

#include "public/DVector.h"
#include "public/DMatrix.h"

class Track {
  public:
   Track(int size, double time, bool smoothing);

   ~Track() = default;

   Track(const Track &in);

   Track &operator=(const Track &in);

   void extrapolate(double dt, DVector &p_state, DMatrix &pcov, int maneuver);

   void SmoothPosition(double time, double *meas, double meas_var, int use_maneuver_det);

   void SmoothVelocity(double time, double *meas, double meas_var, int maneuver);

   void SmoothPositionAndVelocity(double time, double *measurement, double *var_meas, int maneuver);

   double process_noise_ft_per_sec_squared;
   int state_vector_size;
   int measurement_size;
   bool track_smoothing_enabled;
   int radar_tracker_manuvering_enabled;
   int number_smoothing_cycles_with_radar_tracker;
   int number_smoothing_cycles_with_adsb_tracker;

   double last_measurement_smoothing_time;
   DVector state;
   DMatrix covariance;
   DVector predicted_state;

   double acceleration;

  private:
   void copy(const Track &in);

   const double MANEUVER_PROCESS_NOISE_FT_PER_SEC_SQUARED{32.2};
   const double STEADY_STATE_PROCESS_NOISE_FT_PER_SEC_SQUARED{32.2};
   const double PV_UPDATE_DENOM_FACTOR{1.25};
};
