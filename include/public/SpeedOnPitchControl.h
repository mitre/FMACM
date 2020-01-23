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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once


#include "public/AircraftControl.h"

class SpeedOnPitchControl : public AircraftControl
{
public:

   SpeedOnPitchControl(const Units::Speed speedThreshold,
                       const Units::Length altitudeThreshold);

   SpeedOnPitchControl() { };

   ControlCommands CalculateControlCommands(const Guidance &guidance,
                                            const EquationsOfMotionState &eqmState,
                                            const WeatherTruth &wind);

   virtual void Initialize(BadaWithCalc &bada_calculator,
                           const Units::Length &altitude_msl_at_faf,
                           const Units::Angle &max_bank_angle,
                           const PrecalcWaypoint &final_waypoint);

private:

   static log4cplus::Logger m_logger;

   // Control gains
   static Units::Frequency m_gain_alt, m_gain_gamma, m_gain_phi;
   static double m_gain_speedbrake;
   Units::Frequency m_gain_velocity;
   Units::Length m_altitude_threshold;
   Units::Speed m_speed_threshold;
   int m_min_thrust_counter;
   int m_speed_brake_counter;
   bool m_is_speed_brake_on;
   bool m_is_level_flight;

protected:
   virtual void DoVerticalControl(const Guidance &guidance,
                                  const EquationsOfMotionState &eqmState,
                                  Units::Force &thrust_command,
                                  Units::Angle &gamma_command,
                                  Units::Speed &true_airspeed_command,
                                  double &speed_brake_command,
                                  int &new_flap_config,
                                  const WeatherTruth &true_weather);
};
