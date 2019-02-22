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


#include "public/AircraftControl.h"


class SpeedOnThrustControl : public AircraftControl
{

public:

   SpeedOnThrustControl();

   virtual void Initialize(BadaWithCalc &bada_calculator,
                           const Units::Length &altitude_at_final_waypoint,
                           const Units::Angle &max_bank_angle,
                           const PrecalcWaypoint &final_waypoint);

   ControlCommands CalculateControlCommands(const Guidance &guidance,
                                            const EquationsOfMotionState &eqmState,
                                            const WeatherTruth &truth_wind);

private:

   static log4cplus::Logger m_logger;
   static Units::Frequency m_gain_altitude, m_gain_gamma, m_gain_phi;
   static double m_gain_speedbrake;
   Units::Frequency m_gain_velocity;


protected:
   virtual void DoVerticalControl(const Guidance &guidance,
                                  const EquationsOfMotionState &eqmState,
                                  Units::Force &thrust_command,
                                  Units::Angle &gamma_com,
                                  Units::Speed &tas_com,
                                  double &speed_brake_command,
                                  int &newFlapConfig,
                                  const WeatherTruth &weather);

   double m_min_thrust_counter;
   double m_speedbrake_counter;
   bool m_is_speedbrake_on;
};


