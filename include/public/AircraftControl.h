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


#include "aaesim/BadaWithCalc.h"
#include "public/Guidance.h"
#include "public/ControlCommands.h"
#include "public/EquationsOfMotionState.h"
#include "public/TangentPlaneSequence.h"
#include "public/WeatherTruth.h"

/**
 * This class is really intended to be sub-classes to allow specialized behavior and gain values.
 */
class AircraftControl
{

public:
   AircraftControl();

   virtual void Initialize(BadaWithCalc &aircraftPerformance,
                           const Units::Length &altAtFAF,
                           const Units::Angle &mMaxBankAngle,
                           const PrecalcWaypoint &finalWaypoint);

   /**
   * This should be reimplemented in a subclass.
   */
   virtual ControlCommands CalculateControlCommands(const Guidance &guidance,
                                                    const EquationsOfMotionState &eqmState,
                                                    const WeatherTruth &wind);

   const Units::Frequency &getAltGain() const {
      return m_alt_gain;
   }

   const Units::Frequency &getGammaGain() const {
      return m_gamma_gain;
   }

   const Units::Frequency &getPhiGain() const {
      return m_phi_gain;
   }

   double getSpeedBrakeGain() const {
      return m_speed_brake_gain;
   }

   const Units::Frequency &getThrustGain() const {
      return m_thrust_gain;
   }

protected:
   /**
    * Use this to estimate the kinetic forces of lift and drag on the aircraft
    * in it's current state.
    */
   void estimateKineticForces(const EquationsOfMotionState &eqmState,
                              Units::Force &lift,
                              Units::Force &drag,
                              int &newFlapConfiguration,
                              const WeatherTruth &weather);

   virtual Units::Angle doLateralControl(const Guidance &guidance,
                                         const EquationsOfMotionState &eqmState);

   virtual void DoVerticalControl(const Guidance &guidance,
                                  const EquationsOfMotionState &eqmState,
                                  Units::Force &T_com,
                                  Units::Angle &gamma_com,
                                  Units::Speed &tas_com,
                                  double &speedBrakeCom,
                                  int &newFlapConfig,
                                  const WeatherTruth &weather) {
   };

   void calculateSensedWind(const WeatherTruth &wind,
                            const Units::MetersLength &altitude);

   Units::Frequency calculateThrustGain();

   Units::Mass m_ac_mass;
   Units::Area m_wing_area;
   Units::Frequency m_alt_gain, m_gamma_gain, m_phi_gain, m_thrust_gain, m_natural_frequency;
   Units::Speed m_Vwx, m_Vwy;
   Units::Frequency m_dVwx_dh, m_dVwy_dh;
   double m_speed_brake_gain;
   Units::Angle m_max_bank_angle; // Maximum bank angle for dynamics and speed_on_pitch_control_dynamics calculations (parameter max_bank_angle)
   Units::Length m_alt_at_FAF;
   BadaWithCalc *m_bada_calculator;
   PrecalcWaypoint m_final_waypoint; // the last waypoint on the planned route

private:
   static log4cplus::Logger logger;
};



