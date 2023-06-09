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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "aaesim/BadaPerformanceCalculator.h"
#include "public/Guidance.h"
#include "public/ControlCommands.h"
#include "public/EquationsOfMotionState.h"
#include "public/TangentPlaneSequence.h"
#include "public/WeatherTruth.h"

namespace aaesim {
namespace open_source {
class AircraftControl {
  public:
   AircraftControl();

   virtual void Initialize(std::shared_ptr<aaesim::BadaPerformanceCalculator> aircraft_performance,
                           const Units::Angle &max_bank_angle);

   ControlCommands CalculateControlCommands(const Guidance &guidance,
                                            const EquationsOfMotionState &equations_of_motion_state,
                                            const WeatherTruth &weather_truth);

   const Units::Frequency &GetAltGain() const { return m_alt_gain; }

   const Units::Frequency &GetGammaGain() const { return m_gamma_gain; }

   const Units::Frequency &GetPhiGain() const { return m_phi_gain; }

   double GetSpeedBrakeGain() const { return m_speed_brake_gain; }

   const Units::Frequency &GetThrustGain() const { return m_thrust_gain; }

  protected:
   /**
    * Use this to estimate the kinetic forces of lift and drag on the aircraft
    * in it's current state.
    */
   void ConfigureFlapsAndEstimateKineticForces(
         const EquationsOfMotionState &equations_of_motion_state, Units::Force &lift, Units::Force &drag,
         aaesim::open_source::bada_utils::FlapConfiguration &new_flap_configuration, const WeatherTruth &weather);

   virtual Units::Angle DoLateralControl(const Guidance &guidance,
                                         const EquationsOfMotionState &equations_of_motion_state);

   virtual void DoVerticalControl(const Guidance &guidance, const EquationsOfMotionState &eqmState, Units::Force &T_com,
                                  Units::Angle &gamma_com, Units::Speed &tas_com, double &speedBrakeCom,
                                  aaesim::open_source::bada_utils::FlapConfiguration &newFlapConfig,
                                  const WeatherTruth &weather){};

   void DoClimbingControl(const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
                          Units::Force &thrust_command, Units::Angle &gamma_command, Units::Speed &tas_command,
                          aaesim::open_source::bada_utils::FlapConfiguration &new_flap_config,
                          const WeatherTruth &weather_truth);

   void CalculateSensedWind(const WeatherTruth &wind, const Units::MetersLength &altitude_msl);

   Units::Frequency CalculateThrustGain();

   Units::Mass m_ac_mass;
   Units::Area m_wing_area;
   Units::Frequency m_alt_gain, m_gamma_gain, m_phi_gain, m_thrust_gain, m_natural_frequency;
   Units::Speed m_Vwx, m_Vwy;
   Units::Frequency m_dVwx_dh, m_dVwy_dh;
   double m_speed_brake_gain;
   Units::Angle m_max_bank_angle;  // Maximum bank angle for dynamics and speed_on_pitch_control_dynamics calculations
                                   // (parameter max_bank_angle)
   std::shared_ptr<aaesim::BadaPerformanceCalculator> m_bada_calculator;
   bool m_is_level_flight;

  private:
   static log4cplus::Logger m_logger;
};
}  // namespace open_source
}  // namespace aaesim
