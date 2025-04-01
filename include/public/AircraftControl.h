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

#include "public/FixedMassAircraftPerformance.h"
#include "public/Guidance.h"
#include "public/ControlCommands.h"
#include "public/EquationsOfMotionState.h"
#include "public/TrueWeatherOperator.h"

namespace aaesim {
namespace open_source {
class AircraftControl {
  public:
   AircraftControl(const Units::Angle max_bank_angle);
   virtual ~AircraftControl() = default;

   virtual void Initialize(std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> aircraft_performance);

   ControlCommands CalculateControlCommands(
         const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
         std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> sensed_weather);

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
         aaesim::open_source::bada_utils::FlapConfiguration &new_flap_configuration);

   virtual Units::Angle DoLateralControl(const Guidance &guidance,
                                         const EquationsOfMotionState &equations_of_motion_state);

   virtual void DoVerticalControl(const Guidance &guidance, const EquationsOfMotionState &eqmState, Units::Force &T_com,
                                  Units::Angle &gamma_com, Units::Speed &tas_com, double &speedBrakeCom,
                                  aaesim::open_source::bada_utils::FlapConfiguration &newFlapConfig){};

   void DoClimbingControl(const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
                          Units::Force &thrust_command, Units::Angle &gamma_command, Units::Speed &tas_command,
                          aaesim::open_source::bada_utils::FlapConfiguration &new_flap_config);

   Units::Frequency CalculateThrustGain();

   Units::Mass m_ac_mass;
   Units::Area m_wing_area;
   Units::Frequency m_alt_gain, m_gamma_gain, m_phi_gain, m_thrust_gain, m_natural_frequency;
   Units::Speed m_Vwx, m_Vwy;
   Units::Frequency m_dVwx_dh, m_dVwy_dh;
   double m_speed_brake_gain;
   std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> m_bada_calculator;
   Units::Angle m_max_bank_angle;
   bool m_is_level_flight;
   std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> m_sensed_weather;

  private:
   static log4cplus::Logger m_logger;
   void CalculateSensedWind(std::shared_ptr<const aaesim::open_source::TrueWeatherOperator> &sensed_weather,
                            const Units::MetersLength &altitude_msl);
};
}  // namespace open_source
}  // namespace aaesim
