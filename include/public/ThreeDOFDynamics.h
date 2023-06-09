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

#include <string>
#include <map>
#include "aaesim/BadaPerformanceCalculator.h"
#include "public/EquationsOfMotionStateDeriv.h"
#include "public/DynamicsState.h"
#include "public/AircraftControl.h"
#include "public/AircraftState.h"
#include "public/ControlCommands.h"
#include "public/Guidance.h"
#include "public/EquationsOfMotionState.h"
#include "public/LoggingLoadable.h"
#include <scalar/Angle.h>
#include <scalar/Force.h>
#include <scalar/Frequency.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>

namespace aaesim {
namespace open_source {
class ThreeDOFDynamics {
  public:
   ThreeDOFDynamics();

   /**
    * Aircraft update method that calculates the new aircraft state from the given command state
    *
    * @param aircraft_state
    * @param guidance
    * @param aircraft_control
    * @return
    */
   virtual AircraftState Update(const Guidance &guidance, const std::shared_ptr<AircraftControl> &aircraft_control);

   virtual void Initialize(std::shared_ptr<const BadaPerformanceCalculator> aircraft_performance,
                           const Waypoint &initial_position,
                           std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                           Units::Length initial_altitude_msl, Units::Speed initial_true_airspeed,
                           Units::Angle initial_ground_course_enu, double initial_mass_fraction,
                           const WeatherTruth &true_weather);

   /**
    * @return the pair <east-component, north-component>
    */
   const std::pair<Units::Speed, Units::Speed> GetWindComponents() const;

   const WeatherTruth GetTruthWeather() const;

   const DynamicsState GetDynamicsState() const;

   const EquationsOfMotionState GetEquationsOfMotionState() const;

   const EquationsOfMotionStateDeriv GetEquationsOfMotionStateDerivative() const;

  private:
   static log4cplus::Logger m_logger;

  protected:
   virtual AircraftState Integrate(const Guidance &guidance, const std::shared_ptr<AircraftControl> &aircraft_control);

   // Calculate the trim angle correction necessary and provides an updated state
   virtual Units::SignedRadiansAngle CalculateTrimmedPsiForWind(Units::SignedAngle ground_track_enu);

   virtual EquationsOfMotionStateDeriv StatePropagation(Units::Frequency dVwx_dh, Units::Frequency dVwy_dh,
                                                        Units::Frequency k_gamma, Units::Frequency k_t,
                                                        Units::Frequency k_phi, double k_speedBrake,
                                                        ControlCommands commands);

   virtual EquationsOfMotionStateDeriv StatePropagationOnRunway(ControlCommands commands, const Guidance &guidance);

   virtual void CalculateKineticForces(Units::Force &lift, Units::Force &drag);

   virtual void CalculateEnvironmentalWind(WindStack &wind_east, WindStack &wind_north, Units::Frequency &dVwx_dh,
                                           Units::Frequency &dVwy_dh) = 0;

   std::shared_ptr<const aaesim::BadaPerformanceCalculator> m_bada_calculator;
   DynamicsState m_dynamics_state;
   EquationsOfMotionState m_equations_of_motion_state;
   EquationsOfMotionStateDeriv m_equations_of_motion_state_derivative;

   // True wind direction values computed by dynamics and speed_on_pitch_control_dynamics
   Units::Speed m_wind_velocity_east;
   Units::Speed m_wind_velocity_north;

   double m_max_thrust_percent;
   double m_min_thrust_percent;
   WeatherTruth m_true_weather;
};

inline const std::pair<Units::Speed, Units::Speed> ThreeDOFDynamics::GetWindComponents() const {
   return std::make_pair(m_wind_velocity_east, m_wind_velocity_north);
}

inline const WeatherTruth ThreeDOFDynamics::GetTruthWeather() const { return m_true_weather; }

inline const DynamicsState ThreeDOFDynamics::GetDynamicsState() const { return m_dynamics_state; }

inline const EquationsOfMotionState ThreeDOFDynamics::GetEquationsOfMotionState() const {
   return m_equations_of_motion_state;
}

inline const EquationsOfMotionStateDeriv ThreeDOFDynamics::GetEquationsOfMotionStateDerivative() const {
   return m_equations_of_motion_state_derivative;
}

}  // namespace open_source
}  // namespace aaesim