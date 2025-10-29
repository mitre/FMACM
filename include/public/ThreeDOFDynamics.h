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

#include <scalar/Angle.h>
#include <scalar/Force.h>
#include <scalar/Frequency.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>

#include <map>
#include <string>

#include "public/AircraftControl.h"
#include "public/AircraftState.h"
#include "public/DynamicsState.h"
#include "public/EllipsoidalPositionEstimator.h"
#include "public/EquationsOfMotionState.h"
#include "public/EquationsOfMotionStateDeriv.h"
#include "public/FixedMassAircraftPerformance.h"
#include "public/Guidance.h"
#include "public/SimulationTime.h"
#include "public/TrueWeatherOperator.h"

namespace aaesim::open_source {
class ThreeDOFDynamics final {
  public:
   ThreeDOFDynamics() = default;
   ~ThreeDOFDynamics() = default;

   AircraftState Update(const int unique_acid, const aaesim::open_source::SimulationTime &simtime,
                        const Guidance &guidance, const std::shared_ptr<AircraftControl> &aircraft_control);

   void Initialize(const aaesim::open_source::SimulationTime &simulation_time,
                   std::shared_ptr<const aaesim::open_source::FixedMassAircraftPerformance> aircraft_performance,
                   const EarthModel::GeodeticPosition &initial_position,
                   const EarthModel::LocalPositionEnu &initial_position_enu, Units::Length initial_altitude_msl,
                   Units::Speed initial_true_airspeed, Units::Angle initial_ground_course_enu,
                   double initial_mass_fraction,
                   std::shared_ptr<aaesim::open_source::EllipsoidalPositionEstimator> position_estimator,
                   std::shared_ptr<aaesim::open_source::TrueWeatherOperator> true_weather_operator);

   const std::pair<Units::Speed, Units::Speed> GetWindComponents() const;

   const DynamicsState GetDynamicsState() const;

   const EquationsOfMotionState GetEquationsOfMotionState() const;

   const EquationsOfMotionStateDeriv GetEquationsOfMotionStateDerivative() const;

   std::map<const aaesim::open_source::SimulationTime, const DynamicsState> GetDynamicsStateHistory() const;

  private:
   inline static log4cplus::Logger m_logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("ThreeDOFDynamics"))};

   DynamicsState Integrate(const Guidance &guidance, const std::shared_ptr<AircraftControl> &aircraft_control);

   // Calculate the trim angle correction necessary and provides an updated state
   Units::SignedRadiansAngle CalculateTrimmedPsiForWind(Units::SignedAngle ground_track_enu);

   EquationsOfMotionStateDeriv StatePropagation(Units::Frequency dVwx_dh, Units::Frequency dVwy_dh,
                                                Units::Frequency k_gamma, Units::Frequency k_t, Units::Frequency k_phi,
                                                double k_speedBrake, ControlCommands commands);

   EquationsOfMotionStateDeriv StatePropagationOnRunway(ControlCommands commands, const Guidance &guidance);

   void CalculateKineticForces(Units::Force &lift, Units::Force &drag);

   void UpdateTrueWeatherConditions();

   DynamicsState ComputeDynamicsState(const EquationsOfMotionState &equations_of_motion_state,
                                      const EquationsOfMotionStateDeriv &equations_of_motion_state_derivative) const;

   std::shared_ptr<const aaesim::open_source::FixedMassAircraftPerformance> m_bada_calculator{};
   std::shared_ptr<aaesim::open_source::EllipsoidalPositionEstimator> m_position_estimator;
   std::map<const aaesim::open_source::SimulationTime, const DynamicsState> m_dynamics_history{};
   EquationsOfMotionState m_equations_of_motion_state{};
   EquationsOfMotionStateDeriv m_equations_of_motion_state_derivative{};
   EarthModel::GeodeticPosition m_last_resolved_position{};
   Units::Speed m_wind_velocity_east{Units::zero()};
   Units::Speed m_wind_velocity_north{Units::zero()};
   double m_max_thrust_percent{1.0};
   double m_min_thrust_percent{1.0};
   std::shared_ptr<aaesim::open_source::TrueWeatherOperator> m_true_weather_operator;
};

inline const std::pair<Units::Speed, Units::Speed> ThreeDOFDynamics::GetWindComponents() const {
   return std::make_pair(m_wind_velocity_east, m_wind_velocity_north);
}

inline const DynamicsState ThreeDOFDynamics::GetDynamicsState() const {
   if (m_dynamics_history.empty()) return DynamicsState{};
   return std::prev(m_dynamics_history.cend())->second;
}

inline const EquationsOfMotionState ThreeDOFDynamics::GetEquationsOfMotionState() const {
   return m_equations_of_motion_state;
}

inline const EquationsOfMotionStateDeriv ThreeDOFDynamics::GetEquationsOfMotionStateDerivative() const {
   return m_equations_of_motion_state_derivative;
}

inline std::map<const aaesim::open_source::SimulationTime, const DynamicsState>
      ThreeDOFDynamics::GetDynamicsStateHistory() const {
   return m_dynamics_history;
}

}  // namespace aaesim::open_source