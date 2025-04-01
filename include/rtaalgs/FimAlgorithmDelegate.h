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

#include "imalgs/IMTimeBasedAchieve.h"

namespace required_time_of_arrival {
class FimAlgorithmDelegate final : public interval_management::open_source::IMTimeBasedAchieve {
  public:
   FimAlgorithmDelegate() = default;

   void UpdateAssignedSpacingGoal(Units::SecondsTime asg_value) { m_assigned_spacing_goal = asg_value; }

   void InstantiateClassMembers() {
      m_im_kinematic_time_based_maintain =
            std::make_shared<interval_management::open_source::IMKinematicTimeBasedMaintain>();
   }

   void Initialize(const OwnshipPredictionParameters &ownship_prediction_parameters,
                   const AircraftIntent &ownship_aircraft_intent,
                   aaesim::open_source::WeatherPrediction &weather_prediction) override {
      m_target_aircraft_intent = m_im_clearance.GetTargetAircraftIntent();

      if (!m_target_aircraft_intent.IsLoaded()) {
         m_target_aircraft_intent = ownship_aircraft_intent;
      }

      if (m_target_aircraft_intent.GetRouteData().m_name.size() < m_target_aircraft_intent.GetNumberOfWaypoints()) {
         m_target_aircraft_intent.UpdateWaypoint(m_target_aircraft_intent.GetWaypoint(0));
      }

      IMKinematicAchieve::Initialize(ownship_prediction_parameters, ownship_aircraft_intent, weather_prediction);
   }

   AircraftIntent GetOwnshipAircraftIntent() const { return m_ownship_aircraft_intent; }
   Units::Time GetFimSpacingError() const { return m_predicted_spacing_interval - m_assigned_spacing_goal; }
   Units::Time GetTimeToGoToAbp() const { return m_ownship_ttg_to_abp; }

   void PrepareToRun(const aaesim::open_source::SimulationTime &simtime, const Units::Time required_time_of_arrival) {
      m_received_one_valid_target_state = true;
      m_compute_target_kinematic_trajectory = false;
      m_target_kinematic_trajectory_predictor = m_ownship_kinematic_trajectory_predictor;
      m_assigned_spacing_goal = required_time_of_arrival - simtime.GetCurrentSimulationTime();
   }
};
}  // namespace required_time_of_arrival