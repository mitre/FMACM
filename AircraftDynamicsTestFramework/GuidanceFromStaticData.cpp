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

#include "framework/GuidanceFromStaticData.h"

#include <scalar/AngularSpeed.h>

#include "public/CoreUtils.h"

using namespace fmacm;
using namespace aaesim::open_source;

log4cplus::Logger GuidanceFromStaticData::m_logger = log4cplus::Logger::getInstance("GuidanceFromStaticData");

GuidanceFromStaticData::GuidanceFromStaticData()
   : m_vertical_data(),
     m_horizontal_trajectory(),
     m_decrementing_distance_calculator(),
     m_decrementing_position_calculator(),
     m_estimated_distance_to_go(Units::infinity()) {}

GuidanceFromStaticData::GuidanceFromStaticData(const std::vector<HorizontalPath> &horizontal_path,
                                               const VerticalData &vertcal_path,
                                               const PlannedDescentParameters &planned_descent_parameters) {
   m_horizontal_trajectory = horizontal_path;
   m_vertical_data = vertcal_path;
   m_estimated_distance_to_go = Units::infinity();
   m_decrementing_distance_calculator =
         AlongPathDistanceCalculator(m_horizontal_trajectory, TrajectoryIndexProgressionDirection::DECREMENTING);
   m_decrementing_position_calculator =
         PositionCalculator(m_horizontal_trajectory, TrajectoryIndexProgressionDirection::DECREMENTING);
   m_planned_descent_parameters = planned_descent_parameters;
}

aaesim::open_source::Guidance GuidanceFromStaticData::Update(const aaesim::open_source::AircraftState &state) {
   Units::UnsignedAngle estimated_course;
   Units::Length estimated_distance_to_go;
   m_decrementing_distance_calculator.CalculateAlongPathDistanceFromPosition(
         state.GetPositionEnuX(), state.GetPositionEnuY(), estimated_distance_to_go, estimated_course);
   if (estimated_distance_to_go >= m_estimated_distance_to_go) {
      LOG4CPLUS_WARN(
            m_logger,
            "The distance-to-go parameter is not decreasing. This usually indicates a simulation problem. Check "
            "the horizontal data input file for validity...Trying to continue.");
      return m_previous_guidance;
   }

   auto vertical_guidance = CalculateVerticalGuidance(state, estimated_distance_to_go, estimated_course);
   auto horizontal_guidance = CalculateHorizontalGuidance(state, estimated_distance_to_go, estimated_course);
   auto complete_guidance = CombineGuidance(horizontal_guidance, vertical_guidance);
   m_previous_guidance = complete_guidance;
   m_estimated_distance_to_go = estimated_distance_to_go;
   return complete_guidance;
}

aaesim::open_source::Guidance GuidanceFromStaticData::CalculateVerticalGuidance(
      const aaesim::open_source::AircraftState &state, const Units::MetersLength &estimated_distance_to_go,
      const Units::UnsignedAngle &estimated_course) {

   aaesim::open_source::Guidance vertical_guidance;
   vertical_guidance.m_reference_altitude = state.GetAltitudeMsl();
   vertical_guidance.m_vertical_speed = Units::ZERO_SPEED;
   vertical_guidance.m_ias_command = state.GetDynamicsState().v_indicated_airspeed;
   vertical_guidance.m_ground_speed = state.GetGroundSpeed();
   vertical_guidance.m_active_guidance_phase = aaesim::open_source::GuidanceFlightPhase::CRUISE_DESCENT;

   if (estimated_distance_to_go.value() <= fabs(m_vertical_data.m_distance_to_go_meters.back())) {
      double altitude_target;
      double ias_target;
      double altitude_rate_target;
      double groundspeed_target;
      int current_trajectory_index =
            CoreUtils::FindNearestIndex(estimated_distance_to_go.value(), m_vertical_data.m_distance_to_go_meters);

      if (current_trajectory_index == 0) {
         altitude_target = m_vertical_data.m_altitude_meters[current_trajectory_index];
         ias_target = m_vertical_data.m_ias_mps[current_trajectory_index];
         altitude_rate_target = m_vertical_data.m_vertical_speed_mps[current_trajectory_index];
         groundspeed_target = m_vertical_data.m_ground_speed_mps[current_trajectory_index];
      } else {
         altitude_target = CoreUtils::LinearlyInterpolate(current_trajectory_index, estimated_distance_to_go.value(),
                                                          m_vertical_data.m_distance_to_go_meters,
                                                          m_vertical_data.m_altitude_meters);
         ias_target =
               CoreUtils::LinearlyInterpolate(current_trajectory_index, estimated_distance_to_go.value(),
                                              m_vertical_data.m_distance_to_go_meters, m_vertical_data.m_ias_mps);
         altitude_rate_target = CoreUtils::LinearlyInterpolate(
               current_trajectory_index, estimated_distance_to_go.value(), m_vertical_data.m_distance_to_go_meters,
               m_vertical_data.m_vertical_speed_mps);
         groundspeed_target = CoreUtils::LinearlyInterpolate(current_trajectory_index, estimated_distance_to_go.value(),
                                                             m_vertical_data.m_distance_to_go_meters,
                                                             m_vertical_data.m_ground_speed_mps);
      }

      vertical_guidance.m_reference_altitude = Units::MetersLength(altitude_target);
      vertical_guidance.m_vertical_speed = Units::MetersPerSecondSpeed(altitude_rate_target);
      vertical_guidance.m_ias_command = Units::MetersPerSecondSpeed(ias_target);
      vertical_guidance.m_ground_speed = Units::MetersPerSecondSpeed(groundspeed_target);
      if (state.GetAltitudeMsl() >= m_planned_descent_parameters.planned_transition_altitude) {
         vertical_guidance.SetSelectedSpeed(aaesim::open_source::AircraftSpeed::OfMach(
               BoundedValue<double, 0, 1>(m_planned_descent_parameters.planned_cruise_mach)));
      } else {
         vertical_guidance.SetSelectedSpeed(
               aaesim::open_source::AircraftSpeed::OfIndicatedAirspeed(vertical_guidance.m_ias_command));
      }
   }
   return vertical_guidance;
}

aaesim::open_source::Guidance GuidanceFromStaticData::CalculateHorizontalGuidance(
      const aaesim::open_source::AircraftState &state, const Units::MetersLength &estimated_distance_to_go,
      const Units::UnsignedAngle &estimated_course) {
   aaesim::open_source::Guidance horizontal_guidance;
   horizontal_guidance.m_cross_track_error = Units::ZERO_LENGTH;
   horizontal_guidance.m_use_cross_track = false;
   horizontal_guidance.m_reference_bank_angle = Units::ZERO_ANGLE;
   horizontal_guidance.m_enu_track_angle = Units::ZERO_ANGLE;

   Units::UnsignedAngle course_at_position;
   Units::MetersLength estimated_position_on_path_x, estimated_position_on_path_y;
   m_decrementing_position_calculator.CalculatePositionFromAlongPathDistance(
         m_estimated_distance_to_go, estimated_position_on_path_x, estimated_position_on_path_y, course_at_position);
   auto traj_index = m_decrementing_position_calculator.GetCurrentTrajectoryIndex();
   if (traj_index == m_horizontal_trajectory.size() - 1) {
      traj_index--;
   }

   horizontal_guidance.m_enu_track_angle = course_at_position;
   if (horizontal_guidance.m_ground_speed <= Units::zero()) horizontal_guidance.m_ground_speed = state.GetGroundSpeed();
   if (m_horizontal_trajectory[traj_index].m_segment_type == HorizontalPath::SegmentType::TURN) {
      Units::Length unsigned_cross_track =
            Units::sqrt(Units::sqr(state.GetPositionEnuX() - estimated_position_on_path_x) +
                        Units::sqr(state.GetPositionEnuY() - estimated_position_on_path_y));

      Units::Length center_distance = Units::sqrt(
            Units::sqr(state.GetPositionEnuX() -
                       Units::MetersLength(m_horizontal_trajectory[traj_index].m_turn_info.x_position_meters)) +
            Units::sqr(state.GetPositionEnuY() -
                       Units::MetersLength(m_horizontal_trajectory[traj_index].m_turn_info.y_position_meters)));

      Units::FeetLength distance_to_waypoint =
            m_estimated_distance_to_go -
            Units::MetersLength(m_horizontal_trajectory[traj_index].m_path_length_cumulative_meters);
      Units::SecondsTime time_to_waypoint = distance_to_waypoint / horizontal_guidance.m_ground_speed;

      static const Units::DegreesPerSecondAngularSpeed roll_rate(3.0);
      double dimensionless_roll_factor = 1;
      if (traj_index > 0 && (m_horizontal_trajectory[traj_index - 1].m_turn_info.radius.value() < 1)) {
         Units::SecondsTime time_to_bank = m_horizontal_trajectory[traj_index].m_turn_info.bankAngle / roll_rate;

         if (time_to_waypoint <= time_to_bank) {
            dimensionless_roll_factor = time_to_waypoint / time_to_bank;
         }
      }

      Units::Angle aircraft_course = Units::UnsignedRadiansAngle(
            Units::RadiansAngle(m_horizontal_trajectory[traj_index].m_path_course) + Units::PI_RADIANS_ANGLE);
      Units::SignedRadiansAngle course_change = Units::ToSigned(estimated_course - aircraft_course);

      TurnDirection turn_direction = GetTurnDirection(course_change);
      // courseChange - positive is left turn, neg is right turn
      // if left turn, distance < radius is left of m_path_course, distance > radius is right of m_path_course
      // if right turn, distance < radius is right of m_path_course, distance > radius is left of m_path_course
      if (turn_direction == LEFT) {
         horizontal_guidance.m_reference_bank_angle =
               dimensionless_roll_factor * m_horizontal_trajectory[traj_index].m_turn_info.bankAngle;

         if (center_distance < Units::MetersLength(m_horizontal_trajectory[traj_index].m_turn_info.radius)) {
            horizontal_guidance.m_cross_track_error = Units::MetersLength(unsigned_cross_track);
         } else {
            horizontal_guidance.m_cross_track_error = Units::MetersLength(-unsigned_cross_track);
         }
      } else {
         horizontal_guidance.m_reference_bank_angle =
               -dimensionless_roll_factor * m_horizontal_trajectory[traj_index].m_turn_info.bankAngle;

         if (center_distance < Units::MetersLength(m_horizontal_trajectory[traj_index].m_turn_info.radius)) {
            horizontal_guidance.m_cross_track_error = Units::MetersLength(-unsigned_cross_track);
         } else {
            horizontal_guidance.m_cross_track_error = Units::MetersLength(unsigned_cross_track);
         }
      }
   } else {
      horizontal_guidance.m_cross_track_error =
            -(state.GetPositionEnuY() - Units::MetersLength(m_horizontal_trajectory[traj_index].GetYPositionMeters())) *
                  Units::cos(estimated_course) +
            (state.GetPositionEnuX() - Units::MetersLength(m_horizontal_trajectory[traj_index].GetXPositionMeters())) *
                  Units::sin(estimated_course);
   }

   horizontal_guidance.m_use_cross_track = true;
   return horizontal_guidance;
}
