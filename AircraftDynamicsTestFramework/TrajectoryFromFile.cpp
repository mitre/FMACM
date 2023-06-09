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

#include "framework/TrajectoryFromFile.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"
#include "framework/HfpReader2020.h"
#include "utility/CsvParser.h"

#include <scalar/AngularSpeed.h>

TrajectoryFromFile::TrajectoryFromFile()
   : m_vertical_data(),
     m_horizontal_trajectory(),
     m_precalc_waypoints(),
     m_decrementing_distance_calculator(),
     m_decrementing_position_calculator(),
     estimated_distance_to_go(),
     m_vertical_trajectory_file(),
     m_horizontal_trajectory_file(),
     m_mass_percentile(0.5),
     m_loaded(false) {}

TrajectoryFromFile::~TrajectoryFromFile() = default;

bool TrajectoryFromFile::load(DecodedStream *input) {

   set_stream(input);

   register_var("vfp_csv_file", &m_vertical_trajectory_file);
   register_var("hfp_csv_file", &m_horizontal_trajectory_file);

   m_loaded = complete();

   if (m_loaded) {
      ReadVerticalTrajectoryFile();
      ReadHorizontalTrajectoryFile();
   }

   return m_loaded;
}

aaesim::open_source::Guidance TrajectoryFromFile::Update(const aaesim::open_source::AircraftState &state) {
   aaesim::open_source::Guidance result;

   Units::UnsignedAngle estimated_course;
   m_decrementing_distance_calculator.CalculateAlongPathDistanceFromPosition(
         Units::FeetLength(state.m_x), Units::FeetLength(state.m_y), estimated_distance_to_go, estimated_course);

   if (estimated_distance_to_go.value() <= fabs(m_vertical_data.m_distance_to_go_meters.back())) {
      double h_next;
      double v_next;
      double h_dot_next;
      double gs_next;
      int current_trajectory_index =
            CoreUtils::FindNearestIndex(estimated_distance_to_go.value(), m_vertical_data.m_distance_to_go_meters);

      if (current_trajectory_index == 0) {
         h_next = m_vertical_data.m_altitude_meters[current_trajectory_index];
         v_next = m_vertical_data.m_ias_mps[current_trajectory_index];
         h_dot_next = m_vertical_data.m_vertical_speed_mps[current_trajectory_index];
         gs_next = m_vertical_data.m_ground_speed_mps[current_trajectory_index];
      } else {
         h_next = CoreUtils::LinearlyInterpolate(current_trajectory_index, estimated_distance_to_go.value(),
                                                 m_vertical_data.m_distance_to_go_meters,
                                                 m_vertical_data.m_altitude_meters);
         v_next = CoreUtils::LinearlyInterpolate(current_trajectory_index, estimated_distance_to_go.value(),
                                                 m_vertical_data.m_distance_to_go_meters, m_vertical_data.m_ias_mps);
         h_dot_next = CoreUtils::LinearlyInterpolate(current_trajectory_index, estimated_distance_to_go.value(),
                                                     m_vertical_data.m_distance_to_go_meters,
                                                     m_vertical_data.m_vertical_speed_mps);
         gs_next = CoreUtils::LinearlyInterpolate(current_trajectory_index, estimated_distance_to_go.value(),
                                                  m_vertical_data.m_distance_to_go_meters,
                                                  m_vertical_data.m_ground_speed_mps);
      }

      result.m_reference_altitude = Units::MetersLength(h_next);
      result.m_vertical_speed = Units::MetersPerSecondSpeed(h_dot_next);
      result.m_ias_command = Units::MetersPerSecondSpeed(v_next);
      result.m_ground_speed = Units::MetersPerSecondSpeed(gs_next);
   }

   Units::UnsignedAngle course_at_position;
   Units::MetersLength estimated_position_on_path_x, estimated_position_on_path_y;
   m_decrementing_position_calculator.CalculatePositionFromAlongPathDistance(
         estimated_distance_to_go, estimated_position_on_path_x, estimated_position_on_path_y, course_at_position);
   auto traj_index = m_decrementing_position_calculator.GetCurrentTrajectoryIndex();
   if (traj_index == m_horizontal_trajectory.size() - 1) {
      traj_index--;
   }

   result.m_enu_track_angle = course_at_position;

   double unsigned_cross_track_meters = sqrt(pow(state.m_x * FEET_TO_METERS - estimated_position_on_path_x.value(), 2) +
                                             pow(state.m_y * FEET_TO_METERS - estimated_position_on_path_y.value(), 2));

   double center_dist_meters =
         sqrt(pow(state.m_x * FEET_TO_METERS - m_horizontal_trajectory[traj_index].m_turn_info.x_position_meters, 2) +
              pow(state.m_y * FEET_TO_METERS - m_horizontal_trajectory[traj_index].m_turn_info.y_position_meters, 2));

   if (m_horizontal_trajectory[traj_index].m_segment_type == HorizontalPath::SegmentType::TURN) {
      Units::FeetLength distance_to_waypoint =
            estimated_distance_to_go -
            Units::MetersLength(m_horizontal_trajectory[traj_index].m_path_length_cumulative_meters);
      Units::SecondsTime time_to_waypoint = distance_to_waypoint / result.m_ground_speed;

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
      Units::SignedRadiansAngle course_change = AircraftCalculations::Convert0to2Pi(estimated_course - aircraft_course);

      TurnDirection turn_direction = GetTurnDirection(course_change);
      // courseChange - positive is left turn, neg is right turn
      // if left turn, distance < radius is left of m_path_course, distance > radius is right of m_path_course
      // if right turn, distance < radius is right of m_path_course, distance > radius is left of m_path_course
      if (turn_direction == LEFT) {
         result.m_reference_bank_angle =
               dimensionless_roll_factor * m_horizontal_trajectory[traj_index].m_turn_info.bankAngle;

         if (center_dist_meters <
             Units::MetersLength(m_horizontal_trajectory[traj_index].m_turn_info.radius).value())  // left of
                                                                                                   // m_path_course
         {
            result.m_cross_track_error = Units::MetersLength(unsigned_cross_track_meters);
         } else {
            result.m_cross_track_error = Units::MetersLength(-unsigned_cross_track_meters);
         }
      } else {
         result.m_reference_bank_angle =
               -dimensionless_roll_factor * m_horizontal_trajectory[traj_index].m_turn_info.bankAngle;

         if (center_dist_meters < Units::MetersLength(m_horizontal_trajectory[traj_index].m_turn_info.radius).value()) {
            result.m_cross_track_error = Units::MetersLength(-unsigned_cross_track_meters);
         } else {
            result.m_cross_track_error = Units::MetersLength(unsigned_cross_track_meters);
         }
      }
   } else {
      result.m_cross_track_error = Units::MetersLength(
            -(state.m_y * FEET_TO_METERS - m_horizontal_trajectory[traj_index].GetYPositionMeters()) *
                  cos(estimated_course) +
            (state.m_x * FEET_TO_METERS - m_horizontal_trajectory[traj_index].GetXPositionMeters()) *
                  sin(estimated_course));
   }

   result.m_use_cross_track = true;

   return result;
}

void TrajectoryFromFile::CalculateWaypoints(AircraftIntent &intent) {
   m_precalc_waypoints.clear();

   double prev_dist = 0;

   for (int loop = intent.GetNumberOfWaypoints() - 1; loop > 0; loop--) {
      double delta_x = intent.GetRouteData().m_x[loop - 1].value() - intent.GetRouteData().m_x[loop].value();
      double delta_y = intent.GetRouteData().m_y[loop - 1].value() - intent.GetRouteData().m_y[loop].value();
      double leg_length_meters = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
      double course = atan2(delta_y, delta_x);

      PrecalcWaypoint new_waypoint;
      new_waypoint.m_leg_length = Units::MetersLength(leg_length_meters);
      new_waypoint.m_course_angle = Units::RadiansAngle(course);
      new_waypoint.m_name = intent.GetRouteData().m_name[loop];
      new_waypoint.m_x_pos_meters = intent.GetRouteData().m_x[loop];
      new_waypoint.m_y_pos_meters = intent.GetRouteData().m_y[loop];

      new_waypoint.m_precalc_constraints.constraint_along_path_distance =
            Units::MetersLength(leg_length_meters + prev_dist);
      /* new_waypoint.m_precalc_constraints.constraint_altHi =
            intent.GetRouteData().m_high_altitude_constraint[loop - 1];
      new_waypoint.m_precalc_constraints.constraint_altLow =
            intent.GetRouteData().m_low_altitude_constraint[loop - 1];
      new_waypoint.m_precalc_constraints.constraint_speedHi = intent.GetRouteData().m_high_speed_constraint[loop - 1];
      new_waypoint.m_precalc_constraints.constraint_speedLow = intent.GetRouteData().m_low_speed_constraint[loop - 1];
    */

      prev_dist += leg_length_meters;

      m_precalc_waypoints.push_back(new_waypoint);
   }

   // add the final waypoint
   PrecalcWaypoint new_waypoint;
   new_waypoint.m_name = intent.GetRouteData().m_name[0];
   new_waypoint.m_leg_length = Units::MetersLength(0);
   new_waypoint.m_course_angle = m_precalc_waypoints.back().m_course_angle;
   new_waypoint.m_x_pos_meters = intent.GetRouteData().m_x[0];
   new_waypoint.m_y_pos_meters = intent.GetRouteData().m_y[0];
   new_waypoint.m_precalc_constraints.constraint_along_path_distance = Units::MetersLength(prev_dist);
   /*   new_waypoint.m_precalc_constraints.constraint_altHi = intent.GetRouteData().m_high_altitude_constraint[0];
      new_waypoint.m_precalc_constraints.constraint_altLow = intent.GetRouteData().m_low_altitude_constraint[0];
      new_waypoint.m_precalc_constraints.constraint_speedHi = intent.GetRouteData().m_high_speed_constraint[0];
      new_waypoint.m_precalc_constraints.constraint_speedLow = intent.GetRouteData().m_low_speed_constraint[0];*/
   m_precalc_waypoints.push_back(new_waypoint);
}

void TrajectoryFromFile::ReadVerticalTrajectoryFile() {

   std::ifstream file(m_vertical_trajectory_file.c_str());

   if (!file.is_open()) {
      std::cout << "Vertical trajectory file " << m_vertical_trajectory_file.c_str() << " not found" << std::endl;
      exit(-20);
   }

   int numhdrs = 0;

   for (CsvParser::CsvIterator csviter(file); csviter != CsvParser::CsvIterator(); ++csviter) {

      if (numhdrs < 1) {
         numhdrs++;
         continue;
      }

      if ((*csviter).Size() != NUM_VERTICAL_TRAJ_FIELDS) {
         std::cout << "Bad number of fields found in " << m_vertical_trajectory_file.c_str() << std::endl
                   << "vertical trajectory file.  Found " << (*csviter).Size() << " fields expected "
                   << NUM_VERTICAL_TRAJ_FIELDS << " fields." << std::endl;
         exit(-21);
      }

      for (int vfield = TIME_TO_GO_SEC; vfield != NUM_VERTICAL_TRAJ_FIELDS; vfield++) {
         std::string fieldstr = (*csviter)[vfield];

         double val = atof(fieldstr.c_str());

         switch (static_cast<VerticalFields>(vfield)) {
            case TIME_TO_GO_SEC:
               m_vertical_data.m_time_to_go_sec.push_back(val);
               break;

            case DISTANCE_TO_GO_VERT_M:
               m_vertical_data.m_distance_to_go_meters.push_back(val);
               break;

            case ALTITUDE_M:
               m_vertical_data.m_altitude_meters.push_back(val);
               break;

            case IAS_MPS:
               m_vertical_data.m_ias_mps.push_back(val);
               break;

            case DOT_ALTITUDE_MPS:
               m_vertical_data.m_vertical_speed_mps.push_back(val);
               break;

            case GS_MPS:
               m_vertical_data.m_ground_speed_mps.push_back(val);
            default:
               break;
         }
      }
   }

   file.close();
}

void TrajectoryFromFile::ReadHorizontalTrajectoryFile() {
   testvector::HfpReader2020 hfp_reader(m_horizontal_trajectory_file, 1);

   while (hfp_reader.Advance()) {
      HorizontalPath horizontal_path_segment;

      horizontal_path_segment.SetXYPositionMeters(Units::MetersLength(hfp_reader.GetX()).value(),
                                                  Units::MetersLength(hfp_reader.GetY()).value());

      horizontal_path_segment.m_path_length_cumulative_meters = Units::MetersLength(hfp_reader.GetDTG()).value();
      horizontal_path_segment.m_segment_type = hfp_reader.GetSegmentType();

      horizontal_path_segment.m_path_course = Units::RadiansAngle(hfp_reader.GetCourse()).value();

      horizontal_path_segment.m_turn_info.x_position_meters = Units::MetersLength(hfp_reader.GetTurnCenterX()).value();
      horizontal_path_segment.m_turn_info.y_position_meters = Units::MetersLength(hfp_reader.GetTurnCenterY()).value();

      horizontal_path_segment.m_turn_info.q_start = hfp_reader.GetAngleStartOfTurn();
      horizontal_path_segment.m_turn_info.q_end = hfp_reader.GetAngleEndOfTurn();

      horizontal_path_segment.m_turn_info.radius = hfp_reader.GetTurnRadius();
      horizontal_path_segment.m_turn_info.groundspeed = hfp_reader.GetGroundSpeed();
      horizontal_path_segment.m_turn_info.bankAngle = hfp_reader.GetBankAngle();

      m_horizontal_trajectory.push_back(horizontal_path_segment);
   }

   m_decrementing_distance_calculator =
         AlongPathDistanceCalculator(m_horizontal_trajectory, TrajectoryIndexProgressionDirection::DECREMENTING);
   m_decrementing_position_calculator =
         PositionCalculator(m_horizontal_trajectory, TrajectoryIndexProgressionDirection::DECREMENTING);
}
