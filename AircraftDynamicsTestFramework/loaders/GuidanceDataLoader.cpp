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

#include "framework/GuidanceDataLoader.h"

#include "utility/CsvParser.h"
#include "framework/HfpReader2020.h"
#include "public/CoreUtils.h"
#include "framework/WaypointSequenceReader.h"
#include "public/GeolibUtils.h"
#include "public/SingleTangentPlaneSequence.h"
#include "utility/BoundedValue.h"

using namespace fmacm;
using namespace aaesim::open_source;

log4cplus::Logger GuidanceDataLoader::m_logger = log4cplus::Logger::getInstance("GuidanceDataLoader");

bool GuidanceDataLoader::load(DecodedStream *input) {
   double tmp_mach(0), tmp_ias(INT16_MIN), tmp_altitude(INT32_MAX);
   set_stream(input);
   register_var("vfp_csv_file", &m_vfp_filename, true);
   register_var("planned_cruise_mach", &tmp_mach, false);
   register_var("planned_transition_ias", &tmp_ias, false);
   register_var("planned_transition_altitude", &tmp_altitude, false);
   register_var("hfp_csv_file", &m_hfp_filename, false);
   register_var("hfp_compute_xy", &m_compute_xy, false);
   register_var("waypoint_sequence", &m_waypoint_sequence_file, false);
   m_loaded = complete();

   if (tmp_altitude < INT32_MAX)
      m_planned_descent_parameters.planned_transition_altitude = Units::FeetLength(tmp_altitude);
   if (tmp_ias > 0) m_planned_descent_parameters.planned_transition_ias = Units::KnotsSpeed(tmp_ias);
   if (tmp_mach > 0) m_planned_descent_parameters.planned_cruise_mach = tmp_mach;

   return m_loaded;
}

std::shared_ptr<GuidanceFromStaticData> GuidanceDataLoader::BuildGuidanceCalculator() {
   std::pair<std::shared_ptr<TangentPlaneSequence>, std::vector<HorizontalPath>> horizontal_path_data;
   if (!m_hfp_filename.empty()) {
      horizontal_path_data = ProcessHfpData();
   } else if (!m_waypoint_sequence_file.empty()) {
      horizontal_path_data = ProcessWaypointSequenceData();
   } else {
      throw std::runtime_error("unable to load a file that describes the horizontal path");
   }
   m_tangent_plane = horizontal_path_data.first;

   DoDebugLogging(horizontal_path_data.second);

   return std::make_shared<GuidanceFromStaticData>(horizontal_path_data.second, BuildVerticalGuidanceData(),
                                                   m_planned_descent_parameters);
}

std::pair<std::shared_ptr<TangentPlaneSequence>, std::vector<HorizontalPath>> GuidanceDataLoader::ProcessHfpData()
      const {
   auto tangent_plane_sequence = BuildTangentPlaneFromFileData();
   testvector::HfpReader2020 hfp_reader(m_hfp_filename, 1);
   std::vector<HorizontalPath> hpath;
   if (m_compute_xy)
      hpath = BuildHorizontalPathComputeEuclideanComponents(hfp_reader, tangent_plane_sequence);
   else
      hpath = BuildHorizontalPathUsingAllColumns(hfp_reader);
   return std::make_pair(tangent_plane_sequence, hpath);
}

std::pair<std::shared_ptr<TangentPlaneSequence>, std::vector<HorizontalPath>>
      GuidanceDataLoader::ProcessWaypointSequenceData() const {
   WaypointSequenceReader reader(m_waypoint_sequence_file);
   auto data = reader.ReadFile();
   std::list<Waypoint> ordered_waypoints;
   auto convert_to_waypoints = [&ordered_waypoints](const WaypointSequenceReader::WaypointSequenceRow &data_row) {
      std::string name = "WP" + std::to_string(Units::DegreesAngle(data_row.latitude).value()) + "/" +
                         std::to_string(Units::DegreesAngle(data_row.longitude).value());
      Waypoint waypoint(name, data_row.latitude, data_row.longitude);
      ordered_waypoints.push_back(waypoint);
   };
   std::for_each(data.cbegin(), data.cend(), convert_to_waypoints);
   auto tangent_planes = BuildTangentPlane(ordered_waypoints);

   struct ZippedData {
      aaesim::LineOnEllipsoid line;
      aaesim::LatitudeLongitudePoint next_point;
      WaypointSequenceReader::WaypointSequenceRow waypoint_data;
   };
   std::vector<ZippedData> all_lines;
   std::reverse(data.begin(), data.end());
   for (auto loop = 1; loop < data.size(); ++loop) {
      aaesim::LatitudeLongitudePoint start_point(data[loop - 1].latitude, data[loop - 1].longitude);
      aaesim::LatitudeLongitudePoint end_point(data[loop].latitude, data[loop].longitude);
      aaesim::LineOnEllipsoid line = aaesim::LineOnEllipsoid::CreateFromPoints(start_point, end_point);
      ZippedData dz;
      dz.line = line;
      dz.waypoint_data = data[loop];
      if (loop < data.size() - 1) {
         aaesim::LatitudeLongitudePoint next_point(data[loop + 1].latitude, data[loop + 1].longitude);
         dz.next_point = next_point;
      }
      all_lines.push_back(dz);
   }

   std::vector<HorizontalPath> horizontal_path_sequence;
   auto hpath_creator = [&horizontal_path_sequence, tangent_planes](const ZippedData &line_datarow) {
      const auto line = line_datarow.line;
      const auto terminating_waypoint_info = line_datarow.waypoint_data;

      if (horizontal_path_sequence.empty()) {
         HorizontalPath first_segment;
         EarthModel::GeodeticPosition lat_lon_position;
         lat_lon_position.latitude = line.GetStartPoint().GetLatitude();
         lat_lon_position.longitude = line.GetStartPoint().GetLongitude();
         EarthModel::LocalPositionEnu xy_position;
         tangent_planes->ConvertGeodeticToLocal(lat_lon_position, xy_position);
         first_segment.SetXYPositionMeters(Units::MetersLength(xy_position.x).value(),
                                           Units::MetersLength(xy_position.y).value());

         first_segment.m_segment_type = HorizontalPath::STRAIGHT;
         horizontal_path_sequence.push_back(first_segment);
      }

      if (Units::abs(terminating_waypoint_info.turn_radius) < Units::MetersLength(1)) {
         HorizontalPath horizontal_path_segment;
         EarthModel::GeodeticPosition lat_lon_position;
         lat_lon_position.latitude = terminating_waypoint_info.latitude;
         lat_lon_position.longitude = terminating_waypoint_info.longitude;
         EarthModel::LocalPositionEnu xy_position;
         tangent_planes->ConvertGeodeticToLocal(lat_lon_position, xy_position);
         horizontal_path_segment.SetXYPositionMeters(Units::MetersLength(xy_position.x).value(),
                                                     Units::MetersLength(xy_position.y).value());
         horizontal_path_segment.m_segment_type = HorizontalPath::STRAIGHT;
         horizontal_path_sequence.push_back(horizontal_path_segment);
      } else {
         HorizontalPath horizontal_path_segment_turn_point;
         horizontal_path_segment_turn_point.m_segment_type = HorizontalPath::TURN;
         const auto next_terminating_point = line_datarow.next_point;
         aaesim::LineOnEllipsoid inbound_line = line;
         aaesim::LineOnEllipsoid outbound_line =
               aaesim::LineOnEllipsoid::CreateFromPoints(line.GetEndPoint(), next_terminating_point);
         const auto turn_arc = aaesim::GeolibUtils::CreateArcTangentToTwoLines(inbound_line, outbound_line,
                                                                               terminating_waypoint_info.turn_radius);
         EarthModel::GeodeticPosition lat_lon_position;
         lat_lon_position.latitude = turn_arc.second.GetStartPoint().GetLatitude();
         lat_lon_position.longitude = turn_arc.second.GetStartPoint().GetLongitude();
         EarthModel::LocalPositionEnu xy_position;
         tangent_planes->ConvertGeodeticToLocal(lat_lon_position, xy_position);
         horizontal_path_segment_turn_point.SetXYPositionMeters(Units::MetersLength(xy_position.x).value(),
                                                                Units::MetersLength(xy_position.y).value());

         lat_lon_position.latitude = turn_arc.second.GetCenterPoint().GetLatitude();
         lat_lon_position.longitude = turn_arc.second.GetCenterPoint().GetLongitude();
         tangent_planes->ConvertGeodeticToLocal(lat_lon_position, xy_position);
         horizontal_path_segment_turn_point.m_turn_info.x_position_meters = Units::MetersLength(xy_position.x).value();
         horizontal_path_segment_turn_point.m_turn_info.y_position_meters = Units::MetersLength(xy_position.y).value();
         horizontal_path_segment_turn_point.m_turn_info.radius = terminating_waypoint_info.turn_radius;
         horizontal_path_segment_turn_point.m_turn_info.groundspeed = Units::ZERO_SPEED;
         Units::Angle loaded_bank_value = terminating_waypoint_info.bank_angle;
         if (loaded_bank_value == Units::zero()) loaded_bank_value = Units::DegreesAngle(15);
         horizontal_path_segment_turn_point.m_turn_info.bankAngle = loaded_bank_value;
         horizontal_path_segment_turn_point.m_turn_info.turn_type = HorizontalTurnPath::TURN_TYPE::PERFORMANCE;
         horizontal_path_sequence.push_back(horizontal_path_segment_turn_point);

         HorizontalPath horizontal_path_segment_after_turn;
         lat_lon_position.latitude = turn_arc.second.GetEndPoint().GetLatitude();
         lat_lon_position.longitude = turn_arc.second.GetEndPoint().GetLongitude();
         tangent_planes->ConvertGeodeticToLocal(lat_lon_position, xy_position);
         horizontal_path_segment_after_turn.SetXYPositionMeters(Units::MetersLength(xy_position.x).value(),
                                                                Units::MetersLength(xy_position.y).value());
         horizontal_path_segment_after_turn.m_segment_type = HorizontalPath::STRAIGHT;
         horizontal_path_sequence.push_back(horizontal_path_segment_after_turn);
      }
   };
   std::for_each(all_lines.cbegin(), all_lines.cend(), hpath_creator);

   ComputeCourseColumnsInPlace(horizontal_path_sequence);

   return std::make_pair(tangent_planes, horizontal_path_sequence);
}

std::shared_ptr<TangentPlaneSequence> GuidanceDataLoader::BuildTangentPlaneFromFileData() const {
   testvector::HfpReader2020 hfp_reader(m_hfp_filename, 1);
   std::list<Waypoint> ordered_lat_long_points;
   while (hfp_reader.Advance()) {
      Waypoint wp(hfp_reader.GetString(HorizontalFields::IX), hfp_reader.GetLatitude(), hfp_reader.GetLongitude());
      ordered_lat_long_points.push_back(wp);
   }
   std::reverse(ordered_lat_long_points.begin(), ordered_lat_long_points.end());
   return BuildTangentPlane(ordered_lat_long_points);
}

std::shared_ptr<TangentPlaneSequence> GuidanceDataLoader::BuildTangentPlane(
      const std::list<Waypoint> &ordered_waypoints) const {
   auto shortened_legs = CoreUtils::ShortenLongLegs(ordered_waypoints);
   return std::make_shared<SingleTangentPlaneSequence>(shortened_legs);
}

GuidanceFromStaticData::VerticalData GuidanceDataLoader::BuildVerticalGuidanceData() const {
   std::ifstream file(m_vfp_filename.c_str());
   if (!file.is_open()) {
      std::string msg = "Vertical trajectory file " + m_vfp_filename + " not found";
      throw std::runtime_error(msg);
   }

   GuidanceFromStaticData::VerticalData vertical_data;
   int numhdrs = 0;
   for (CsvParser::CsvIterator csviter(file); csviter != CsvParser::CsvIterator(); ++csviter) {
      if (numhdrs < 1) {
         numhdrs++;
         continue;
      }

      if ((*csviter).Size() != NUM_VERTICAL_TRAJ_FIELDS) {
         std::string msg = "Invalid number of fields found in " + m_vfp_filename + "vertical trajectory file";
         throw std::runtime_error(msg);
      }

      for (int vfield = TIME_TO_GO_SEC; vfield != NUM_VERTICAL_TRAJ_FIELDS; vfield++) {
         std::string fieldstr = (*csviter)[vfield];
         double val = atof(fieldstr.c_str());
         switch (static_cast<VerticalFields>(vfield)) {
            case TIME_TO_GO_SEC:
               vertical_data.m_time_to_go_sec.push_back(val);
               break;

            case DISTANCE_TO_GO_VERT_M:
               vertical_data.m_distance_to_go_meters.push_back(val);
               break;

            case ALTITUDE_M:
               vertical_data.m_altitude_meters.push_back(val);
               break;

            case IAS_MPS:
               vertical_data.m_ias_mps.push_back(val);
               break;

            case DOT_ALTITUDE_MPS:
               vertical_data.m_vertical_speed_mps.push_back(val);
               break;

            case GS_MPS:
               vertical_data.m_ground_speed_mps.push_back(val);
            default:
               break;
         }
      }
   }

   file.close();

   return vertical_data;
}

std::vector<HorizontalPath> GuidanceDataLoader::BuildHorizontalPathUsingAllColumns(
      testvector::HfpReader2020 &hfp_reader) const {
   std::vector<HorizontalPath> horizontal_path_sequence;

   while (hfp_reader.Advance()) {
      HorizontalPath horizontal_path_segment;

      horizontal_path_segment.SetXYPositionMeters(Units::MetersLength(hfp_reader.GetX()).value(),
                                                  Units::MetersLength(hfp_reader.GetY()).value());
      horizontal_path_segment.m_path_length_cumulative_meters = Units::MetersLength(hfp_reader.GetDTG()).value();
      horizontal_path_segment.m_segment_type = hfp_reader.GetSegmentType();
      horizontal_path_segment.m_path_course = Units::RadiansAngle(hfp_reader.GetCourse()).value();

      if (horizontal_path_segment.m_segment_type == HorizontalPath::SegmentType::TURN) {
         horizontal_path_segment.m_turn_info.x_position_meters =
               Units::MetersLength(hfp_reader.GetTurnCenterX()).value();
         horizontal_path_segment.m_turn_info.y_position_meters =
               Units::MetersLength(hfp_reader.GetTurnCenterY()).value();
         horizontal_path_segment.m_turn_info.q_start = hfp_reader.GetAngleStartOfTurn();
         horizontal_path_segment.m_turn_info.q_end = hfp_reader.GetAngleEndOfTurn();
         horizontal_path_segment.m_turn_info.radius = hfp_reader.GetTurnRadius();
         horizontal_path_segment.m_turn_info.groundspeed = hfp_reader.GetGroundSpeed();
         Units::Angle loaded_bank_value = hfp_reader.GetBankAngle();
         if (loaded_bank_value == Units::zero()) loaded_bank_value = Units::DegreesAngle(15);
         horizontal_path_segment.m_turn_info.bankAngle = loaded_bank_value;
         horizontal_path_segment.m_turn_info.turn_type = HorizontalTurnPath::TURN_TYPE::PERFORMANCE;
      }

      horizontal_path_sequence.push_back(horizontal_path_segment);
   }
   return horizontal_path_sequence;
}

std::vector<HorizontalPath> GuidanceDataLoader::BuildHorizontalPathComputeEuclideanComponents(
      testvector::HfpReader2020 &hfp_reader, std::shared_ptr<TangentPlaneSequence> &tangent_plane_sequence) const {
   std::vector<HorizontalPath> horizontal_path_sequence;
   while (hfp_reader.Advance()) {
      HorizontalPath horizontal_path_segment;

      EarthModel::GeodeticPosition lat_lon_position;
      lat_lon_position.latitude = hfp_reader.GetLatitude();
      lat_lon_position.longitude = hfp_reader.GetLongitude();
      EarthModel::LocalPositionEnu xy_position;
      tangent_plane_sequence->ConvertGeodeticToLocal(lat_lon_position, xy_position);
      horizontal_path_segment.SetXYPositionMeters(Units::MetersLength(xy_position.x).value(),
                                                  Units::MetersLength(xy_position.y).value());
      horizontal_path_segment.m_segment_type = hfp_reader.GetSegmentType();

      if (horizontal_path_segment.m_segment_type == HorizontalPath::SegmentType::TURN) {
         lat_lon_position.latitude = hfp_reader.GetTurnCenterLatitude();
         lat_lon_position.longitude = hfp_reader.GetTurnCenterLongitude();
         tangent_plane_sequence->ConvertGeodeticToLocal(lat_lon_position, xy_position);

         horizontal_path_segment.m_turn_info.x_position_meters = Units::MetersLength(xy_position.x).value();
         horizontal_path_segment.m_turn_info.y_position_meters = Units::MetersLength(xy_position.y).value();
         horizontal_path_segment.m_turn_info.radius = hfp_reader.GetTurnRadius();
         horizontal_path_segment.m_turn_info.groundspeed = hfp_reader.GetGroundSpeed();
         Units::Angle loaded_bank_value = hfp_reader.GetBankAngle();
         if (loaded_bank_value == Units::zero()) loaded_bank_value = Units::DegreesAngle(15);
         horizontal_path_segment.m_turn_info.bankAngle = loaded_bank_value;
         horizontal_path_segment.m_turn_info.turn_type = HorizontalTurnPath::TURN_TYPE::PERFORMANCE;
      }

      horizontal_path_sequence.push_back(horizontal_path_segment);
   }

   ComputeCourseColumnsInPlace(horizontal_path_sequence);
   return horizontal_path_sequence;
}

void GuidanceDataLoader::ComputeCourseColumnsInPlace(std::vector<HorizontalPath> &horizontal_path_sequence) const {

   const auto start_at_turn = horizontal_path_sequence.front().m_segment_type == HorizontalPath::SegmentType::TURN;
   const auto end_at_turn = horizontal_path_sequence.back().m_segment_type == HorizontalPath::SegmentType::TURN;
   if (start_at_turn || end_at_turn) {
      std::string msg = "Invalid HFP data. Cannot start or end the path with a TURN segment";
      throw std::runtime_error(msg);
   }

   for (auto idx = 1; idx < horizontal_path_sequence.size(); ++idx) {
      Units::MetersLength dx = Units::MetersLength(horizontal_path_sequence[idx].GetXPositionMeters()) -
                               Units::MetersLength(horizontal_path_sequence[idx - 1].GetXPositionMeters());
      Units::MetersLength dy = Units::MetersLength(horizontal_path_sequence[idx].GetYPositionMeters()) -
                               Units::MetersLength(horizontal_path_sequence[idx - 1].GetYPositionMeters());
      Units::MetersLength distance_between = Units::sqrt(Units::sqr(dx) + Units::sqr(dy));
      horizontal_path_sequence[idx].m_path_length_cumulative_meters =
            horizontal_path_sequence[idx - 1].m_path_length_cumulative_meters + distance_between.value();

      auto path_course = Units::UnsignedRadiansAngle(Units::arctan2(dy.value(), dx.value())).value();
      horizontal_path_sequence[idx - 1].m_path_course = path_course;

      if (horizontal_path_sequence[idx].m_segment_type == HorizontalPath::SegmentType::TURN &&
          idx < horizontal_path_sequence.size() - 1) {
         Units::MetersLength dx_start =
               Units::MetersLength(horizontal_path_sequence[idx].GetXPositionMeters()) -
               Units::MetersLength(horizontal_path_sequence[idx].m_turn_info.x_position_meters);
         Units::MetersLength dy_start =
               Units::MetersLength(horizontal_path_sequence[idx].GetYPositionMeters()) -
               Units::MetersLength(horizontal_path_sequence[idx].m_turn_info.y_position_meters);
         horizontal_path_sequence[idx].m_turn_info.q_start = Units::arctan2(dy_start.value(), dx_start.value());

         Units::MetersLength dx_end = Units::MetersLength(horizontal_path_sequence[idx + 1].GetXPositionMeters()) -
                                      Units::MetersLength(horizontal_path_sequence[idx].m_turn_info.x_position_meters);
         Units::MetersLength dy_end = Units::MetersLength(horizontal_path_sequence[idx + 1].GetYPositionMeters()) -
                                      Units::MetersLength(horizontal_path_sequence[idx].m_turn_info.y_position_meters);
         horizontal_path_sequence[idx].m_turn_info.q_end = Units::arctan2(dy_end.value(), dx_end.value());
      }
   }

   for (auto idx = 0; idx < horizontal_path_sequence.size(); ++idx) {
      if (horizontal_path_sequence[idx].m_segment_type == HorizontalPath::SegmentType::TURN)
         horizontal_path_sequence[idx].m_path_course = horizontal_path_sequence[idx - 1].m_path_course;
   }
   horizontal_path_sequence.back().m_path_course = (--(--horizontal_path_sequence.cend()))->m_path_course;
}