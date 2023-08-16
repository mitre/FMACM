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

#include "loader/Loadable.h"

#include <list>

#include "framework/GuidanceFromStaticData.h"
#include "public/TangentPlaneSequence.h"
#include "HfpReader2020.h"
#include "public/Waypoint.h"
#include "MiniCSV/minicsv.h"

namespace fmacm {
class GuidanceDataLoader final : public Loadable {
  public:
   GuidanceDataLoader()
      : m_loaded(false),
        m_hfp_filename(),
        m_vfp_filename(),
        m_tangent_plane(),
        m_compute_xy(true),
        m_planned_descent_parameters() {}
   bool load(DecodedStream *input) override;
   std::shared_ptr<GuidanceFromStaticData> BuildGuidanceCalculator();
   std::shared_ptr<TangentPlaneSequence> GetTangentPlaneSequence() const;
   const bool IsLoaded() const;
   GuidanceFromStaticData::PlannedDescentParameters GetPlannedDescentParameters() const;

  private:
   static log4cplus::Logger m_logger;
   enum VerticalFields {
      TIME_TO_GO_SEC = 0,
      DISTANCE_TO_GO_VERT_M,
      ALTITUDE_M,
      IAS_MPS,
      DOT_ALTITUDE_MPS,
      GS_MPS,
      NUM_VERTICAL_TRAJ_FIELDS
   };
   enum HorizontalFields {
      IX = 0,
      X_M,
      Y_M,
      DISTANCE_TO_GO_HORZ_M,
      SEGMENT_TYPE,
      COURSE_R,
      TURN_CENTER_X_M,
      TURN_CENTER_Y_M,
      ANGLE_AT_TURN_START_R,
      ANGLE_AT_TURN_END_R,
      TURN_RADIUS_M,
      GROUND_SPEED_MPS,
      BANK_ANGLE_DEG,
      LAT_D,
      LON_D,
      TURN_CENTER_LAT_D,
      TURN_CENTER_LON_D,
      NUM_HORIZONTAL_TRAJ_FIELDS
   };

   static void DoDebugLogging(const std::vector<HorizontalPath> &horizontal_path);
   GuidanceFromStaticData::VerticalData BuildVerticalGuidanceData() const;
   std::pair<std::shared_ptr<TangentPlaneSequence>, std::vector<HorizontalPath>> ProcessHfpData() const;
   std::pair<std::shared_ptr<TangentPlaneSequence>, std::vector<HorizontalPath>> ProcessWaypointSequenceData() const;
   std::vector<HorizontalPath> BuildHorizontalPathUsingAllColumns(testvector::HfpReader2020 &hfp_reader) const;
   std::vector<HorizontalPath> BuildHorizontalPathComputeEuclideanComponents(
         testvector::HfpReader2020 &hfp_reader, std::shared_ptr<TangentPlaneSequence> &tangent_plane_sequence) const;
   std::shared_ptr<TangentPlaneSequence> BuildTangentPlaneFromFileData() const;
   std::shared_ptr<TangentPlaneSequence> BuildTangentPlane(const std::list<Waypoint> &ordered_waypoints) const;
   void ComputeCourseColumnsInPlace(std::vector<HorizontalPath> &horizontal_path) const;

   bool m_loaded;
   std::string m_hfp_filename, m_vfp_filename, m_waypoint_sequence_file;
   std::shared_ptr<TangentPlaneSequence> m_tangent_plane;
   bool m_compute_xy;
   GuidanceFromStaticData::PlannedDescentParameters m_planned_descent_parameters;
};

inline const bool GuidanceDataLoader::IsLoaded() const { return m_loaded; }

inline std::shared_ptr<TangentPlaneSequence> GuidanceDataLoader::GetTangentPlaneSequence() const {
   return m_tangent_plane;
}

inline GuidanceFromStaticData::PlannedDescentParameters GuidanceDataLoader::GetPlannedDescentParameters() const {
   return m_planned_descent_parameters;
}

inline void GuidanceDataLoader::DoDebugLogging(const std::vector<HorizontalPath> &horizontal_path_sequence) {
   if (m_logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
      mini::csv::ofstream os("debug_hfp_data.csv");
      if (!os.is_open()) {
         std::string emsg = "Cannot open debug file";
         throw std::runtime_error(emsg);
      }
      os.set_delimiter(',', ",");
      auto column_inserter = [&os](std::string &column_name) { os << column_name; };
      std::vector<std::string> COLUMN_NAMES = {"HPT_j",
                                               "x[m]",
                                               "y[m]",
                                               "DTG[m]",
                                               "Segment_Type",
                                               "Course[rad]",
                                               "Turn_Center_x[m]",
                                               "Turn_Center_y[m]",
                                               "Angle_Start_of_Turn[rad]",
                                               "Angle_End_of_Turn[rad]",
                                               "R[m]",
                                               "GS[m/s]",
                                               "Bank_Angle[deg]",
                                               "Lat[deg]",
                                               "Lon[deg]",
                                               "Turn_Center_Lat[deg]",
                                               "Turn_Center_Lon[deg]"};
      std::for_each(COLUMN_NAMES.begin(), COLUMN_NAMES.end(), column_inserter);
      os << NEWLINE;
      os.flush();
      int index = 0;
      auto hfp_writer = [&index, &os](const HorizontalPath &path_segment) {
         os << index;
         os << path_segment.GetXPositionMeters();
         os << path_segment.GetYPositionMeters();
         os << path_segment.m_path_length_cumulative_meters;
         os << path_segment.GetSegmentTypeAsString();
         os << path_segment.m_path_course;
         os << path_segment.m_turn_info.x_position_meters;
         os << path_segment.m_turn_info.y_position_meters;
         os << path_segment.m_turn_info.q_start.value();
         os << path_segment.m_turn_info.q_end.value();
         os << path_segment.m_turn_info.radius.value();
         os << path_segment.m_turn_info.groundspeed.value();
         os << Units::DegreesAngle(path_segment.m_turn_info.bankAngle).value();
         os << "0";
         os << "0";
         os << "0";
         os << "0";
         os << NEWLINE;
         os.flush();
         ++index;
      };
      std::for_each(horizontal_path_sequence.cbegin(), horizontal_path_sequence.cend(), hfp_writer);
      os.close();
   }
}

}  // namespace fmacm
