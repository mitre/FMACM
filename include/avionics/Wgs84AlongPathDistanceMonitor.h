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

#include "avionics/Wgs84HorizontalPathMonitor.h"

#include "public/AircraftState.h"
#include "public/WaypointPassingMonitor.h"

namespace aaesim::avionics {
class Wgs84AlongPathDistanceMonitor final : public Wgs84HorizontalPathMonitor {

  public:
   static std::shared_ptr<Wgs84AlongPathDistanceMonitor> FromHorizontalPath(
         const std::vector<Wgs84HorizontalPathSegment> &path);
   static std::shared_ptr<Wgs84AlongPathDistanceMonitor> FromReversedHorizontalPath(
         const std::vector<Wgs84HorizontalPathSegment> &path);
   struct ResultData {
      Units::Length abs_dist_to_segment{Units::negInfinity()};
      Units::Length distance_from_path_start{Units::negInfinity()};
      Units::Length distance_to_path_end{Units::negInfinity()};
      Units::Length distance_to_segment_end{Units::negInfinity()};
      Units::Length distance_to_segment_start{Units::negInfinity()};
      Units::SignedAngle enu_course_along_path{Units::zero()};
      LatitudeLongitudePoint point_on_segment{};
      ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE side_of_segment{
            ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE::UNSET};
      bool solution_is_valid{false};
   };
   Wgs84AlongPathDistanceMonitor() = default;
   Wgs84AlongPathDistanceMonitor(const std::vector<Wgs84HorizontalPathSegment> &path_direction_of_flight);
   ~Wgs84AlongPathDistanceMonitor() = default;
   bool IsPassedEndOfRoute() const override;
   bool IsBeforeStartOfRoute() const override;
   const std::vector<Wgs84HorizontalPathSegment> &GetHorizontalPath() const override;
   const Wgs84HorizontalPathSegment &GetCurrentSegment() const override;
   const Wgs84HorizontalPathSegment &GetNextSegment() const override;
   ResultData ComputeAlongPathDistance(const aaesim::open_source::AircraftState &state);

  private:
   inline static const Units::MetersLength GAP_NODE_TOLERANCE{300};
   void BuildHorizontalPointMonitors(const std::vector<Wgs84HorizontalPathSegment> &extended_path);
   const Wgs84HorizontalPathSegment &FindCurrentSegment(const aaesim::open_source::AircraftState &state);
   void AdvanceToNextShape();
   void RevertToPreviousShape();
   void CheckBooleans();
   ResultData ComputeForSegment(const Wgs84HorizontalPathSegment &segment,
                                const aaesim::LatitudeLongitudePoint &point_near_path,
                                const aaesim::LatitudeLongitudePoint &point_on_segment);

   std::vector<
         std::pair<std::shared_ptr<aaesim::open_source::WaypointPassingMonitor>, const Wgs84HorizontalPathSegment>>
         m_ordered_hpoint_monitors{};
   std::vector<
         std::pair<std::shared_ptr<aaesim::open_source::WaypointPassingMonitor>, const Wgs84HorizontalPathSegment>>
         m_sequenced_hpoint_monitors{};
   std::vector<Wgs84HorizontalPathSegment> m_original_path{};
   std::vector<Wgs84HorizontalPathSegment> m_extended_path{};
   bool m_before_start_of_original_route{false};
   bool m_after_end_of_original_route{false};
   Units::Length m_distance_to_go_from_current_segment_start{Units::infinity()};
   Units::Length m_distance_from_path_start{Units::zero()};
};

inline bool Wgs84AlongPathDistanceMonitor::IsPassedEndOfRoute() const { return m_after_end_of_original_route; }

inline bool Wgs84AlongPathDistanceMonitor::IsBeforeStartOfRoute() const { return m_before_start_of_original_route; }

inline const std::vector<Wgs84HorizontalPathSegment> &Wgs84AlongPathDistanceMonitor::GetHorizontalPath() const {
   return m_original_path;
}

inline const Wgs84HorizontalPathSegment &Wgs84AlongPathDistanceMonitor::GetCurrentSegment() const {
   return m_ordered_hpoint_monitors.back().second;
}

inline const Wgs84HorizontalPathSegment &Wgs84AlongPathDistanceMonitor::GetNextSegment() const {
   if (not m_after_end_of_original_route) return m_ordered_hpoint_monitors[m_ordered_hpoint_monitors.size() - 2].second;
   throw std::runtime_error("Invalid Situation: client code is calling for next leg when there is no next leg");
}

}  // namespace aaesim::avionics