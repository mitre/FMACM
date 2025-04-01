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

#include <string>
#include "avionics/Wgs84HorizontalTurnPath.h"

namespace aaesim {

class Wgs84HorizontalPathSegment {

  public:
   enum SegmentType { STRAIGHT, TURN, UNSET };

   enum TurnDirection { LEFT = -1, NO_TURN = 0, RIGHT = 1 };

   Wgs84HorizontalPathSegment(LineOnEllipsoid line, Units::Length cumulative_distance);

   Wgs84HorizontalPathSegment(ArcOnEllipsoid arc, Units::Length cumulative_distance);

   Wgs84HorizontalPathSegment(const ArcOnEllipsoid arc, const Units::Length cumulative_distance,
                              Units::UnsignedAngle bankangle, Units::Speed gs,
                              aaesim::open_source::HorizontalTurnPath::TURN_TYPE turn_type);

   /**
    * This is provided as a kind of copy constructor. Use with this object's getters to create a separate memory copy.
    * @param shape
    * @param segment_type
    * @param cumulative_distance
    * @param bankangle
    * @param gs
    * @param turn_type
    */
   Wgs84HorizontalPathSegment(std::shared_ptr<const ShapeOnEllipsoid> shape, SegmentType segment_type,
                              const Units::Length cumulative_distance, Units::UnsignedAngle bankangle, Units::Speed gs,
                              aaesim::open_source::HorizontalTurnPath::TURN_TYPE turn_type);

   virtual ~Wgs84HorizontalPathSegment() = default;
   bool operator==(const Wgs84HorizontalPathSegment &rhs) const;
   bool operator!=(const Wgs84HorizontalPathSegment &rhs) const;
   SegmentType GetSegmentType() const;
   std::string GetSegmentTypeAsString() const;
   TurnDirection GetTurnDirection() const;
   Units::SignedRadiansAngle GetTurnAngularExtent() const;
   LatitudeLongitudePoint GetTurnCenter() const;
   std::shared_ptr<const ShapeOnEllipsoid> GetShapeOnEllipsoid() const;
   const Wgs84HorizontalTurnPath &GetTurnInfo() const;
   const Units::Length &GetCumulativePathLength() const;
   const Units::Length &GetCumulativePathLengthIncludingThisShape() const;
   const Units::SignedAngle GetCourseEnuAtStartPoint() const;
   Units::Length GetRadius() const;

  private:
   SegmentType m_segment_type;
   std::shared_ptr<const ShapeOnEllipsoid> m_shape_on_ellipsoid;
   Units::MetersLength m_cumulative_path_length_including_this_shape;
   Units::MetersLength m_cumulative_path_length_not_including_this_shape;
   Wgs84HorizontalTurnPath m_turn_info;

   // Keep the empty constructor private
   Wgs84HorizontalPathSegment() = default;
};

inline Wgs84HorizontalPathSegment::SegmentType Wgs84HorizontalPathSegment::GetSegmentType() const {
   return m_segment_type;
}

inline std::shared_ptr<const ShapeOnEllipsoid> Wgs84HorizontalPathSegment::GetShapeOnEllipsoid() const {
   return m_shape_on_ellipsoid;
}

inline const Wgs84HorizontalTurnPath &Wgs84HorizontalPathSegment::GetTurnInfo() const { return m_turn_info; }

inline const Units::Length &Wgs84HorizontalPathSegment::GetCumulativePathLength() const {
   return m_cumulative_path_length_not_including_this_shape;
}

inline const Units::Length &Wgs84HorizontalPathSegment::GetCumulativePathLengthIncludingThisShape() const {
   return m_cumulative_path_length_including_this_shape;
}

inline std::string Wgs84HorizontalPathSegment::GetSegmentTypeAsString() const {
   switch (GetSegmentType()) {
      case UNSET:
         return "UNSET";
      case STRAIGHT:
         return "STRAIGHT";
      case TURN:
         return "TURN";
      default:
         throw std::logic_error("Found invalid segment type: " + GetSegmentType());
   }
}
}  // namespace aaesim