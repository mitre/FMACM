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

#include "public/ArcOnEllipsoid.h"
#include "utility/CustomUnits.h"
#include "public/GeolibUtils.h"

using namespace geolib_idealab;
using namespace aaesim;

log4cplus::Logger ArcOnEllipsoid::m_logger = log4cplus::Logger::getInstance("ArcOnEllipsoid");

ArcOnEllipsoid::ArcOnEllipsoid(const geolib_idealab::Arc &arc) {
   m_arc_primitive = arc;
}

Units::Length aaesim::ArcOnEllipsoid::GetShapeLength() const {
   /*
    * Note: this implementation could call CalculateDistanceFromPointOnShapeToEnd and get the same result,
    * but that would also carry multiple extra iterative calls to geolib_idealab methods unnecessarily. In order to
    * create a more computationally efficient computation of length, this calls directly into geolib_idealab here
    * with all the known arc information.
    */
   ErrorSet error_set = ErrorCodes::SUCCESS;
   int steps = INT32_MIN;
   double length = arcLength(m_arc_primitive.centerPoint,
                       m_arc_primitive.radius,
                       m_arc_primitive.startAz,
                       m_arc_primitive.endAz,
                       m_arc_primitive.dir,
                       &steps,
                       &error_set,
                       GEOLIB_TOLERANCE,
                       GEOLIB_EPSILON);
   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, GeolibUtils::m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(GeolibUtils::m_basic_error_message);
   }

   return Units::NauticalMilesLength(length);
}

Units::SignedAngle ArcOnEllipsoid::GetArcAngularExtent() const {
   return Units::SignedRadiansAngle(m_arc_primitive.subtendedAngle);
}
LatitudeLongitudePoint ArcOnEllipsoid::GetCenterPoint() const {
   return LatitudeLongitudePoint::CreateFromGeolibPrimitive(m_arc_primitive.centerPoint);
}
Units::Length ArcOnEllipsoid::GetRadius() const {
   return Units::NauticalMilesLength(m_arc_primitive.radius);
}
Units::SignedAngle ArcOnEllipsoid::GetStartAzimuthEnu() const {
   const Units::UnsignedAngle start_azimuth_ned = Units::UnsignedRadiansAngle(m_arc_primitive.startAz);
   return GeolibUtils::ConvertCourseFromNedToEnu(start_azimuth_ned);
}
Units::SignedAngle ArcOnEllipsoid::GetEndAzimuthEnu() const {
   const Units::UnsignedAngle end_azimuth_ned = Units::UnsignedRadiansAngle(m_arc_primitive.endAz);
   return GeolibUtils::ConvertCourseFromNedToEnu(end_azimuth_ned);
}
geolib_idealab::ArcDirection ArcOnEllipsoid::GetArcDirection() const {
   return m_arc_primitive.dir;
}

bool ArcOnEllipsoid::IsPointOnShape(const LatitudeLongitudePoint &test_point) const {
   return GeolibUtils::IsPointOnArc(*this, test_point);;
}

bool ArcOnEllipsoid::IsPointInsideArc(const LatitudeLongitudePoint &test_point) const {
   return GeolibUtils::IsPointInsideArcSegment(*this, test_point);
}

Units::SignedAngle ArcOnEllipsoid::GetCourseEnuTangentToStartPoint() const {
   Units::UnsignedDegreesAngle add_this(90);
   if (m_arc_primitive.dir == ArcDirection::COUNTERCLOCKWISE) {
      // change sign
      add_this *= -1;
   }
   const Units::UnsignedAngle tangent_course_ned = Units::UnsignedRadiansAngle(m_arc_primitive.startAz) + add_this;
   return GeolibUtils::ConvertCourseFromNedToEnu(tangent_course_ned);
}

Units::SignedAngle ArcOnEllipsoid::GetCourseEnuTangentToEndPoint() const {
   Units::UnsignedDegreesAngle add_this(90);
   if (m_arc_primitive.dir == ArcDirection::COUNTERCLOCKWISE) {
      // change sign
      add_this *= -1;
   }
   const Units::UnsignedAngle tangent_course_ned = Units::UnsignedRadiansAngle(m_arc_primitive.endAz) + add_this;
   return GeolibUtils::ConvertCourseFromNedToEnu(tangent_course_ned);
}

LatitudeLongitudePoint ArcOnEllipsoid::GetStartPoint() const {
   return LatitudeLongitudePoint::CreateFromGeolibPrimitive(m_arc_primitive.startPoint);
}

LatitudeLongitudePoint ArcOnEllipsoid::GetEndPoint() const {
   return LatitudeLongitudePoint::CreateFromGeolibPrimitive(m_arc_primitive.endPoint);
}

ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE ArcOnEllipsoid::GetRelativeDirection(const LatitudeLongitudePoint &latitude_longitude_point) const {
   ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE return_this = ShapeOnEllipsoid::UNSET;
   if (IsPointOnShape(latitude_longitude_point)) {
      return_this = ShapeOnEllipsoid::ON_SHAPE;
   } else {
      /*
       * Algorithm:
       * If latitude_longitude_point is inside the arc AND arc direction is CLOCKWISE, return RIGHT_OF_SHAPE
       * If latitude_longitude_point is outside the arc AND arc direction is CLOCKWISE, return LEFT_OF_SHAPE
       * If latitude_longitude_point is inside the arc AND arc direction is COUNTERCLOCKWISE, return LEFT_OF_SHAPE
       * If latitude_longitude_point is outside the arc AND arc direction is COUNTERCLOCKWISE, return RIGHT_OF_SHAPE
       */
      const bool is_inside_arc = IsPointInsideArc(latitude_longitude_point);
      if (is_inside_arc && GetArcDirection()==ArcDirection::CLOCKWISE) {
         return_this = ShapeOnEllipsoid::RIGHT_OF_SHAPE;
      }
      else if (!is_inside_arc && GetArcDirection()==ArcDirection::CLOCKWISE) {
         return_this = ShapeOnEllipsoid::LEFT_OF_SHAPE;
      }
      else if (is_inside_arc && GetArcDirection()==ArcDirection::COUNTERCLOCKWISE) {
         return_this = ShapeOnEllipsoid::LEFT_OF_SHAPE;
      }
      else if (!is_inside_arc && GetArcDirection()==ArcDirection::COUNTERCLOCKWISE) {
         return_this = ShapeOnEllipsoid::RIGHT_OF_SHAPE;
      }
   }
   return return_this;
}

Units::Length ArcOnEllipsoid::GetDistanceToEndPoint(const LatitudeLongitudePoint &latitude_longitude_point) const {
   return ShapeOnEllipsoid::GetDistanceToEndPoint(latitude_longitude_point);
}

LatitudeLongitudePoint ArcOnEllipsoid::GetNearestPointOnShape(const LatitudeLongitudePoint &latitude_longitude_point) const {
   std::pair<bool, LatitudeLongitudePoint> perp_info = GeolibUtils::FindNearestPointOnArcUsingPerpendiculorProjection(*this, latitude_longitude_point);
   return std::get<1>(perp_info);
}

Units::Length ArcOnEllipsoid::CalculateDistanceFromPointOnShapeToEnd(const LatitudeLongitudePoint &point_on_shape) const {
   ErrorSet error_set = ErrorCodes::SUCCESS;
   int steps = INT32_MIN;
   const Units::SignedAngle start_azimuth_enu = GetCenterPoint().CalculateRelationshipBetweenPoints(point_on_shape).second;
   const Units::UnsignedRadiansAngle start_azimuth_ned = GeolibUtils::ConvertCourseFromEnuToNed(start_azimuth_enu);
   double length = arcLength(m_arc_primitive.centerPoint,
                             m_arc_primitive.radius,
                             start_azimuth_ned.value(),
                             m_arc_primitive.endAz,
                             m_arc_primitive.dir,
                             &steps,
                             &error_set,
                             GEOLIB_TOLERANCE,
                             GEOLIB_EPSILON);
   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, GeolibUtils::m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(GeolibUtils::m_basic_error_message);
   }

   return Units::NauticalMilesLength(length);
}

LatitudeLongitudePoint ArcOnEllipsoid::CalculatePointAtDistanceFromStartPoint(const Units::Length &distance_along_shape_from_start_point) const {
   LLPoint calculated_point;
   double calculated_subtended_angle;
   ErrorSet error_set = arcFromLength(m_arc_primitive.centerPoint,
                             m_arc_primitive.startPoint,
                             m_arc_primitive.dir,
                             Units::NauticalMilesLength(distance_along_shape_from_start_point).value(),
                             &calculated_point,
                             &calculated_subtended_angle,
                             GEOLIB_TOLERANCE,
                             GEOLIB_EPSILON);
   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, GeolibUtils::m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(GeolibUtils::m_basic_error_message);
   }

   return LatitudeLongitudePoint::CreateFromGeolibPrimitive(calculated_point);
}

std::pair<Units::SignedAngle,
          LatitudeLongitudePoint> ArcOnEllipsoid::CalculateCourseAtDistanceFromStartPoint(const Units::Length &distance_along_shape_from_start_point) const {
   const LatitudeLongitudePoint point_on_arc_at_distance_from_start_point = CalculatePointAtDistanceFromStartPoint(distance_along_shape_from_start_point);
   const ArcOnEllipsoid subarc = GeolibUtils::CreateArcOnEllipsoid(GetStartPoint(),
                                                                   point_on_arc_at_distance_from_start_point,
                                                                   GetCenterPoint(),
                                                                   GetArcDirection());

   return std::make_pair(subarc.GetCourseEnuTangentToEndPoint(), point_on_arc_at_distance_from_start_point);
}

