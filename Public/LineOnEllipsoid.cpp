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

#include <public/EllipsoidalEarthModel.h>
#include "public/LineOnEllipsoid.h"
#include "public/GeolibUtils.h"

using namespace aaesim;
using namespace geolib_idealab;

log4cplus::Logger LineOnEllipsoid::m_logger = log4cplus::Logger::getInstance("LineOnEllipsoid");

LineOnEllipsoid::LineOnEllipsoid(const geolib_idealab::Geodesic &geodesic) { m_geolib_geodesic = geodesic; }

const geolib_idealab::Geodesic &LineOnEllipsoid::GetGeolibPrimitiveGeodesic() const { return m_geolib_geodesic; }

Units::SignedAngle LineOnEllipsoid::GetForwardCourseEnuAtStartPoint() const {
   return GeolibUtils::ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(m_geolib_geodesic.startAz));
}

Units::SignedAngle LineOnEllipsoid::GetForwardCourseEnuAtEndPoint() const {
   return GeolibUtils::ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(m_geolib_geodesic.endAz));
}

Units::Length LineOnEllipsoid::GetShapeLength() const { return Units::NauticalMilesLength(m_geolib_geodesic.length); }

LineOnEllipsoid LineOnEllipsoid::CreateFromPoints(const LatitudeLongitudePoint &start_point,
                                                  const LatitudeLongitudePoint &end_point) {
   return GeolibUtils::CreateLineOnEllipsoid(start_point, end_point);
}

LatitudeLongitudePoint LineOnEllipsoid::GetEndPoint() const {
   return LatitudeLongitudePoint::CreateFromGeolibPrimitive(m_geolib_geodesic.endPoint);
}

LatitudeLongitudePoint LineOnEllipsoid::GetStartPoint() const {
   return LatitudeLongitudePoint::CreateFromGeolibPrimitive(m_geolib_geodesic.startPoint);
}

const geolib_idealab::LineType LineOnEllipsoid::GetLineType() const { return geolib_idealab::LineType::SEGMENT; }

bool LineOnEllipsoid::IsPointOnShape(const LatitudeLongitudePoint &test_point) const {
   return GeolibUtils::IsPointOnLine(*this, test_point);
}

LineOnEllipsoid LineOnEllipsoid::CreateExtendedLine(Units::Length extended_distance) const {
   const LatitudeLongitudePoint new_end_point =
         this->GetEndPoint().ProjectDistanceAlongCourse(extended_distance, this->GetForwardCourseEnuAtEndPoint());
   return LineOnEllipsoid::CreateFromPoints(this->GetStartPoint(), new_end_point);
}

ShapeOnEllipsoid::DIRECTION_RELATIVE_TO_SHAPE LineOnEllipsoid::GetRelativeDirection(
      const LatitudeLongitudePoint &point_not_on_shape) const {
   // Don't call parent implementation at all
   DIRECTION_RELATIVE_TO_SHAPE return_this = ShapeOnEllipsoid::UNSET;
   if (IsPointOnShape(point_not_on_shape)) {
      return_this = ShapeOnEllipsoid::ON_SHAPE;
   } else {
      /**
       * Cross Product 1: cross product of ECEF vector pointing to nearest point on the line and ECEF vector pointing to
       * test point Cross Product 2: cross product of ECEF vector pointing to line start point and ECEF vector pointing
       * to nearest position on the line
       */
      EllipsoidalEarthModel earth_model;
      EllipsoidalEarthModel::AbsolutePositionEcef test_point_position_absolute, nearest_point_position_absolute,
            line_start_point_position_absolute;
      std::tuple<LatitudeLongitudePoint, Units::SignedDegreesAngle, Units::Length> perp_info =
            GeolibUtils::FindNearestPointOnLineUsingPerpendicularProjection(*this, point_not_on_shape);
      const LatitudeLongitudePoint nearest_point_on_line = std::get<0>(perp_info);

      // ECEF Vector from center of earth to test point
      EllipsoidalEarthModel::GeodeticPosition test_point_position_geodetic;
      test_point_position_geodetic.latitude = point_not_on_shape.GetLatitude();
      test_point_position_geodetic.longitude = point_not_on_shape.GetLongitude();
      earth_model.ConvertGeodeticToAbsolute(test_point_position_geodetic, test_point_position_absolute);
      const EllipsoidalEarthModel::AbsolutePositionEcef test_point_position_unit_vector =
            test_point_position_absolute.ToUnitVector();

      // ECEF Vector from center of earth to line intersection point
      EllipsoidalEarthModel::GeodeticPosition nearest_point_position_geodetic;
      nearest_point_position_geodetic.latitude = nearest_point_on_line.GetLatitude();
      nearest_point_position_geodetic.longitude = nearest_point_on_line.GetLongitude();
      earth_model.ConvertGeodeticToAbsolute(nearest_point_position_geodetic, nearest_point_position_absolute);
      const EllipsoidalEarthModel::AbsolutePositionEcef nearest_point_position_unit_vector =
            nearest_point_position_absolute.ToUnitVector();

      const EllipsoidalEarthModel::AbsolutePositionEcef cross_product =
            VectorCrossProduct(nearest_point_position_unit_vector, test_point_position_unit_vector).ToUnitVector();
      const Units::SignedDegreesAngle east_west_tolerance(5);  // this is not very sensitive and does not need to be
                                                               // very small
      const bool line_is_almost_directly_east =
            Units::abs(Units::SignedRadiansAngle(GetForwardCourseEnuAtStartPoint())) < east_west_tolerance;
      const bool line_is_almost_directly_west =
            Units::abs(Units::SignedRadiansAngle(GetForwardCourseEnuAtStartPoint()) - Units::PI_RADIANS_ANGLE) <
            east_west_tolerance;
      if (line_is_almost_directly_east) {
         /*
          * Note: the algorithm gets here only when the line being followed is "nearly directly" east.
          */
         if (cross_product.x < Units::zero()) {
            return_this = ShapeOnEllipsoid::LEFT_OF_SHAPE;
         } else {
            return_this = ShapeOnEllipsoid::RIGHT_OF_SHAPE;
         }
      } else if (line_is_almost_directly_west) {
         /*
          * Note: the algorithm gets here only when the line being followed is "nearly directly" west.
          */
         if (cross_product.x > Units::zero()) {
            return_this = ShapeOnEllipsoid::LEFT_OF_SHAPE;
         } else {
            return_this = ShapeOnEllipsoid::RIGHT_OF_SHAPE;
         }
      } else if (Units::SignedRadiansAngle(GetForwardCourseEnuAtStartPoint()) > Units::zero()) {
         if (cross_product.z < Units::zero()) {
            return_this = ShapeOnEllipsoid::LEFT_OF_SHAPE;
         } else {
            return_this = ShapeOnEllipsoid::RIGHT_OF_SHAPE;
         }
      } else {
         if (cross_product.z < Units::zero()) {
            return_this = ShapeOnEllipsoid::RIGHT_OF_SHAPE;
         } else {
            return_this = ShapeOnEllipsoid::LEFT_OF_SHAPE;
         }
      }
      //----------------------------------------------------------------------------------------------------------------
   }

   // done
   return return_this;
}

Units::Length LineOnEllipsoid::GetDistanceToEndPoint(const LatitudeLongitudePoint &latitude_longitude_point) const {
   return ShapeOnEllipsoid::GetDistanceToEndPoint(latitude_longitude_point);
}

LatitudeLongitudePoint LineOnEllipsoid::GetNearestPointOnShape(const LatitudeLongitudePoint &point_not_on_shape) const {
   std::tuple<LatitudeLongitudePoint, Units::SignedAngle, Units::Length> perp_info =
         GeolibUtils::FindNearestPointOnLineUsingPerpendicularProjection(*this, point_not_on_shape);
   return std::get<0>(perp_info);
}

Units::Length LineOnEllipsoid::CalculateDistanceFromPointOnShapeToEnd(
      const LatitudeLongitudePoint &point_on_shape) const {
   return GetEndPoint().CalculateRelationshipBetweenPoints(point_on_shape).first;
}

LatitudeLongitudePoint LineOnEllipsoid::CalculatePointAtDistanceFromStartPoint(
      const Units::Length &distance_along_shape_from_start_point) const {
   LatitudeLongitudePoint point_on_line_at_distance_along_path = GetStartPoint().ProjectDistanceAlongCourse(
         distance_along_shape_from_start_point, GetForwardCourseEnuAtStartPoint());
   return point_on_line_at_distance_along_path;
}
std::pair<Units::SignedAngle, LatitudeLongitudePoint> LineOnEllipsoid::CalculateCourseAtDistanceFromStartPoint(
      const Units::Length &distance_along_shape_from_start_point) const {

   double temp_course_1, temp_course_2, dist_to_point;
   ErrorSet error_set = ErrorCodes::SUCCESS;
   LatitudeLongitudePoint point_on_geodesic =
         CalculatePointAtDistanceFromStartPoint(distance_along_shape_from_start_point);
   double course_ned_at_point = geoCrs(m_geolib_geodesic, point_on_geodesic.GetGeolibPrimitiveLLPoint(), &temp_course_1,
                                       &temp_course_2, &dist_to_point, &error_set, GEOLIB_TOLERANCE, GEOLIB_EPSILON);
   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, GeolibUtils::m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(GeolibUtils::m_basic_error_message);
   }
   const Units::UnsignedRadiansAngle course_ned_to_return(course_ned_at_point);
   return std::make_pair(GeolibUtils::ConvertCourseFromNedToEnu(course_ned_to_return), point_on_geodesic);
}
