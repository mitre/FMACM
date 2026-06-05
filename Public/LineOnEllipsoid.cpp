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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/LineOnEllipsoid.h"

#include <public/EllipsoidalEarthModel.h>

#include <tuple>
#include <utility>

#include "public/GeolibUtils.h"

using namespace aaesim;
using namespace geolib_idealab;

log4cplus::Logger LineOnEllipsoid::m_logger = log4cplus::Logger::getInstance("LineOnEllipsoid");

LineOnEllipsoid::LineOnEllipsoid(const geolib_idealab::Geodesic &geodesic) : geolib_geodesic_(geodesic) {
   ComputeUnitVectorNormalToLineStartEnd();
}

const geolib_idealab::Geodesic &LineOnEllipsoid::GetGeolibPrimitiveGeodesic() const { return geolib_geodesic_; }

Units::SignedAngle LineOnEllipsoid::GetForwardCourseEnuAtStartPoint() const {
   return GeolibUtils::ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(geolib_geodesic_.startAz));
}

Units::SignedAngle LineOnEllipsoid::GetForwardCourseEnuAtEndPoint() const {
   return GeolibUtils::ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(geolib_geodesic_.endAz));
}

Units::Length LineOnEllipsoid::GetShapeLength() const { return Units::NauticalMilesLength(geolib_geodesic_.length); }

LineOnEllipsoid LineOnEllipsoid::CreateFromPoints(const LatitudeLongitudePoint &start_point,
                                                  const LatitudeLongitudePoint &end_point) {
   return GeolibUtils::CreateLineOnEllipsoid(start_point, end_point);
}

LatitudeLongitudePoint LineOnEllipsoid::GetEndPoint() const {
   return LatitudeLongitudePoint::CreateFromGeolibPrimitive(geolib_geodesic_.endPoint);
}

LatitudeLongitudePoint LineOnEllipsoid::GetStartPoint() const {
   return LatitudeLongitudePoint::CreateFromGeolibPrimitive(geolib_geodesic_.startPoint);
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

void LineOnEllipsoid::ComputeUnitVectorNormalToLineStartEnd() {
   EllipsoidalEarthModel earth_model;
   EllipsoidalEarthModel::GeodeticPosition start_point_position_geodetic;
   start_point_position_geodetic.latitude = GetStartPoint().GetLatitude();
   start_point_position_geodetic.longitude = GetStartPoint().GetLongitude();
   EllipsoidalEarthModel::AbsolutePositionEcef start_point_position_absolute;
   earth_model.ConvertGeodeticToAbsolute(start_point_position_geodetic, start_point_position_absolute);
   const EllipsoidalEarthModel::AbsolutePositionEcef start_point_position_unit_vector =
         start_point_position_absolute.ToUnitVector();

   EllipsoidalEarthModel::GeodeticPosition end_point_position_geodetic;
   end_point_position_geodetic.latitude = GetEndPoint().GetLatitude();
   end_point_position_geodetic.longitude = GetEndPoint().GetLongitude();
   EllipsoidalEarthModel::AbsolutePositionEcef end_point_position_absolute;
   earth_model.ConvertGeodeticToAbsolute(end_point_position_geodetic, end_point_position_absolute);
   const EllipsoidalEarthModel::AbsolutePositionEcef end_point_position_unit_vector =
         end_point_position_absolute.ToUnitVector();

   unit_vector_normal_to_line_start_end_ =
         VectorCrossProduct(start_point_position_unit_vector, end_point_position_unit_vector).ToUnitVector();
}

ShapeOnEllipsoid::kDirectionRelativeToShape LineOnEllipsoid::GetRelativeDirection(
      const LatitudeLongitudePoint &point_not_on_shape) const {
   EllipsoidalEarthModel earth_model;

   // Cross product of line start and point not on shape
   std::tuple<LatitudeLongitudePoint, Units::SignedDegreesAngle, Units::Length> perp_info =
         GeolibUtils::FindNearestPointOnLineUsingPerpendicularProjection(*this, point_not_on_shape);
   const LatitudeLongitudePoint nearest_point_on_line = std::get<0>(perp_info);
   EllipsoidalEarthModel::GeodeticPosition nearest_point_position_geodetic;
   nearest_point_position_geodetic.latitude = nearest_point_on_line.GetLatitude();
   nearest_point_position_geodetic.longitude = nearest_point_on_line.GetLongitude();
   EllipsoidalEarthModel::AbsolutePositionEcef nearest_point_position_absolute;
   earth_model.ConvertGeodeticToAbsolute(nearest_point_position_geodetic, nearest_point_position_absolute);

   EllipsoidalEarthModel::GeodeticPosition test_point_position_geodetic;
   test_point_position_geodetic.latitude = point_not_on_shape.GetLatitude();
   test_point_position_geodetic.longitude = point_not_on_shape.GetLongitude();
   EllipsoidalEarthModel::AbsolutePositionEcef test_point_position_absolute;
   earth_model.ConvertGeodeticToAbsolute(test_point_position_geodetic, test_point_position_absolute);
   const EllipsoidalEarthModel::AbsolutePositionEcef vector_to_test_point =
         VectorDifference(nearest_point_position_absolute, test_point_position_absolute);
   const EllipsoidalEarthModel::AbsolutePositionEcef vector_to_test_point_unit_vector =
         vector_to_test_point.ToUnitVector();

   // Use angle between vectors to compute relative direction
   const Units::MetersLength dp =
         VectorDotProduct(unit_vector_normal_to_line_start_end_, vector_to_test_point_unit_vector);
   const double ratio = dp / Units::MetersLength(1.0);
   const auto beta = Units::SignedRadiansAngle{std::acos(ratio)};
   if (beta < Units::HALF_PI_RADIANS_ANGLE) {
      return ShapeOnEllipsoid::RIGHT_OF_SHAPE;
   } else if (beta > Units::HALF_PI_RADIANS_ANGLE) {
      return ShapeOnEllipsoid::LEFT_OF_SHAPE;
   }
   return ShapeOnEllipsoid::ON_SHAPE;
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
   ErrorSet error_set{ErrorCodes::SUCCESS};
   LatitudeLongitudePoint point_on_geodesic =
         CalculatePointAtDistanceFromStartPoint(distance_along_shape_from_start_point);
   double course_ned_at_point = geoCrs(geolib_geodesic_, point_on_geodesic.GetGeolibPrimitiveLLPoint(), &temp_course_1,
                                       &temp_course_2, &dist_to_point, &error_set, GEOLIB_TOLERANCE, GEOLIB_EPSILON);
   if (!GeolibUtils::IsSuccess(error_set)) {
      LOG4CPLUS_ERROR(m_logger, GeolibUtils::m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(GeolibUtils::m_basic_error_message);
   }
   const Units::UnsignedRadiansAngle course_ned_to_return(course_ned_at_point);
   return std::make_pair(GeolibUtils::ConvertCourseFromNedToEnu(course_ned_to_return), point_on_geodesic);
}
