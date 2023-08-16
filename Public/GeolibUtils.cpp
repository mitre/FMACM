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

#include <cfloat>
#include <cmath>
#include "public/GeolibUtils.h"

using namespace aaesim;
using namespace geolib_idealab;

log4cplus::Logger GeolibUtils::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("GeolibUtils"));
std::string GeolibUtils::m_basic_error_message("Arg! Something very bad occurred inside a geolib library operation!");

LatitudeLongitudePoint GeolibUtils::CalculateNewPoint(const LatitudeLongitudePoint &start_point,
                                                      const Units::Length &distance,
                                                      const Units::SignedAngle &course_enu) {
   const Units::UnsignedRadiansAngle angle_from_true_north = ConvertCourseFromEnuToNed(course_enu);

   // Project self to a new location and return that location
   LLPoint destination_calculated;
   ErrorSet error_set = direct(start_point.GetGeolibPrimitiveLLPoint(), angle_from_true_north.value(),
                               Units::NauticalMilesLength(distance).value(), &destination_calculated, GEOLIB_EPSILON);
   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }
   return LatitudeLongitudePoint(Units::RadiansAngle(destination_calculated.latitude),
                                 Units::RadiansAngle(destination_calculated.longitude));
}

std::pair<Units::Length, Units::SignedAngle> GeolibUtils::CalculateRelationshipBetweenPoints(
      const LatitudeLongitudePoint &start_point, const LatitudeLongitudePoint &end_point) {
   auto info = CalculateRelationshipBetweenPointsExpanded(start_point, end_point);
   return std::make_pair(std::get<0>(info), std::get<1>(info));
}

const Units::UnsignedAngle GeolibUtils::ConvertCourseFromEnuToNed(const Units::SignedAngle &course_enu) {
   const double cos_term = cos(course_enu);
   const double sin_term = sin(course_enu);
   const double arctan2_result = std::atan2(cos_term, sin_term);
   const Units::SignedRadiansAngle tmp(arctan2_result);
   return Units::UnsignedAngle(tmp);
}

const Units::SignedAngle GeolibUtils::ConvertCourseFromNedToEnu(const Units::UnsignedAngle &course_ned) {
   const double cos_term = cos(Units::SignedRadiansAngle(course_ned));
   const double sin_term = sin(Units::SignedRadiansAngle(course_ned));
   const double arctan2_result = std::atan2(cos_term, sin_term);
   return Units::SignedRadiansAngle(arctan2_result);
}

const LineOnEllipsoid GeolibUtils::CreateLineOnEllipsoid(const LatitudeLongitudePoint &start_point,
                                                         const LatitudeLongitudePoint &end_point) {
   Geodesic geolib_geodesic;
   ErrorSet error_set = createGeo(&geolib_geodesic, start_point.GetGeolibPrimitiveLLPoint(),
                                  end_point.GetGeolibPrimitiveLLPoint(), LineType::SEGMENT, GEOLIB_EPSILON);
   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   LineOnEllipsoid line_on_ellipsoid(geolib_geodesic);
   return line_on_ellipsoid;
}
const ArcOnEllipsoid GeolibUtils::CreateArcOnEllipsoid(const LatitudeLongitudePoint &start_point,
                                                       const LatitudeLongitudePoint &end_point,
                                                       const LatitudeLongitudePoint &center_point,
                                                       const geolib_idealab::ArcDirection &arc_direction) {

   Arc arc_primitive;
   ErrorSet error_set =
         createArc(&arc_primitive, center_point.GetGeolibPrimitiveLLPoint(), start_point.GetGeolibPrimitiveLLPoint(),
                   end_point.GetGeolibPrimitiveLLPoint(), arc_direction, GEOLIB_TOLERANCE, GEOLIB_EPSILON);
   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   ArcOnEllipsoid arc_on_ellipsoid(arc_primitive);
   return arc_on_ellipsoid;
}

const std::tuple<bool, LatitudeLongitudePoint, std::vector<Units::Length> >
      GeolibUtils::CalculateLineLineIntersectionPoint(const LineOnEllipsoid &line1, const LineOnEllipsoid &line2) {

   // Line-Line intersection
   double crs31, distance_line1_start_to_intx_point, crs32, distance_line2_start_to_intx_point;
   geolib_idealab::LLPoint intersection_point;
   ErrorSet error_set =
         geoIntx(line1.GetStartPoint().GetGeolibPrimitiveLLPoint(), line1.GetEndPoint().GetGeolibPrimitiveLLPoint(),
                 line1.GetLineType(), &crs31, &distance_line1_start_to_intx_point,
                 line2.GetStartPoint().GetGeolibPrimitiveLLPoint(), line2.GetEndPoint().GetGeolibPrimitiveLLPoint(),
                 line2.GetLineType(), &crs32, &distance_line2_start_to_intx_point, &intersection_point,
                 GEOLIB_TOLERANCE, GEOLIB_EPSILON);

   bool return_this = true;
   if (error_set && ErrorCodes::NO_INTERSECTION_ERR) {
      LOG4CPLUS_WARN(m_logger, formatErrorMessage(error_set));
      return_this = false;
   } else if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   std::vector<Units::Length> distances_to_intersection_point = {
         Units::NauticalMilesLength(distance_line1_start_to_intx_point),
         Units::NauticalMilesLength(distance_line2_start_to_intx_point)};
   return std::make_tuple(return_this, LatitudeLongitudePoint::CreateFromGeolibPrimitive(intersection_point),
                          distances_to_intersection_point);
}
const bool GeolibUtils::IsPointOnLine(const LineOnEllipsoid &line, const LatitudeLongitudePoint &test_point) {
   geolib_idealab::ErrorSet error_set = ErrorCodes::SUCCESS;
   int ret = geolib_idealab::ptIsOnGeo(
         line.GetStartPoint().GetGeolibPrimitiveLLPoint(), line.GetEndPoint().GetGeolibPrimitiveLLPoint(),
         test_point.GetGeolibPrimitiveLLPoint(), line.GetLineType(), &error_set, GEOLIB_TOLERANCE, GEOLIB_EPSILON);
   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   bool return_this = (ret != 0);
   return return_this;
}
const bool GeolibUtils::IsPointOnArc(const ArcOnEllipsoid &arc, const LatitudeLongitudePoint &test_point) {
   geolib_idealab::ErrorSet error_set = ErrorCodes::SUCCESS;
   int ret = geolib_idealab::ptIsOnArc(
         arc.GetCenterPoint().GetGeolibPrimitiveLLPoint(), Units::NauticalMilesLength(arc.GetRadius()).value(),
         Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(arc.GetStartAzimuthEnu())).value(),
         Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(arc.GetEndAzimuthEnu())).value(),
         arc.GetArcDirection(), test_point.GetGeolibPrimitiveLLPoint(), &error_set, GEOLIB_TOLERANCE, GEOLIB_EPSILON);
   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   bool return_this = (ret == 1);
   return return_this;
}

std::tuple<LatitudeLongitudePoint, Units::SignedAngle, Units::Length>
      GeolibUtils::FindNearestPointOnLineUsingPerpendicularProjection(const LineOnEllipsoid &line,
                                                                      const LatitudeLongitudePoint &point_not_on_line) {
   geolib_idealab::LLPoint point_on_line;
   double crs_ned_to_line_from_point_not_on_line, distance_along_perpendicular_projection;
   ErrorSet error_set = projectToGeo(
         line.GetStartPoint().GetGeolibPrimitiveLLPoint(),
         Units::UnsignedRadiansAngle(ConvertCourseFromEnuToNed(line.GetForwardCourseEnuAtStartPoint())).value(),
         point_not_on_line.GetGeolibPrimitiveLLPoint(), &point_on_line, &crs_ned_to_line_from_point_not_on_line,
         &distance_along_perpendicular_projection, GEOLIB_TOLERANCE, GEOLIB_EPSILON);

   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   return std::make_tuple(
         LatitudeLongitudePoint::CreateFromGeolibPrimitive(point_on_line),
         ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(crs_ned_to_line_from_point_not_on_line)),
         Units::NauticalMilesLength(distance_along_perpendicular_projection));
}

const bool GeolibUtils::IsPointInsideArcSegment(const ArcOnEllipsoid &finite_arc,
                                                const LatitudeLongitudePoint &test_point) {
   geolib_idealab::ErrorSet error_set = ErrorCodes::SUCCESS;
   int ret = geolib_idealab::ptIsInsideArc(
         finite_arc.GetCenterPoint().GetGeolibPrimitiveLLPoint(),
         Units::NauticalMilesLength(finite_arc.GetRadius()).value(),
         Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(finite_arc.GetStartAzimuthEnu())).value(),
         Units::UnsignedRadiansAngle(GeolibUtils::ConvertCourseFromEnuToNed(finite_arc.GetEndAzimuthEnu())).value(),
         finite_arc.GetArcDirection(), test_point.GetGeolibPrimitiveLLPoint(), &error_set, GEOLIB_TOLERANCE,
         GEOLIB_EPSILON);
   if (error_set != ErrorCodes::SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   bool return_this = (ret != 0);
   return return_this;
}
std::pair<bool, const LatitudeLongitudePoint> GeolibUtils::FindNearestPointOnArcUsingPerpendiculorProjection(
      const ArcOnEllipsoid &arc, const LatitudeLongitudePoint &point_not_on_arc) {
   // line from arc center to point_not_on_arc
   LineOnEllipsoid radius_line_through_point =
         LineOnEllipsoid::CreateFromPoints(arc.GetCenterPoint(), point_not_on_arc);

   // Get new point at arc-radius distance from center along the line's course
   LatitudeLongitudePoint point_on_arc = GeolibUtils::CalculateNewPoint(
         arc.GetCenterPoint(), arc.GetRadius(), radius_line_through_point.GetForwardCourseEnuAtStartPoint());

   // Make sure it is on the arc
   bool is_on_arc = arc.IsPointOnShape(point_on_arc);

   return std::pair<bool, const LatitudeLongitudePoint>(is_on_arc, point_on_arc);
}

bool GeolibUtils::ArePointsMathematicallyEqual(const LatitudeLongitudePoint &point1,
                                               const LatitudeLongitudePoint &point2) {
   int ret = geolib_idealab::ptsAreSame(point1.GetGeolibPrimitiveLLPoint(), point2.GetGeolibPrimitiveLLPoint(),
                                        GEOLIB_TOLERANCE);

   bool return_this = (ret != 0);
   return return_this;
}

std::pair<bool, ArcOnEllipsoid> GeolibUtils::CreateArcTangentToTwoLines(const LineOnEllipsoid &line1,
                                                                        const LineOnEllipsoid &line2,
                                                                        const Units::Length &required_radius) {

   LLPoint arc_center_point;
   LLPoint arc_start_point;
   LLPoint arc_end_point;
   geolib_idealab::ArcDirection arc_direction;
   ErrorSet error_set = arcTanToTwoGeos(
         line1.GetStartPoint().GetGeolibPrimitiveLLPoint(),
         Units::UnsignedRadiansAngle(ConvertCourseFromEnuToNed(line1.GetForwardCourseEnuAtStartPoint())).value(),
         line2.GetStartPoint().GetGeolibPrimitiveLLPoint(),
         Units::UnsignedRadiansAngle(ConvertCourseFromEnuToNed(line2.GetForwardCourseEnuAtStartPoint())).value(),
         Units::NauticalMilesLength(required_radius).value(), &arc_center_point, &arc_start_point, &arc_end_point,
         &arc_direction, GEOLIB_TOLERANCE, GEOLIB_EPSILON);

   bool one_arc_found = true;
   ArcOnEllipsoid arc_to_return;
   if (error_set == ErrorCodes::SUCCESS) {
      arc_to_return =
            CreateArcOnEllipsoid(LatitudeLongitudePoint::CreateFromGeolibPrimitive(arc_start_point),
                                 LatitudeLongitudePoint::CreateFromGeolibPrimitive(arc_end_point),
                                 LatitudeLongitudePoint::CreateFromGeolibPrimitive(arc_center_point), arc_direction);
   } else if (error_set && ErrorCodes::NO_TANGENT_ARC_ERR) {
      // this is okay, just return with no formed arc
      one_arc_found = false;
   } else {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   return std::pair<bool, ArcOnEllipsoid>(one_arc_found, arc_to_return);
}

ArcOnEllipsoid GeolibUtils::CreateArcFromInboundShapeAndEndPoint(const ShapeOnEllipsoid *inbound_shape,
                                                                 const LatitudeLongitudePoint &end_point) {

   geolib_idealab::Arc calculated_arc;
   ErrorSet error_set = arcFromStartAndEnd(
         inbound_shape->GetEndPoint().GetGeolibPrimitiveLLPoint(),
         Units::UnsignedRadiansAngle(ConvertCourseFromEnuToNed(inbound_shape->GetForwardCourseEnuAtEndPoint())).value(),
         end_point.GetGeolibPrimitiveLLPoint(), &calculated_arc, GEOLIB_TOLERANCE, GEOLIB_EPSILON);

   ArcOnEllipsoid arc_to_return;
   if (error_set == ErrorCodes::SUCCESS) {
      arc_to_return = ArcOnEllipsoid(calculated_arc);
   } else {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   return arc_to_return;
}

std::vector<std::pair<bool, LatitudeLongitudePoint> > GeolibUtils::CalculateLineArcIntersectionPoints(
      const LineOnEllipsoid &line, const ArcOnEllipsoid &arc) {

   LLPointPair intersection_pairs_on_circle;
   int number_of_intersections = INT32_MIN;
   ErrorSet error_set = geoArcIntx(
         line.GetStartPoint().GetGeolibPrimitiveLLPoint(),
         Units::UnsignedRadiansAngle(ConvertCourseFromEnuToNed(line.GetForwardCourseEnuAtStartPoint())).value(),
         arc.GetCenterPoint().GetGeolibPrimitiveLLPoint(), Units::NauticalMilesLength(arc.GetRadius()).value(),
         intersection_pairs_on_circle, &number_of_intersections, GEOLIB_TOLERANCE, GEOLIB_EPSILON);

   std::vector<std::pair<bool, LatitudeLongitudePoint> > return_this;
   if (error_set == ErrorCodes::SUCCESS) {
      if (number_of_intersections == 0) {
         // no intersections found
         auto no_intersection_return = {
               std::make_pair(false, LatitudeLongitudePoint(Units::ZERO_ANGLE, Units::ZERO_ANGLE))};
         return no_intersection_return;
      } else {
         for (auto i = 0; i < number_of_intersections; ++i) {
            const LatitudeLongitudePoint intx_point =
                  LatitudeLongitudePoint::CreateFromGeolibPrimitive(intersection_pairs_on_circle[i]);
            const bool is_point_on_line = line.IsPointOnShape(intx_point);
            const bool is_point_on_arc = arc.IsPointOnShape(intx_point);
            return_this.push_back(std::make_pair(is_point_on_arc && is_point_on_line, intx_point));
         }
      }
   } else {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   return return_this;
}
const ArcOnEllipsoid GeolibUtils::CreateFullCircleOnEllipsoid(const LatitudeLongitudePoint &center_point,
                                                              const Units::Length &arc_radius,
                                                              const geolib_idealab::ArcDirection &arc_direction) {

   geolib_idealab::Arc calculated_arc;
   LatitudeLongitudePoint start_point = center_point.ProjectDistanceAlongCourse(arc_radius, Units::ZERO_ANGLE);
   LLPoint end_point = start_point.GetGeolibPrimitiveLLPoint();
   ErrorSet error_set =
         arcEndFromStartAndCenter(start_point.GetGeolibPrimitiveLLPoint(), 0, center_point.GetGeolibPrimitiveLLPoint(),
                                  arc_direction, end_point, &calculated_arc, GEOLIB_TOLERANCE, GEOLIB_EPSILON);

   ArcOnEllipsoid arc_to_return;
   if (error_set == ErrorCodes::SUCCESS) {
      arc_to_return = ArcOnEllipsoid(calculated_arc);
   } else {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   return arc_to_return;
}

LineOnEllipsoid GeolibUtils::CreateLineOfZeroLength(const LatitudeLongitudePoint &location,
                                                    const Units::SignedAngle &course_enu) {

   geolib_idealab::Geodesic zero_length_geodesic;
   zero_length_geodesic.length = 0;
   zero_length_geodesic.startPoint = location.GetGeolibPrimitiveLLPoint();
   zero_length_geodesic.endPoint = zero_length_geodesic.startPoint;
   zero_length_geodesic.startAz = Units::UnsignedRadiansAngle(ConvertCourseFromEnuToNed(course_enu)).value();
   zero_length_geodesic.endAz = zero_length_geodesic.startAz;
   zero_length_geodesic.lineType = SEGMENT;
   return LineOnEllipsoid(zero_length_geodesic);
}

std::tuple<Units::Length, Units::SignedAngle, Units::SignedAngle>
      GeolibUtils::CalculateRelationshipBetweenPointsExpanded(const LatitudeLongitudePoint &start_point,
                                                              const LatitudeLongitudePoint &end_point) {
   double start_crs_radians_ned_unsigned = DBL_MIN;
   double end_course_radians_ned_unsigned = DBL_MIN;
   double distance_nm = DBL_MIN;
   ErrorSet error_set =
         inverse(start_point.GetGeolibPrimitiveLLPoint(), end_point.GetGeolibPrimitiveLLPoint(),
                 &start_crs_radians_ned_unsigned, &end_course_radians_ned_unsigned, &distance_nm, GEOLIB_EPSILON);
   if (error_set != SUCCESS) {
      LOG4CPLUS_ERROR(m_logger, m_basic_error_message << formatErrorMessage(error_set));
      throw std::runtime_error(m_basic_error_message);
   }

   return std::make_tuple(Units::NauticalMilesLength(distance_nm),
                          ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(start_crs_radians_ned_unsigned)),
                          ConvertCourseFromNedToEnu(Units::UnsignedRadiansAngle(end_course_radians_ned_unsigned)));
}
