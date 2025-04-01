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

#include <bitset>

#include "public/LatitudeLongitudePoint.h"
#include "public/LineOnEllipsoid.h"
#include "public/ArcOnEllipsoid.h"

namespace aaesim {
//---------------------------------------------
/* Developers:
 * These (epsilon and tolerance) values can be played with (tuned), but the values directly impact the precision of
 * geolib's WGS84 calculations. And, the two values are related. So changing one may require changing both. If you don't
 * understand the low-level details of the geolib algorithms, you probably should -not- be messing with this.
 */
static const double GEOLIB_EPSILON = 1e-20;
static const double GEOLIB_TOLERANCE = 1.37e-9;
static const Units::Length GEOLIB_TOLERANCE_UNITZED = Units::NauticalMilesLength(GEOLIB_TOLERANCE);
//---------------------------------------------

class GeolibUtils {

  public:
   inline static std::string m_basic_error_message{
         "Arg! Something very bad occurred inside a geolib library operation!"};

   inline static bool IsSuccess(const geolib_idealab::ErrorSet &error_set) { return std::bitset<32>(error_set).none(); }

   inline static bool HasErrorBitSet(const geolib_idealab::ErrorSet &error_set, geolib_idealab::ErrorCodes code) {
      if (IsSuccess(error_set)) {
         if (!IsSuccess(code)) return false;
         return true;
      }
      return error_set & code;
   }

   /**
    * Create a line of zero length that has directionality correctly defined.
    *
    * @param location
    * @param course_enu
    * @return
    */
   static LineOnEllipsoid CreateLineOfZeroLength(const LatitudeLongitudePoint &location,
                                                 const Units::SignedAngle &course_enu);

   /**
    *
    * @param point1
    * @param point2
    * @return true if point1 and point2 are mathematically equal on the ellipsoid
    */
   static bool ArePointsMathematicallyEqual(const LatitudeLongitudePoint &point1, const LatitudeLongitudePoint &point2);

   /**
    * From a start_point, project a distance along a course and return that location.
    *
    * @param start_point, the starting location of the projection
    * @param distance, the distance to project
    * @param course_enu, the ENU course to project along
    * @return the new point
    */
   static LatitudeLongitudePoint CalculateNewPoint(const LatitudeLongitudePoint &start_point,
                                                   const Units::Length &distance, const Units::SignedAngle &course_enu);

   /**
    * Define the mathematical relationship between two points on the WGS84 ellipsoid.
    *
    * @param start_point
    * @param end_point
    * @return a pair that contains:
    *    - the wgs84 distance between the points;
    *    - the course from start_point to end_point in the ENU coordinate frame.
    */
   static std::pair<Units::Length, Units::SignedAngle> CalculateRelationshipBetweenPoints(
         const LatitudeLongitudePoint &start_point, const LatitudeLongitudePoint &end_point);

   /**
    * Define the mathematical relationship between two points on the WGS84 ellipsoid.
    *
    * @param start_point
    * @param end_point
    * @return a tuple that contains:
    *    - the wgs84 distance between the points;
    *    - the course from start_point to end_point in the ENU coordinate frame;
    *    - the course from end_point to start_point in the ENU coordinate frame.
    */
   static std::tuple<Units::Length, Units::SignedAngle, Units::SignedAngle> CalculateRelationshipBetweenPointsExpanded(
         const LatitudeLongitudePoint &start_point, const LatitudeLongitudePoint &end_point);

   /**
    * AAESim uses the East-North-Up coordinate frame and the angle is -usually- on the interval [-180, 180]. This means
    * that 0 is east, 90 is north, 180 is west and -90 is south.
    * By constrast, geolib uses an NED coordinate frame and the course angle is on the interval [0, 360]. This means
    * that 0 is north, 90 is east, 180 is south and 270 is west.
    *
    * This method converts course from ENU to NED.
    *
    * @param course_enu
    * @return course_ned
    */
   static const Units::UnsignedAngle ConvertCourseFromEnuToNed(const Units::SignedAngle &course_enu);

   /**
    * AAESim uses the East-North-Up coordinate frame and the angle is -usually- on the interval [-180, 180]. This means
    * that 0 is east, 90 is north, 180 is west and -90 is south.
    * By constrast, geolib uses an NED coordinate frame and the course angle is on the interval [0, 360]. This means
    * that 0 is north, 90 is east, 180 is south and 270 is west.
    *
    * This method converts course from NED to ENU.
    *
    * @param course_ned, an unsigned angle on the interval [0, 360]
    * @return course_enu, a signed angle on the interval [-180, 180]
    */
   static const Units::SignedAngle ConvertCourseFromNedToEnu(const Units::UnsignedAngle &course_ned);

   /**
    *
    * @param start_point
    * @param end_point
    * @return LineOnEllipsoid, fully populated
    */
   static const LineOnEllipsoid CreateLineOnEllipsoid(const LatitudeLongitudePoint &start_point,
                                                      const LatitudeLongitudePoint &end_point);

   /**
    * Create an ArcOnEllipsoid object.
    *
    * NOTE: This is mathematically sensitive! The Latitude/Longitude must be on-the-arc in a double precision sense.
    *
    * @param start_point
    * @param end_point
    * @param center_point
    * @param arc_direction geolib_idealab::ArcDirection which is CLOCKWISE or COUNTERCLOCKWISE
    * @return ArcOnEllipsoid, fully populated
    */
   static const ArcOnEllipsoid CreateArcOnEllipsoid(const LatitudeLongitudePoint &start_point,
                                                    const LatitudeLongitudePoint &end_point,
                                                    const LatitudeLongitudePoint &center_point,
                                                    const geolib_idealab::ArcDirection &arc_direction);

   /**
    * Create a full-circle ArcOnEllipsoid object.
    *
    * @param center_point
    * @param arc_radius
    * @param arc_direction
    * @return
    */
   static const ArcOnEllipsoid CreateFullCircleOnEllipsoid(const LatitudeLongitudePoint &center_point,
                                                           const Units::Length &arc_radius,
                                                           const geolib_idealab::ArcDirection &arc_direction);

   /**
    * Calculate an intersection point of two lines. The returned point is guaranteed to be between the start and end
    * point on both lines.
    *
    * @param line1
    * @param line2
    * @return a tuple that contains:
    * - index 0: bool for valid intersection;
    * - index 1: intersection point as LatitudeLongitudePoint;
    * - index 2: vector of distances between the start point of each line and the intersection point
    */
   static const std::tuple<bool, LatitudeLongitudePoint, std::vector<Units::Length> >
         CalculateLineLineIntersectionPoint(const LineOnEllipsoid &line1, const LineOnEllipsoid &line2);

   /**
    * Check if the provided point is on the line.
    *
    * @param line
    * @param test_point
    * @return true or false
    */
   static const bool IsPointOnLine(const LineOnEllipsoid &line, const LatitudeLongitudePoint &test_point);

   /**
    *
    * @param arc
    * @param test_point
    * @return true or false
    */
   static const bool IsPointOnArc(const ArcOnEllipsoid &arc, const LatitudeLongitudePoint &test_point);

   /**
    * Project from point_not_on_line to the line, making a perpendicular line. Return that point that has been
    * calculated as the projection point.
    *
    * This returned point also defines the nearest location on the line to point_not_on_line.
    *
    * @param line
    * @param point_not_on_line
    * @return a tuple that contains:
    * - the point on the line that creates a perpendicular angle to the line when connected to point_not_on_line
    * - an ENU course from point_not_on_line to the point this is returned
    * - a distance from the point_not_on_line to the point that is on the line
    */
   static std::tuple<LatitudeLongitudePoint, Units::SignedAngle, Units::Length>
         FindNearestPointOnLineUsingPerpendicularProjection(const LineOnEllipsoid &line,
                                                            const LatitudeLongitudePoint &point_not_on_line);

   /**
    * Checks the test point to see if it lies in the boundary of the arc segment, taking the start and end points of
    * the arc into account.
    *
    * @param finite_arc
    * @param test_point
    * @return true if inside the arc segment, false otherwise
    */
   static const bool IsPointInsideArcSegment(const ArcOnEllipsoid &finite_arc,
                                             const LatitudeLongitudePoint &test_point);

   /**
    * Calculates a point on the arc that is the nearest to the point_not_on_arc. The point returned is constrained to
    * be on the finite arc, taking into account its start and end points.
    *
    * @param arc
    * @param point_not_on_arc
    * @return a pair of boolean and LatitudeLongitudePoint
    */
   static std::pair<bool, const LatitudeLongitudePoint> FindNearestPointOnArcUsingPerpendiculorProjection(
         const ArcOnEllipsoid &arc, const LatitudeLongitudePoint &point_not_on_arc);

   /**
    * For two given lines, calculate an arc that is tangent to both lines and that has a specific radius.
    *
    * @param line1
    * @param line2
    * @param required_radius
    * @return a pair that contains a boolean and an ArcOnEllipsoid. The boolean will be false if no arc could be found,
    * true otherwise.
    */
   static std::pair<bool, ArcOnEllipsoid> CreateArcTangentToTwoLines(const LineOnEllipsoid &line1,
                                                                     const LineOnEllipsoid &line2,
                                                                     const Units::Length &required_radius);

   /**
    * For a shape that is inbound to an arc, and that has an end point where the arc must begin, and also given a
    * location where the arc must end, find an arc that fits.
    *
    * @param inbound_line
    * @param end_point
    * @return a fully formed ArcOnEllipsoid object
    * @throws std::runtime_error
    */
   static ArcOnEllipsoid CreateArcFromInboundShapeAndEndPoint(const ShapeOnEllipsoid *inbound_shape,
                                                              const LatitudeLongitudePoint &end_point);

   /**
    * For a given line and arc, calculate any points of intersection.
    *
    * @param line
    * @param arc
    * @return a vector of pair. Each pair contains a boolean and a point. The boolean is true if the point lies on both
    * the arc and the line and false otherwise.
    */
   static std::vector<std::pair<bool, LatitudeLongitudePoint> > CalculateLineArcIntersectionPoints(
         const LineOnEllipsoid &line, const ArcOnEllipsoid &arc);

   /**
    * Build a reverse line.
    */
   static const LineOnEllipsoid ReverseLine(const LineOnEllipsoid &line) {
      return GeolibUtils::CreateLineOnEllipsoid(line.GetEndPoint(), line.GetStartPoint());
   }

   /**
    * Build a reverse arc.
    */
   static const ArcOnEllipsoid ReverseArc(const ArcOnEllipsoid &arc) {
      geolib_idealab::ArcDirection reversed_direction;
      if (arc.GetArcDirection() == geolib_idealab::ArcDirection::CLOCKWISE) {
         reversed_direction = geolib_idealab::ArcDirection::COUNTERCLOCKWISE;
      } else {
         reversed_direction = geolib_idealab::ArcDirection::CLOCKWISE;
      }
      return GeolibUtils::CreateArcOnEllipsoid(arc.GetEndPoint(), arc.GetStartPoint(), arc.GetCenterPoint(),
                                               reversed_direction);
   }

  private:
   inline static log4cplus::Logger m_logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("GeolibUtils"))};
};

}  // namespace aaesim
