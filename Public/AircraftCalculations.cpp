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

#include "public/AircraftCalculations.h"
#include <stdexcept>
#include <public/CoreUtils.h>
#include <public/AircraftCalculations.h>
#include <public/PositionCalculator.h>

using namespace std;
using namespace aaesim::open_source;

log4cplus::Logger AircraftCalculations::logger = log4cplus::Logger::getInstance("AircraftCalculations");

bool AircraftCalculations::LegacyGetPositionFromPathLength(const Units::Length &distance_to_go,
                                                           const std::vector<HorizontalPath> &horizontal_trajectory,
                                                           Units::Length &x_position, Units::Length &y_position,
                                                           Units::UnsignedAngle &course, int &traj_index) {

   PositionCalculator position_calculator(horizontal_trajectory, TrajectoryIndexProgressionDirection::UNDEFINED);
   bool is_valid =
         position_calculator.CalculatePositionFromAlongPathDistance(distance_to_go, x_position, y_position, course);

   traj_index = static_cast<int>(position_calculator.GetCurrentTrajectoryIndex());
   if (distance_to_go < Units::zero()) {
      traj_index = 0;
   }

   return is_valid;
}

void AircraftCalculations::LegacyGetPathLengthFromPosition(const Units::Length x, const Units::Length y,
                                                           const vector<HorizontalPath> &horizontal_trajectory,
                                                           Units::Length &distance_along_path, Units::Angle &course) {
   vector<HorizontalPath>::size_type ignored_trajectory_index;
   const std::vector<HorizontalPath>::size_type default_start_index = 0;
   CalculateDistanceAlongPathFromPosition(x, y, horizontal_trajectory, default_start_index, distance_along_path, course,
                                          ignored_trajectory_index);
}

vector<AircraftCalculations::PathDistance> AircraftCalculations::ComputePathDistances(
      const Units::Length x, const Units::Length y, const std::vector<HorizontalPath>::size_type &starting_index,
      const vector<HorizontalPath> &hTraj) {

   // returns distances and cross track errors for all points in a vector
   //         ordered ascending by distance.

   vector<PathDistance> path_distances;

   for (auto i = starting_index; i < hTraj.size(); ++i) {

      PathDistance pd;

      Units::MetersLength d = sqrt(Units::sqr(x - Units::MetersLength(hTraj[i].GetXPositionMeters())) +
                                   Units::sqr(y - Units::MetersLength(hTraj[i].GetYPositionMeters())));
      pd.m_distance_to_path_node = d;
      pd.m_horizontal_path_index = i;

      if (std::isnan(d.value())) {
         string msg = "undefined distance (NaN) computed for path distance";
         LOG4CPLUS_FATAL(logger, msg);
      }

      if (path_distances.empty()) {
         path_distances.push_back(pd);
      } else {

         auto path_distance_iter = path_distances.begin();

         while (path_distance_iter < path_distances.end()) {
            if ((*path_distance_iter).m_distance_to_path_node > pd.m_distance_to_path_node) {
               break;
            }

            path_distance_iter++;
         }

         path_distances.insert(path_distance_iter, pd);
      }
   }

   return path_distances;
}

Units::NauticalMilesLength AircraftCalculations::PtToPtDist(Units::Length x0, Units::Length y0, Units::Length x1,
                                                            Units::Length y1) {
   // Computes distance between points.
   //
   // x0:x component of first point in feet.
   // y0:y component of first point in feet.
   // x1:x component of last point in feet.
   // y1:y component of last point in feet.
   //
   // returns distance between (x0,y0) to (x1,y1) in nmi.

   Units::NauticalMilesLength dist = sqrt(Units::sqr(x1 - x0) + Units::sqr(y1 - y0));

   return dist;
}

void AircraftCalculations::CrossTrackError(const Units::Length position_enu_x, const Units::Length position_enu_y,
                                           int current_trajectory_index,
                                           const vector<HorizontalPath> &horizontal_trajectory,
                                           int &next_trajectory_index, Units::Length &cross_track_error) {

   //  Calculates the cross track for a segment leading to a horizontal
   //  trajectory point.  The calculation is performed differently for a
   //  turn segment and for a straight segment.
   //
   //  Turn Segment:
   //  Position is tested to see if it is in the space formed by the two
   //  directed line segments that form the turn arc.
   //  First, determine the turn orientation: see if the points center of
   //  turn, start turn, and end turn are clockwise or counter-clockwise.
   //  If the orientation of the points center of turn, start turn, position
   //  do not have the same orientation, then return a large number for cte.
   //  If the orientation of the points center of turn, turn stop, position
   //  does not have the opposite orientation, return a large number for cte.
   //  If the conditions are met, then return the distance from the turn
   //  center minus the radius for cte.  It is possible for two sequential
   //  turn segments to be slightly misaligned so that a position could be
   //  after one segment and before another segment.  In this case, the
   //  position will be considered as on the next segment.
   //
   //  Straight Segment:
   //  Straight segments could have a small turn, and still be considered
   //  straight. A perpendicular to the m_path_course segment through the
   //  position is calculated.  If this perpendicular intersects the
   //  segment, its distance from the segment is returned.  If the position
   //  is after the horizontal trajectory point, a large value for cte is
   //  returned.  If the position is before the beginning of the segment,
   //  the distance from the position to the beginning of the segment
   //  (preceding horizontal trajectory point) is returned for cte and the
   //  horizontal trajectory point is returned for nextTrajIx.
   //
   //  In either case, the preceding segment could be of the other type and
   //  a gap could exist.  Must test if in the gap and not within the
   //  preceding segment.

   cross_track_error = Units::Infinity();
   next_trajectory_index = (int)-INFINITY;

   if (current_trajectory_index >= horizontal_trajectory.size() - 1) {
      // No segment going to first point on route (last trajectory point)
      return;
   }

   if (horizontal_trajectory[current_trajectory_index].m_segment_type == HorizontalPath::SegmentType::TURN) {
      // Compute cross track error for turn.
      const double x0 = horizontal_trajectory[current_trajectory_index].m_turn_info.x_position_meters;  // turn center
      const double y0 = horizontal_trajectory[current_trajectory_index].m_turn_info.y_position_meters;
      const double x1 = horizontal_trajectory[current_trajectory_index + 1].GetXPositionMeters();  // start turn
      const double y1 = horizontal_trajectory[current_trajectory_index + 1].GetYPositionMeters();
      const double x2 = horizontal_trajectory[current_trajectory_index].GetXPositionMeters();  // stop turn
      const double y2 = horizontal_trajectory[current_trajectory_index].GetYPositionMeters();

      const double dx = Units::MetersLength(position_enu_x).value();
      const double dy = Units::MetersLength(position_enu_y).value();

      // determine orientation of turn.  If crossproduct is zero, then points are colinear
      // sign(P0, P1, p2) i.e., center, start, end.
      double crossProdSign0 = (y0 - y1) * x2 + (x1 - x0) * y2 + (x0 * y1 - x1 * y0);
      if (crossProdSign0 == 0) {  // zero length turn
         return;
      }

      // determine orientation of turn center, start turn, position
      const double crossProdSign1 = (y0 - y1) * dx + (x1 - x0) * dy + (x0 * y1 - x1 * y0);
      if (((crossProdSign0 > 0) && (crossProdSign1 < 0)) || ((crossProdSign0 < 0) && (crossProdSign1 > 0))) {
         // position is before start turn
         // need to test if not in previous segment, i.e. in the V between two segments (see AAES-360)
         if (horizontal_trajectory[current_trajectory_index + 1].m_segment_type == HorizontalPath::SegmentType::TURN) {
            const double x3 = horizontal_trajectory[current_trajectory_index + 1].m_turn_info.x_position_meters;
            const double y3 = horizontal_trajectory[current_trajectory_index + 1].m_turn_info.y_position_meters;
            const double crossProdSign3 = (y3 - y1) * dx + (x1 - x3) * dy + (x3 * y1 - x1 * y3);
            if (((crossProdSign3 > 0) && (crossProdSign1 < 0)) ||
                ((crossProdSign3 < 0) && (crossProdSign1 > 0))) {  // not in previous section - so calculate error
               next_trajectory_index = current_trajectory_index;
               cross_track_error =
                     abs(Units::MetersLength(sqrt(pow(x0 - dx, 2) + pow(y0 - dy, 2))) -
                         Units::MetersLength(horizontal_trajectory[current_trajectory_index].m_turn_info.radius));
            }      // otherwise return not in segment
         } else {  // previous segment is straight
            // Use Pythagorean Theorem
            if (current_trajectory_index >= horizontal_trajectory.size() - 2) {
               return;
            }
            const double x3 = horizontal_trajectory[current_trajectory_index + 2].GetXPositionMeters();
            const double y3 = horizontal_trajectory[current_trajectory_index + 2].GetYPositionMeters();
            if (pow(x3 - dx, 2) + pow(y3 - dy, 2) >
                (pow(x1 - dx, 2) + pow(y1 - dy, 2) + pow(x3 - x1, 2) + pow(y3 - y1, 2))) {
               // not in preceding segment so calculate cte
               cross_track_error = abs(Units::MetersLength(sqrt(pow(x1 - dx, 2) + pow(y1 - dy, 2))));
               next_trajectory_index = current_trajectory_index;
            }
         }
         return;  // position is before start turn
      }

      const double crossProdSign2 = (y0 - y2) * dx + (x2 - x0) * dy + (x0 * y2 - x2 * y0);
      if (((crossProdSign1 > 0) && (crossProdSign2 > 0)) || ((crossProdSign1 < 0) && (crossProdSign2 < 0))) {
         if (current_trajectory_index == 0) {
            Units::MetersLength endDistance = abs(Units::MetersLength(sqrt(pow(x2 - dx, 2) + pow(y2 - dy, 2))));
            if (endDistance < Units::MetersLength(500)) {
               cross_track_error = endDistance;
               next_trajectory_index = current_trajectory_index;
            }
         }
         return;  // position is after stop turn
      }

      cross_track_error = abs(Units::MetersLength(sqrt(pow(x0 - dx, 2) + pow(y0 - dy, 2))) -
                              Units::MetersLength(horizontal_trajectory[current_trajectory_index].m_turn_info.radius));
      next_trajectory_index = current_trajectory_index;
      return;

   } else if (horizontal_trajectory[current_trajectory_index].m_segment_type == HorizontalPath::SegmentType::STRAIGHT) {

      const double x0 = horizontal_trajectory[current_trajectory_index + 1].GetXPositionMeters();  // start of segment
      const double y0 = horizontal_trajectory[current_trajectory_index + 1].GetYPositionMeters();
      const double x1 = horizontal_trajectory[current_trajectory_index].GetXPositionMeters();  // end of segment
      const double y1 = horizontal_trajectory[current_trajectory_index].GetYPositionMeters();
      const double dx = Units::MetersLength(position_enu_x).value();  // position
      const double dy = Units::MetersLength(position_enu_y).value();

      const double vx = x1 - x0;
      const double vy = y1 - y0;
      const double wx = dx - x0;
      const double wy = dy - y0;

      // test for zero length leg
      if (fabs(vx) < 0.00001 && fabs(vy) < 0.00001) {
         // is the query point at the same spot?
         if (fabs(wx) < 0.00001 && fabs(wy) < 0.00001) {
            cross_track_error = Units::zero();
            next_trajectory_index = current_trajectory_index;
         }
         return;
      }

      const double c1 = vx * wx + vy * wy;
      if (c1 < 0) {  // before start of segment
         // See if in preceding segment
         if (current_trajectory_index + 2 < horizontal_trajectory.size()) {
            if (horizontal_trajectory[current_trajectory_index + 1].m_segment_type ==
                HorizontalPath::SegmentType::STRAIGHT) {
               const double x2 = horizontal_trajectory[current_trajectory_index + 2].GetXPositionMeters();
               const double y2 = horizontal_trajectory[current_trajectory_index + 2].GetYPositionMeters();
               const double ux = x2 - x0;
               const double uy = y2 - y0;
               const double c3 = ux * wx + uy * wy;
               if (c3 >= 0) {
                  // position is in the preceding segment
                  return;
               }
            } else {  // previous segment is turn}
               const double x3 =
                     horizontal_trajectory[current_trajectory_index].m_turn_info.x_position_meters;  // turn center
               const double y3 = horizontal_trajectory[current_trajectory_index].m_turn_info.y_position_meters;
               const double x2 =
                     horizontal_trajectory[current_trajectory_index + 1].GetXPositionMeters();  // start turn
               const double y2 = horizontal_trajectory[current_trajectory_index + 1].GetYPositionMeters();
               const double crossProdSign0 = (y3 - y0) * x2 + (x0 - x3) * y2 + (x3 * y0 - x0 * y3);
               if (crossProdSign0 == 0) {  // zero length turn
                  return;
               }
               const double crossProdSign1 = (y3 - y0) * dx + (x0 - x3) * dy + (x3 * y0 - x0 * y3);
               if (((crossProdSign0 > 0) && (crossProdSign1 > 0)) || ((crossProdSign0 < 0) && (crossProdSign1 < 0))) {
                  return;  // in previous section
               }
            }
         }
         // Not in preeceding segment so include with this segment
         cross_track_error = abs(Units::MetersLength(sqrt(pow(x0 - dx, 2) + pow(y0 - dy, 2))));
         next_trajectory_index = current_trajectory_index;
         return;
      }

      const double c2 = vx * vx + vy * vy;
      if (c2 < c1) {  // after end of segment
         // check that not at end of route (trajIx == 0)
         if (current_trajectory_index > 0) {
            return;
         }
         // Need to include positions after end of route or simulation will
         // not terminate properly
         cross_track_error = abs(Units::MetersLength(sqrt(pow(x1 - dx, 2) + pow(y1 - dy, 2))));
         next_trajectory_index = current_trajectory_index;
         return;
      }

      // position is between the end points of the segment
      const double b = c1 / c2;
      const double xb = x0 + b * vx;
      const double yb = y0 + b * vy;
      cross_track_error = abs(Units::MetersLength(sqrt(pow(xb - dx, 2) + pow(yb - dy, 2))));
      next_trajectory_index = current_trajectory_index;
      return;

   } else {
      // Error
      throw logic_error("Invalid segment_type condition found in crossTrackError");
   }

}  // crossTrackError()

Units::SignedRadiansAngle AircraftCalculations::ComputeAngleBetweenVectors(
      const Units::Length &xvertex, const Units::Length &yvertex, const Units::Length &x1, const Units::Length &y1,
      const Units::Length &x2, const Units::Length &y2) {
   // vector 1 is from vertex to x1,y1
   Units::MetersLength dx1 = x1 - xvertex;
   Units::MetersLength dy1 = y1 - yvertex;
   double norm1 = sqrt(dx1.value() * dx1.value() + dy1.value() * dy1.value());

   // vector 2 is from turn center to turn end
   Units::MetersLength dx2 = x2 - xvertex;
   Units::MetersLength dy2 = y2 - yvertex;
   double norm2 = sqrt(dx2.value() * dx2.value() + dy2.value() * dy2.value());

   // theta is acos(dot product)
   double dotp = dx1.value() / norm1 * dx2.value() / norm2 + dy1.value() / norm1 * dy2.value() / norm2;
   if (fabs(dotp) > 1) {
      dotp = CoreUtils::SignOfValue(dotp) * 1.0;
   }
   Units::SignedRadiansAngle theta(acos(dotp));  // acos is on [0,pi]
   return theta;
}

Units::Area AircraftCalculations::ComputeCrossProduct(const Units::Length &xvertex, const Units::Length &yvertex,
                                                      const Units::Length &x1, const Units::Length &y1,
                                                      const Units::Length &x2, const Units::Length &y2) {
   return (yvertex - y1) * x2 + (x1 - xvertex) * y2 + (xvertex * y1 - x1 * yvertex);
}

bool AircraftCalculations::CalculateDistanceAlongPathFromPosition(
      const Units::Length position_x, const Units::Length position_y,
      const std::vector<HorizontalPath> &horizontal_trajectory,
      const std::vector<HorizontalPath>::size_type starting_trajectory_index, Units::Length &distance_along_path,
      Units::Angle &course, std::vector<HorizontalPath>::size_type &resolved_trajectory_index) {

   static const Units::NauticalMilesLength cte_tolerance(2.5);  // legacy tolerance. do not change.
   return CalculateDistanceAlongPathFromPosition(cte_tolerance, position_x, position_y, horizontal_trajectory,
                                                 starting_trajectory_index, distance_along_path, course,
                                                 resolved_trajectory_index);
}

bool AircraftCalculations::CalculateDistanceAlongPathFromPosition(
      const Units::Length cross_track_tolerance, const Units::Length position_x, const Units::Length position_y,
      const std::vector<HorizontalPath> &horizontal_trajectory,
      const std::vector<HorizontalPath>::size_type starting_trajectory_index, Units::Length &distance_along_path,
      Units::Angle &course, std::vector<HorizontalPath>::size_type &resolved_trajectory_index) {
   LOG4CPLUS_TRACE(logger,
                   "Calculating distance from ("
                         << Units::MetersLength(position_x) << "," << Units::MetersLength(position_y)
                         << ") to hpath starting at ("
                         << Units::MetersLength(horizontal_trajectory[starting_trajectory_index].GetXPositionMeters())
                         << ","
                         << Units::MetersLength(horizontal_trajectory[starting_trajectory_index].GetYPositionMeters())
                         << "):" << starting_trajectory_index << "/" << horizontal_trajectory.size());

   // Dummy values
   distance_along_path = Units::NegInfinity();
   course = Units::RadiansAngle(Units::infinity());

   // Compute Euclidean distances for all horizontal trajectory points
   // and order in ascending sequence.
   vector<PathDistance> distances = AircraftCalculations::ComputePathDistances(
         position_x, position_y,
         starting_trajectory_index < 1 ? starting_trajectory_index : starting_trajectory_index - 1,
         horizontal_trajectory);

   // Find smallest distance with an acceptable cross track error.
   int nextTrajIx = -1;
   Units::NauticalMilesLength cte;
   for (auto i = 0; ((i < distances.size()) && (nextTrajIx == -1)); ++i) {

      int computedNextIx;
      AircraftCalculations::CrossTrackError(position_x, position_y, distances[i].m_horizontal_path_index,
                                            horizontal_trajectory, computedNextIx, cte);

      if (cte <= cross_track_tolerance) {
         nextTrajIx = computedNextIx;
      }
   }

   resolved_trajectory_index = static_cast<unsigned long>(nextTrajIx);

   if (nextTrajIx == -1) {
      /*
       * Developers: note that this if logic is no longer a FATAL condition, but we
       * must throw. Some callers will catch and handle the situation because
       * it is sometimes normal. See AAES-382, AAES-633
       */
      char msg[500];
      sprintf(msg,
              "Trajectory point with acceptable cross track error not found %lf nmi\nThis can occur for two "
              "reasons:\n\t1. Position cannot be projected onto a route segment (before beginning or after end),\n\t2. "
              "Position is farther than horizontal tolerance from horizontal trajectory.\nThe second can occur for "
              "clearance type of CAPTURE or MAINTAIN (see Issue AAES-1037)",
              cte.value());
      throw logic_error(msg);
   }

   // Calculate DTG and course of aircraft.
   Units::Length dap;
   Units::RadiansAngle theta;
   if (horizontal_trajectory[nextTrajIx].m_segment_type == HorizontalPath::SegmentType::STRAIGHT) {

      Units::Length d =
            sqrt(Units::sqr(position_x - Units::MetersLength(horizontal_trajectory[nextTrajIx].GetXPositionMeters())) +
                 Units::sqr(position_y - Units::MetersLength(horizontal_trajectory[nextTrajIx].GetYPositionMeters())));

      course = Units::ToUnsigned(Units::RadiansAngle(horizontal_trajectory[nextTrajIx].m_path_course) +
                                 Units::PI_RADIANS_ANGLE);

      if (fabs(Units::MetersLength(d).value()) < 1e-5) {
         dap = Units::MetersLength(0.0);
      } else {
         Units::MetersLength dx =
               Units::MetersLength(horizontal_trajectory[nextTrajIx].GetXPositionMeters()) - position_x;
         Units::MetersLength dy =
               Units::MetersLength(horizontal_trajectory[nextTrajIx].GetYPositionMeters()) - position_y;
         theta = Units::RadiansAngle(atan2(dy.value(), dx.value()));

         Units::SignedAngle deltaTheta = Units::ToSigned(theta - course);

         dap = d * cos(deltaTheta);
      }

   } else if (horizontal_trajectory[nextTrajIx].m_segment_type == HorizontalPath::SegmentType::TURN) {

      Units::MetersLength dx =
            position_x - Units::MetersLength(horizontal_trajectory[nextTrajIx].m_turn_info.x_position_meters);
      Units::MetersLength dy =
            position_y - Units::MetersLength(horizontal_trajectory[nextTrajIx].m_turn_info.y_position_meters);

      // theta is undefined if dx and dy are both zero
      // if within 5 meters of the point, consider at the point
      double lx = (Units::MetersLength(position_x)).value();
      double ly = (Units::MetersLength(position_y)).value();

      if (pow(lx - (horizontal_trajectory[nextTrajIx]).GetXPositionMeters(), 2) +
                pow(ly - (horizontal_trajectory[nextTrajIx]).GetYPositionMeters(), 2) <
          9) {
         distance_along_path = Units::MetersLength(horizontal_trajectory[nextTrajIx].m_path_length_cumulative_meters);
         course = Units::ToUnsigned(Units::RadiansAngle(horizontal_trajectory[nextTrajIx].m_path_course) +
                                    Units::PI_RADIANS_ANGLE);
         return true;
      }

      theta = Units::UnsignedRadiansAngle(atan2(dy.value(), dx.value()));

      Units::RadiansAngle deltaTheta =
            Units::ToSigned(Units::UnsignedRadiansAngle(horizontal_trajectory[nextTrajIx].m_turn_info.q_start) - theta);

      dap = Units::MetersLength(horizontal_trajectory[nextTrajIx].m_turn_info.radius) * fabs(deltaTheta.value());

      if (CoreUtils::SignOfValue(deltaTheta.value()) > 0) {
         course = theta + Units::PI_RADIANS_ANGLE / 2;
      } else {
         course = theta - Units::PI_RADIANS_ANGLE / 2;
      }

      course = Units::ToUnsigned(course);
   } else {
      throw logic_error("Non straight non turn segment_type in CalculateDistanceAlongPathFromPosition");
   }

   distance_along_path = dap + Units::MetersLength(horizontal_trajectory[nextTrajIx].m_path_length_cumulative_meters);
   return true;
}
