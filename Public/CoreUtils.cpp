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

#include "public/CoreUtils.h"

#include <cfloat>
#include <iomanip>
#include <stdexcept>

#include "public/GeolibUtils.h"
#include "public/LatitudeLongitudePoint.h"
#include "public/SimulationTime.h"

using namespace std;

int CoreUtils::FindNearestIndex(const double &value_to_find, const vector<double> &vector_to_search) {
   int idx;
   if (value_to_find > vector_to_search.back()) {
      // Special handling that upper_bound() won't accomplish
      idx = static_cast<int>(vector_to_search.size() - 1);
   } else {
      auto itr = std::upper_bound(vector_to_search.begin(), vector_to_search.end(), value_to_find);
      idx = static_cast<int>(itr - vector_to_search.begin());
   }

   return idx;
}

double CoreUtils::LinearlyInterpolate(int upper_index, double x_interpolation_value,
                                      const std::vector<double> &x_values, const std::vector<double> &y_values) {
   if (upper_index < 1 || upper_index >= x_values.size()) {
      char msg[200];
      snprintf(msg, sizeof(msg), "upper_index (%d) is not between 1 and %d", upper_index,
               static_cast<int>(x_values.size() - 1));
      LOG4CPLUS_FATAL(m_logger, msg);
      throw out_of_range(msg);
   }

   const double v2 = x_values[upper_index];
   const double v1 = x_values[upper_index - 1];
   const double o2 = y_values[upper_index];
   const double o1 = y_values[upper_index - 1];

   if ((x_interpolation_value - v1) * (x_interpolation_value - v2) > 0) {
      char msg[200];
      snprintf(msg, sizeof(msg), "ratio (%lf) is not between %lf and %lf.", x_interpolation_value, v1, v2);

      double ratio = (x_interpolation_value - v1) / (x_interpolation_value - v2);
      if (upper_index + 1 == x_values.size() && (ratio < .1 || ratio > 10)) {
         LOG4CPLUS_WARN(m_logger, msg);
      } else {
         LOG4CPLUS_FATAL(m_logger, msg);
         throw domain_error(msg);
      }
   }

   return ((o2 - o1) / (v2 - v1)) * (x_interpolation_value - v1) + o1;
}

Units::Speed CoreUtils::LinearlyInterpolate(int upper_index, Units::Length x_interpolation_value,
                                            const std::vector<double> &x_values,
                                            const std::vector<Units::Speed> &y_values) {
   std::vector<double> y_values_as_double{};
   auto insert_speed_as_double = [&y_values_as_double](Units::Speed speed_value) {
      y_values_as_double.push_back(Units::MetersPerSecondSpeed(speed_value).value());
   };
   std::for_each(y_values.begin(), y_values.end(), insert_speed_as_double);
   return Units::MetersPerSecondSpeed(LinearlyInterpolate(
         upper_index, Units::MetersLength(x_interpolation_value).value(), x_values, y_values_as_double));
}

const Units::Length CoreUtils::CalculateEuclideanDistance(const std::pair<Units::Length, Units::Length> &xyLoc1,
                                                          const std::pair<Units::Length, Units::Length> &xyLoc2) {
   Units::Length xdiff = xyLoc1.first - xyLoc2.first;
   Units::Length ydiff = xyLoc1.second - xyLoc2.second;
   const Units::Length eucldist = sqrt((xdiff * xdiff) + (ydiff * ydiff));
   return eucldist;
}

const double CoreUtils::LimitOnInterval(double value, double low_limit, double high_limit) {
   return (value < low_limit ? low_limit : (value > high_limit ? high_limit : value));
}

const int CoreUtils::SignOfValue(double value) { return (((value) == (0)) ? 0 : (((value) > (0)) ? (1) : (-1))); }

std::list<Waypoint> CoreUtils::ShortenLongLegs(const std::list<Waypoint> &ordered_waypoints,
                                               Units::Length maximum_allowable_length) {
   using namespace geolib_idealab;
   using namespace aaesim;

   std::list<Waypoint> replacement_waypoints = {};
   Waypoint previous_waypoint = (ordered_waypoints).front();
   for (const Waypoint &next_waypoint : ordered_waypoints) {
      if (previous_waypoint.GetName() != next_waypoint.GetName()) {
         if (next_waypoint.GetRfTurnArcRadius() == Units::zero()) {
            const LatitudeLongitudePoint point1 = LatitudeLongitudePoint::CreateFromWaypoint(previous_waypoint);
            const LatitudeLongitudePoint point2 = LatitudeLongitudePoint::CreateFromWaypoint(next_waypoint);
            const LineOnEllipsoid line_on_ellipsoid = LineOnEllipsoid::CreateFromPoints(point1, point2);

            if (line_on_ellipsoid.GetShapeLength() > maximum_allowable_length) {
               auto intermediate_waypoints =
                     GetIntermediateWaypointsForLongLeg(line_on_ellipsoid, maximum_allowable_length);
               replacement_waypoints.insert(replacement_waypoints.end(), intermediate_waypoints.begin(),
                                            intermediate_waypoints.end());
            }
         }
      }
      replacement_waypoints.push_back(next_waypoint);
      previous_waypoint = next_waypoint;
   }

   return std::list<Waypoint>(replacement_waypoints);
}

std::list<Waypoint> CoreUtils::GetIntermediateWaypointsForLongLeg(const aaesim::LineOnEllipsoid &line_on_ellipsoid,
                                                                  Units::Length maximum_allowable_single_leg_distance) {
   using namespace geolib_idealab;
   using namespace aaesim;

   Units::NauticalMilesLength distance_to_end_point(line_on_ellipsoid.GetShapeLength());

   std::list<Waypoint> intermediate_waypoints;
   int loop_counter = 1;
   static const Units::NauticalMilesLength five_nm(5);
   //  static const Units::NauticalMilesLength two_nm(2);
   while (distance_to_end_point > maximum_allowable_single_leg_distance) {
      LatitudeLongitudePoint new_intermediate_point_on_line = line_on_ellipsoid.CalculatePointAtDistanceFromStartPoint(
            maximum_allowable_single_leg_distance * static_cast<float>(loop_counter));
      Waypoint new_waypoint(INTERMEDIATE_WAYPOINT_ROOT_NAME + std::to_string(loop_counter),
                            new_intermediate_point_on_line.GetLatitude(),
                            new_intermediate_point_on_line.GetLongitude());
      distance_to_end_point = line_on_ellipsoid.GetDistanceToEndPoint(new_intermediate_point_on_line);
      if (distance_to_end_point < five_nm) {
         // if (distance_to_end_point < two_nm) {
         //  too close to the end point. exit loop
         break;
      } else {
         intermediate_waypoints.push_back(new_waypoint);
         loop_counter++;
      }
   }

   return intermediate_waypoints;
}