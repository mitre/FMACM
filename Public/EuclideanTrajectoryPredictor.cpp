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

#include <stdexcept>
#include <iomanip>
#include "public/EuclideanTrajectoryPredictor.h"
#include "public/Wind.h"
#include "public/AircraftCalculations.h"
#include <public/AlongPathDistanceCalculator.h>
#include <scalar/AngularSpeed.h>

using namespace std;
using namespace aaesim::open_source::constants;
using namespace aaesim::open_source;

log4cplus::Logger EuclideanTrajectoryPredictor::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("EuclideanTrajectoryPredictor"));

// returns whether three points are counter-clockwise (positive sign),
// colinear (zero), or clockwise (negative sign)
double EuclideanTrajectoryPredictor::CounterClockwise(const double ax, const double ay, const double bx,
                                                      const double by, const double cx, const double cy) {
   return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

// Returns whether (0,0) -> (ax, ay) -> (bx, by) is counterclockwise, clockwise,
// or colinear.
double EuclideanTrajectoryPredictor::CounterClockwise(const double ax, const double ay, const double bx,
                                                      const double by) {
   return ax * by - bx * ay;
}

// returns true if the trajectory point and waypoint are at the same location
bool EuclideanTrajectoryPredictor::SamePoint(const PrecalcWaypoint &wp, const HorizontalPath &hp) {
   return (wp.m_x_pos_meters.value() == hp.GetXPositionMeters() &&
           wp.m_y_pos_meters.value() == hp.GetYPositionMeters());
}

// given a start point and destination point, finds the point that is distance
// radius from both points on the same side of the line from the start point
// to the destination point as the originally given turn point center.
// Returns the distance from the original turn point center to the calculated
// point.
double EuclideanTrajectoryPredictor::FindCenterPoint(double fromX, double fromY, double toX, double toY, double gcpX,
                                                     double gcpY, double radius, double &cpX, double &cpY) {
   double deltaX = toX - fromX;
   double deltaY = toY - fromY;
   double leg_distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
   double half_leg = leg_distance / 2.0;
   double midX = (toX + fromX) / 2.0;
   double midY = (toY + fromY) / 2.0;
   double p_distance = sqrt(pow(radius, 2) - pow(half_leg, 2));
   double direction = CounterClockwise(fromX, fromY, toX, toY, gcpX, gcpY);
   double deltaCX = deltaY * p_distance / leg_distance;
   double deltaCY = deltaX * p_distance / leg_distance;
   if (direction > 0) {  // ccw
      cpX = midX - deltaCX;
      cpY = midY + deltaCY;
   } else {
      cpX = midX + deltaCX;
      cpY = midY - deltaCY;
   }

   deltaX = cpX - gcpX;
   deltaY = cpY - gcpY;
   return (sqrt(pow(deltaX, 2) + pow(deltaY, 2)));
}

// Returns half of hte arc-distance of a turn in meters
double EuclideanTrajectoryPredictor::HalfTurn(HorizontalTurnPath turn) {
   Units::SignedRadiansAngle course_change =
         AircraftCalculations::ConvertPitoPi(Units::RadiansAngle(turn.q_end - turn.q_start));
   return (Units::MetersLength(turn.radius).value() * fabs(course_change.value()) / 2.0);
}

EuclideanTrajectoryPredictor::EuclideanTrajectoryPredictor(void) {
   m_waypoint_vector.clear();
   m_horizontal_path.clear();
   m_bank_angle = Units::DUMMY_DEGREES_ANGLE;
   m_altitude_at_final_waypoint = Units::FeetLength(-50.0);
   m_distance_calculator = AlongPathDistanceCalculator();
   m_position_calculator = PositionCalculator();
   m_aircraft_distance_to_go = Units::infinity();
}

void EuclideanTrajectoryPredictor::CalculateWaypoints(
      const AircraftIntent &aircraft_intent, const aaesim::open_source::WeatherPrediction &weather_prediction) {
   // Sets waypoints, achieve by point, and altitude at FAF (final) waypoint
   // based on intent.

   m_aircraft_intent = aircraft_intent;

   // Set altitude at FAF to final altitude in feet.
   m_altitude_at_final_waypoint = Units::FeetLength(Units::MetersLength(
         aircraft_intent.GetRouteData().m_nominal_altitude[(aircraft_intent.GetNumberOfWaypoints() - 1)]));

   // set waypoints
   m_waypoint_vector.clear();  // empties the waypoint vector for Aircraft Intent Waypoints

   double prev_dist = 0;

   // begin turn
   double delta_Bx;
   double delta_By;
   // end turn
   double delta_Ex;
   double delta_Ey;
   // straight m_path_course
   double delta_x;
   double delta_y;
   // for calculating new center point of turn
   double turnPtX;
   double turnPtY;

   Units::RadiansAngle bTperp = Units::ZERO_ANGLE;  // Direction vector from center of turn to begin of turn waypoint
   Units::RadiansAngle eTperp;                      // Direction vector from center of turn to end of turn waypoint

   double leg_length;
   Units::RadiansAngle course;
   double ccw;

   PrecalcWaypoint new_waypoint;
   // loop to translate all of the intent waypoints into Precalc Waypoints, works from
   // back to front since precalc starts from the endpoint
   for (int loop = aircraft_intent.GetNumberOfWaypoints() - 1; loop > 0; loop--) {
      delta_x =
            Units::MetersLength(aircraft_intent.GetRouteData().m_x[loop - 1] - aircraft_intent.GetRouteData().m_x[loop])
                  .value();  // from - to, to get m_path_course correct
      delta_y = aircraft_intent.GetRouteData().m_y[loop - 1].value() - aircraft_intent.GetRouteData().m_y[loop].value();

      // Note: radius is entered to the nearest 10th of a mile = 185.2 meters
      if (aircraft_intent.GetRouteData().m_rf_radius[loop].value() > 0.00001) {  // This is an RF leg
         bool radiusTooLarge = false;
         // find distance from this point to center of turn
         delta_Bx = aircraft_intent.GetRouteData().m_x[loop - 1].value() -
                    aircraft_intent.GetRouteData().m_x_rf_center[loop].value();
         delta_By = aircraft_intent.GetRouteData().m_y[loop - 1].value() -
                    aircraft_intent.GetRouteData().m_y_rf_center[loop].value();
         // TODO compare square so we do not have to take square root
         double radBloop = sqrt(pow(delta_Bx, 2) + pow(delta_By, 2));
         // radius is input to nearest 10th of a mile.
         if (fabs(radBloop - aircraft_intent.GetRouteData().m_rf_radius[loop].value()) > 100.0) {  // difference of 100
                                                                                                   // meters
            radiusTooLarge = true;
         }
         delta_Ex = aircraft_intent.GetRouteData().m_x[loop].value() -
                    aircraft_intent.GetRouteData().m_x_rf_center[loop].value();
         delta_Ey = aircraft_intent.GetRouteData().m_y[loop].value() -
                    aircraft_intent.GetRouteData().m_y_rf_center[loop].value();
         // TODO compare square so we do not have to take square root
         double radEloop = sqrt(pow(delta_Ex, 2) + pow(delta_Ey, 2));
         if (fabs(radEloop - aircraft_intent.GetRouteData().m_rf_radius[loop].value()) > 100.0) {  // difference of 100
                                                                                                   // meters
            radiusTooLarge = true;
         }
         if (radiusTooLarge) {
            double miss_dist = FindCenterPoint(
                  Units::MetersLength(aircraft_intent.GetRouteData().m_x[loop - 1]).value(),
                  aircraft_intent.GetRouteData().m_y[loop - 1].value(),
                  aircraft_intent.GetRouteData().m_x[loop].value(), aircraft_intent.GetRouteData().m_y[loop].value(),
                  aircraft_intent.GetRouteData().m_x_rf_center[loop].value(),
                  aircraft_intent.GetRouteData().m_y_rf_center[loop].value(),
                  aircraft_intent.GetRouteData().m_rf_radius[loop].value(), turnPtX, turnPtY);

            LOG4CPLUS_INFO(m_logger, "Calculated turn center point at " << aircraft_intent.GetWaypointName(loop)
                                                                        << " is " << miss_dist
                                                                        << " meters different than input.");

            new_waypoint.m_rf_leg_center_x = Units::MetersLength(turnPtX);
            new_waypoint.m_rf_leg_center_y = Units::MetersLength(turnPtY);
         } else {
            new_waypoint.m_rf_leg_center_x = aircraft_intent.GetRouteData().m_x_rf_center[loop];
            new_waypoint.m_rf_leg_center_y = aircraft_intent.GetRouteData().m_y_rf_center[loop];
         }

         ccw = CounterClockwise(delta_Bx, delta_By, delta_Ex, delta_Ey);

         if (fabs(ccw) < 1.0e-10) {  // Consider to be colinear
            printf("Colinear points loop: %d, x: %f, y:%f, prev_x:%f, prev_y:%f, cp_x:%f, cp_y:%f\n", loop,
                   aircraft_intent.GetRouteData().m_x[loop].value(), aircraft_intent.GetRouteData().m_y[loop].value(),
                   aircraft_intent.GetRouteData().m_x[loop - 1].value(),
                   aircraft_intent.GetRouteData().m_y[loop - 1].value(),
                   aircraft_intent.GetRouteData().m_x_rf_center[loop].value(),
                   aircraft_intent.GetRouteData().m_y_rf_center[loop].value());
            LOG4CPLUS_ERROR(m_logger, "RF Leg Center of Turn is colinear with Waypoints");
         }
         new_waypoint.m_name = aircraft_intent.GetRouteData().m_name[loop];
         new_waypoint.m_x_pos_meters = aircraft_intent.GetRouteData().m_x[loop];
         new_waypoint.m_y_pos_meters = aircraft_intent.GetRouteData().m_y[loop];
         new_waypoint.m_radius_rf_leg = aircraft_intent.GetRouteData().m_rf_radius[loop];

         double directDist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
         double radius = aircraft_intent.GetRouteData().m_rf_radius[loop].value();
         leg_length = 2 * asin(directDist / (2 * radius)) * radius;

         eTperp = Units::RadiansAngle(atan2(delta_Ey, delta_Ex));
         bTperp = Units::RadiansAngle(atan2(delta_By, delta_Bx));

         if (ccw < 0.0) {  // right turn
            course = eTperp + Units::DegreesAngle(90);
         } else {  // left turn
            course = eTperp - Units::DegreesAngle(90);
         }
      } else {  // straight leg
         leg_length = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

         // calculate Psi m_path_course
         course = Units::RadiansAngle(atan2(delta_y, delta_x));

         new_waypoint.m_name = aircraft_intent.GetRouteData().m_name[loop];
         new_waypoint.m_x_pos_meters = aircraft_intent.GetRouteData().m_x[loop];
         new_waypoint.m_y_pos_meters = aircraft_intent.GetRouteData().m_y[loop];
         new_waypoint.m_rf_leg_center_x = aircraft_intent.GetRouteData().m_x_rf_center[loop];
         new_waypoint.m_rf_leg_center_y = aircraft_intent.GetRouteData().m_y_rf_center[loop];
         new_waypoint.m_radius_rf_leg = aircraft_intent.GetRouteData().m_rf_radius[loop];
      }

      prev_dist += leg_length;
      new_waypoint.m_leg_length = Units::MetersLength(leg_length);
      new_waypoint.m_course_angle = course;
      new_waypoint.m_precalc_constraints.constraint_along_path_distance = Units::MetersLength(prev_dist);
      new_waypoint.m_precalc_constraints.constraint_altHi =
            aircraft_intent.GetRouteData().m_high_altitude_constraint[loop - 1];
      new_waypoint.m_precalc_constraints.constraint_altLow =
            aircraft_intent.GetRouteData().m_low_altitude_constraint[loop - 1];
      new_waypoint.m_precalc_constraints.constraint_speedHi =
            aircraft_intent.GetRouteData().m_high_speed_constraint[loop - 1];
      new_waypoint.m_precalc_constraints.constraint_speedLow =
            aircraft_intent.GetRouteData().m_low_speed_constraint[loop - 1];

      m_waypoint_vector.push_back(new_waypoint);
   }

   new_waypoint.m_leg_length = Units::zero();
   new_waypoint.m_name = aircraft_intent.GetRouteData().m_name[0];
   new_waypoint.m_x_pos_meters = aircraft_intent.GetRouteData().m_x[0];
   new_waypoint.m_y_pos_meters = aircraft_intent.GetRouteData().m_y[0];
   new_waypoint.m_precalc_constraints.constraint_along_path_distance = Units::MetersLength(prev_dist);
   new_waypoint.m_precalc_constraints.constraint_altHi = aircraft_intent.GetRouteData().m_high_altitude_constraint[0];
   new_waypoint.m_precalc_constraints.constraint_altLow = aircraft_intent.GetRouteData().m_low_altitude_constraint[0];
   new_waypoint.m_precalc_constraints.constraint_speedHi = aircraft_intent.GetRouteData().m_high_speed_constraint[0];
   new_waypoint.m_precalc_constraints.constraint_speedLow = aircraft_intent.GetRouteData().m_low_speed_constraint[0];
   if (aircraft_intent.GetRouteData().m_rf_radius[0].value() < 0.000001) {  // straight leg
      new_waypoint.m_radius_rf_leg = Units::MetersLength(0);
      new_waypoint.m_course_angle = m_waypoint_vector.back().m_course_angle;
      new_waypoint.m_rf_leg_center_x = Units::MetersLength(0);
      new_waypoint.m_rf_leg_center_y = Units::MetersLength(0);
   } else {
      new_waypoint.m_rf_leg_center_x = m_waypoint_vector.back().m_rf_leg_center_x;
      new_waypoint.m_rf_leg_center_y = m_waypoint_vector.back().m_rf_leg_center_x;
      ccw = CounterClockwise(new_waypoint.m_x_pos_meters.value(), new_waypoint.m_y_pos_meters.value(),
                             m_waypoint_vector.back().m_x_pos_meters.value(),
                             m_waypoint_vector.back().m_y_pos_meters.value(), new_waypoint.m_rf_leg_center_x.value(),
                             new_waypoint.m_rf_leg_center_y.value());
      new_waypoint.m_radius_rf_leg = aircraft_intent.GetRouteData().m_rf_radius[0];

      if (ccw < 0.0) {  // right turn
         new_waypoint.m_course_angle = bTperp + Units::DegreesAngle(90);
      } else {  // left turn (ignore colinear)
         new_waypoint.m_course_angle = bTperp - Units::DegreesAngle(90);
      }
   }
   m_waypoint_vector.push_back(new_waypoint);

   Units::KnotsSpeed start_speed = aircraft_intent.GetRouteData().m_nominal_ias[0];
   AdjustConstraints(start_speed);
}

// Calculates the trajectory for a turn when the leg length is less than the required turn distance.
// courseChange1 is first turn (change in m_path_course at the first waypoint).
// courseChange2 is second turn (change in m_path_course at the second waypoint).
// legLength is the distance between the first waypoint and the second waypoint.
// Calculates the inscribed turn that is tangent to the m_path_course into the first waypoint,
// tangent to the line between waypoint 1 and 2, and tangent to the m_path_course out of the
// second waypoint.
// output parameters:
//   radius - the radius of the inscribed circle
//   turnDist - the turn anticipation distance for the first turn
void EuclideanTrajectoryPredictor::CalculateTrajUsingLawOfSines(const double courseChange1, const double courseChange2,
                                                                const double legLength, double &radius,
                                                                double &turnDist) {
   Units::Angle A, B, AOB, AOC;
   double AO;  // side distances
   double AC;  // turn anticipation distances

   A = AircraftCalculations::Convert0to2Pi(Units::DegreesAngle(180) - Units::RadiansAngle(fabs(courseChange1)));
   B = AircraftCalculations::Convert0to2Pi(Units::DegreesAngle(180) - Units::RadiansAngle(fabs(courseChange2)));
   AOB = AircraftCalculations::Convert0to2Pi(Units::DegreesAngle(180) - A / 2 - B / 2);
   AO = legLength * sin(B / 2) / sin(AOB);
   AOC = AircraftCalculations::Convert0to2Pi(Units::DegreesAngle(90) - A / 2);
   radius = AO * sin(A / 2);
   AC = AO * sin(AOC);
   turnDist = AC;
}

void EuclideanTrajectoryPredictor::CalculateTrajUsingKite(const double courseChange1, const double courseChange2,
                                                          const double legLength, double &radius, double &turnDist) {
   // Calculates the trajectory for a turn when the leg length is less than the required turn distance.
   // courseChange1 is first turn (change in m_path_course at the firat waypoint) in degrees.
   // courseChange2 is second turn (change in m_path_course at the second waypoint)in degrees.
   // legLength is the distance between the first waypoint and the second waypoint.
   // Calculates the inscribed turn that is tangent to the m_path_course into the firat waypoint,
   // tangent to the line between waypoint 1 and 2, and tangent to the m_path_course out of the
   // second waypoint.

   // varaibles for Kite algorithm
   double c, d;  // tangent segments for Kite algorithm

   /*
   //           A
   //          / \
   //         /   \
   //        B  O  C
   //         \ E /
   //          \ /F
   //           D
   // E is center of inscribed circle
   // F is tangent point of inscribed circle and segment_type CD
    */

   Units::Angle A, C, D;               // Kite angles
   double AC, OC, CD;                  // Kite side distances
   Units::Angle ECD, ACE, ACO, OCE;    // Angles for inscribed circle points
   double BC, OE, OA, AE, AD, ED, EF;  // distances for inscribed circle calculations
   // radius;  // equal to EF

   A = AircraftCalculations::Convert0to2Pi(Units::DegreesAngle(180) - Units::RadiansAngle(fabs(courseChange1)));
   C = AircraftCalculations::Convert0to2Pi(Units::DegreesAngle(180) - Units::RadiansAngle(fabs(courseChange2)));
   // B = C;  // unused
   D = Units::DegreesAngle(360) - (A + C + C);
   // test for D positive
   if (Units::DegreesAngle(D).value() < 0.0)  // turns do not form a kite
   {
      LOG4CPLUS_INFO(m_logger, "Kite called for tight turn that cannot form a kite, Using half leg-length instead.");
      radius = -1;
      turnDist = 0;
      return;
   }

   // find kite side distances in meters
   AC = legLength;
   // AB = AC; // unused
   OC = AC * sin(A / 2);
   CD = OC / sin(D / 2);
   // BD = CD;  // unused

   // find radius of inscribed circle
   ECD = C / 2;
   ACE = ECD;
   ACO = Units::DegreesAngle(90) - A / 2;
   OCE = ACE - ACO;
   BC = OC * 2;
   OE = BC * tan(OCE) / 2;
   OA = AC * cos(A / 2);
   AE = OA + OE;
   AD = AC * cos(A / 2) + CD * cos(D / 2);
   ED = AD - AE;
   EF = ED * sin(D / 2);
   radius = EF;

   // find tangent segments
   // segment_type a is the turn anticipation at angle A.
   // segment_type b = c is the turn anticipation at angle C
   // segment_type d is the turn anticipation at Angle D, which does not exist as a way point
   d = ED * cos(D / 2);
   c = CD - d;
   // b= c;
   // a = AC - c;
   turnDist = AC - c;
}

vector<aaesim::open_source::TurnAnticipation> EuclideanTrajectoryPredictor::CalculateTurnAnticipation(
      const HorizontalTrajOption option) {
   double turn_radius;
   aaesim::open_source::TurnAnticipation thisTurnAnticipation;
   vector<aaesim::open_source::TurnAnticipation> turnAnticipation;
   // waypoints are indexed backwards: first waypoint is route destination
   // modified for RF legs

   aaesim::open_source::TurnAnticipation straightTurnAnticipation = aaesim::open_source::TurnAnticipation();
   // first point has zero turn anticipation
   turnAnticipation.push_back(straightTurnAnticipation);
   // loop through all but last waypoint (starting point for route)
   for (unsigned int loop = 1; loop < m_waypoint_vector.size() - 1; loop++) {
      // if this is an RF leg, then no turn anticipation
      double alt_at_turn = 0;
      double gspeed_at_turn = 0;  // should be meters per second

      // loop to sum leg lengths used for both altitude approximations
      double leg_sum = 0;
      for (unsigned int curr_leg = 0; curr_leg < loop; curr_leg++) {
         leg_sum += Units::MetersLength(m_waypoint_vector[curr_leg].m_leg_length).value();
      }

      if (m_waypoint_vector[loop].m_radius_rf_leg.value() > 0.000001) {
         turnAnticipation.push_back(straightTurnAnticipation);
         // If an RF Leg, need to calculate bank angle and groundspeed
         // Consider moving this loop to a separate method
         if (option == SECOND_PASS) {
            // loop to find the location in the Precalculated Descent to get Ground Speed
            unsigned int curr_index = 0;
            bool found = false;  // index of distance found flag
            for (unsigned int next_index = 0;
                 next_index < m_vertical_predictor->GetVerticalPath().along_path_distance_m.size() && !found;
                 next_index++) {
               if (fabs(m_vertical_predictor->GetVerticalPath().along_path_distance_m[next_index]) > leg_sum) {
                  found = true;
               }

               curr_index = next_index;
            }

            if (curr_index != 0 && found) {
               alt_at_turn = m_vertical_predictor->GetVerticalPath().altitude_m[curr_index];
               gspeed_at_turn = m_vertical_predictor->GetVerticalPath().gs_mps[curr_index];
            } else {
               const string msg = "Unable to find position in vertical prediction";
               LOG4CPLUS_FATAL(m_logger, msg);
               throw runtime_error(msg);
            }

            m_waypoint_vector[loop].m_ground_speed = Units::MetersPerSecondSpeed(gspeed_at_turn);
            m_waypoint_vector[loop].m_bank_angle = Units::UnsignedRadiansAngle(
                  atan(gspeed_at_turn * gspeed_at_turn /
                       (GRAVITY_METERS_PER_SECOND * m_waypoint_vector[loop].m_radius_rf_leg.value())));
         }
         continue;
      }
      // if the next leg (previously calculated leg) is an RF leg, no anticipation
      if (m_waypoint_vector[loop - 1].m_radius_rf_leg.value() > 0.0000001) {
         turnAnticipation.push_back(straightTurnAnticipation);
         continue;
      }
      // calculate change in waypoint m_path_course
      double course_change = AircraftCalculations::ConvertPitoPi(m_waypoint_vector[loop].m_course_angle -
                                                                 m_waypoint_vector[loop - 1].m_course_angle)
                                   .value();
      if (course_change == 0.0)  // straight segment
      {
         turnAnticipation.push_back(straightTurnAnticipation);
         continue;
      }

      // check options
      // if option 1 calculate approximate altitude at turn using 3-deg fpa
      if (option == FIRST_PASS) {
         double tas_at_turn;

         alt_at_turn = leg_sum * tan(3 * PI / 180) +
                       Units::MetersLength(m_vertical_predictor->GetAltitudeAtEndOfRoute()).value();

         if (alt_at_turn > Units::MetersLength(m_vertical_predictor->GetCruiseAltitude()).value()) {
            alt_at_turn = Units::MetersLength(m_vertical_predictor->GetCruiseAltitude()).value();
         }

         // check if altitude is more or less than the Transition Altitude to calculate groundspeed at turn
         if (alt_at_turn < Units::MetersLength(m_vertical_predictor->GetTransitionAltitude()).value()) {
            // LAW: use constraint speed, if available

            tas_at_turn = Units::MetersPerSecondSpeed(m_atmosphere->CAS2TAS(m_vertical_predictor->GetTransitionIas(),
                                                                            Units::MetersLength(alt_at_turn)))
                                .value();

            gspeed_at_turn = tas_at_turn;
            // LAW: use Predicted Wind Vector to estimate ground speed in turn
         } else {
            Units::KelvinTemperature t =
                  m_atmosphere->GetTemperature(Units::MetersLength(alt_at_turn));  // gets temperature
            tas_at_turn = m_vertical_predictor->GetTransitionMach() *
                          sqrt(GAMMA * R.value() * t.value());  // calculate true airspeed
            gspeed_at_turn = tas_at_turn;
            // LAW: use Predicted Wind Vector to estimate ground speed in turn
         }
      }  // END if FIRST_PASS
         // else if option 2 calculate approximate altitude at turn from 1st pass Vertical Trajectory
      else if (option == SECOND_PASS) {
         // loop to find the location in the Precalculated Descent to get Ground Speed
         unsigned int curr_index = 0;
         bool found = false;  // index of distance found flag
         for (unsigned int next_index = 0;
              next_index < m_vertical_predictor->GetVerticalPath().along_path_distance_m.size() && !found;
              next_index++) {
            if (fabs(m_vertical_predictor->GetVerticalPath().along_path_distance_m[next_index]) > leg_sum) {
               found = true;
            }
            curr_index = next_index;
         }

         if (curr_index != 0 && found) {
            alt_at_turn = m_vertical_predictor->GetVerticalPath().altitude_m[curr_index];
            gspeed_at_turn = m_vertical_predictor->GetVerticalPath().gs_mps[curr_index];
         } else {
            VerticalPath newTrajectory = m_vertical_predictor->GetVerticalPath();
            ofstream gout("newVerticalTrajectory.csv");
            gout << "Iteration,AC_ID,TimeToGo(sec),DistanceToGo(meters),Altitude(feet),IAS_Speed(knots),Altitude_"
                    "Change(fpm),Velocity_Change(mps2),Theta(deg),TAS_GroundSpeed(knots),Mass"
                 << endl;
            gout << fixed;
            for (int i = 0; i < newTrajectory.along_path_distance_m.size(); i++) {
               gout << "0,0," << setprecision(2) << newTrajectory.time_to_go_sec[i] << "," << setprecision(6)
                    << Units::MetersLength(newTrajectory.along_path_distance_m[i]).value() << ","
                    << Units::FeetLength(Units::MetersLength(newTrajectory.altitude_m[i])).value() << ","
                    << Units::KnotsSpeed(Units::MetersPerSecondSpeed(newTrajectory.cas_mps[i])).value() << ","
                    << Units::FeetPerMinuteSpeed(Units::MetersPerSecondSpeed(newTrajectory.altitude_rate_mps[i]))
                             .value()
                    << "," << Units::KnotsSpeed(newTrajectory.true_airspeed[i]).value() << ","
                    << Units::MetersSecondAcceleration(newTrajectory.tas_rate_mps[i]).value() << ","
                    << Units::DegreesAngle(Units::RadiansAngle(newTrajectory.theta_radians[i])).value() << ","
                    << Units::KnotsSpeed(Units::MetersPerSecondSpeed(newTrajectory.gs_mps[i])).value() << ","
                    << newTrajectory.mass_kg[i] << endl;
            }
            gout.close();

            const string msg = "Unable to find position in vertical prediction";
            LOG4CPLUS_FATAL(m_logger, msg);
            throw runtime_error(msg);
         }
      }  // END of SECOND_PASS

      // Calculate Turn Radius
      double bankangle = 0.5 * fabs(course_change);
      thisTurnAnticipation.groundspeed = gspeed_at_turn;

      if (alt_at_turn < 19500.0 * FEET_TO_METERS) {
         thisTurnAnticipation.maxAngle = Units::RadiansAngle(m_bank_angle).value();
         if (bankangle > Units::RadiansAngle(m_bank_angle).value()) {
            bankangle = Units::RadiansAngle(m_bank_angle).value();
         } else if (bankangle < 10.0 / DEGREES_PER_RADIAN) {
            bankangle = 10.0 / DEGREES_PER_RADIAN;
         }
      } else {
         thisTurnAnticipation.maxAngle = 15.0 / DEGREES_PER_RADIAN;
         if (bankangle > 15.0 / DEGREES_PER_RADIAN) {
            bankangle = 15.0 / DEGREES_PER_RADIAN;
         } else if (bankangle < 5.0 / DEGREES_PER_RADIAN) {
            bankangle = 5.0 / DEGREES_PER_RADIAN;
         }
      }
      thisTurnAnticipation.bankAngle = bankangle;

      m_waypoint_vector[loop].m_bank_angle = Units::UnsignedRadiansAngle(bankangle);
      m_waypoint_vector[loop].m_ground_speed = Units::MetersPerSecondSpeed(gspeed_at_turn);

      turn_radius = (pow(gspeed_at_turn, 2)) / (GRAVITY_METERS_PER_SECOND * tan(bankangle));
      thisTurnAnticipation.radius = turn_radius;

      double halfCourseChange = fabs(course_change) / 2.0;

      thisTurnAnticipation.distance = turn_radius * tan(halfCourseChange);

      if (thisTurnAnticipation.distance <= (0.2 * NAUTICAL_MILES_TO_METERS))  // 1200 feet
      {
         turnAnticipation.push_back(straightTurnAnticipation);  // handle as straight
         continue;
      }

      /*
      // allow for now.  calculate_Horizontal_Trajectory will check leg lengths
      // turn anticipation can never be more than the leg length
      if (thisTurnAnticipation > waypoint_vector[loop-1].leg_length)
          thisTurnAnticipation = waypoint_vector[loop-1].leg_length;
      if (thisTurnAnticipation > waypoint_vector[loop].leg_length)
          thisTurnAnticipation = waypoint_vector[loop].leg_length;
      */

      turnAnticipation.push_back(thisTurnAnticipation);

   }  // end for (loop through waypoints to calculate turnAnticipation

   // add turnAnticipation for route start point
   turnAnticipation.push_back(straightTurnAnticipation);

   if (turnAnticipation.size() != m_waypoint_vector.size()) {
      LOG4CPLUS_ERROR(m_logger, "Turn Anticipation vector not the same size as waypoint_vector");
   }

   return turnAnticipation;
}

void EuclideanTrajectoryPredictor::CalculateHorizontalTrajectory(const HorizontalTrajOption option) {
   vector<HorizontalPath> results;
   HorizontalPath temp;
   double radius;
   double turnDist;
   const Units::DegreesAngle epsilonAngle(5.0);

   if (m_waypoint_vector.size() == 0) {
      m_horizontal_path = results;
      return;
   }

   // do initialization of first 2 elements
   results.push_back(temp);
   results.push_back(temp);

   results[0].SetXYPositionMeters(m_waypoint_vector[0].m_x_pos_meters.value(),
                                  m_waypoint_vector[0].m_y_pos_meters.value());

   if (m_waypoint_vector.size() == 1) {
      results[1].SetXYPositionMeters(m_waypoint_vector[0].m_x_pos_meters.value(),
                                     m_waypoint_vector[0].m_y_pos_meters.value());
      results[0].m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
      results[0].m_path_course = m_waypoint_vector[0].m_course_angle.value();
      results[0].m_path_length_cumulative_meters = 0.0;
      results[1].m_path_length_cumulative_meters = 0.0;
      results[1].m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
      results[1].m_path_course = m_waypoint_vector[0].m_course_angle.value();
      m_horizontal_path = results;
      return;
   }

   // Calculate turn anticipation for each turn point
   vector<aaesim::open_source::TurnAnticipation> turnAnticipation = CalculateTurnAnticipation(option);

   // loop to create horizontal trajectory points

   // Use the turn anticipation information to generate the horizontal path
   int makeconstturn = 1;  // flag for which part of turn is being computed
   // 1 - normal turn or first part of tight turn,
   // 2 - second part of tight turn
   int counter = 1;  // index of last element in results

   // Next turn anticipation must be less than remaining leg length
   double remainingLegDistance = Units::MetersLength(m_waypoint_vector[0].m_leg_length).value();

   double course_change;
   double turn_radius;
   for (unsigned int loop = 1; loop < m_waypoint_vector.size(); loop++) {
      if (turnAnticipation[loop].distance == 0)  // straight segment_type, or this is RF leg, or previous was RF leg
      {
         // results[counter].x = waypoint_vector[loop-1].x_pos + waypoint_vector[loop-1].leg_length *
         // cos(waypoint_vector[loop-1].course_angle); results[counter].y = waypoint_vector[loop-1].y_pos +
         // waypoint_vector[loop-1].leg_length * sin(waypoint_vector[loop-1].course_angle);
         results[counter].SetXYPositionMeters(m_waypoint_vector[loop].m_x_pos_meters.value(),
                                              m_waypoint_vector[loop].m_y_pos_meters.value());
         if (m_waypoint_vector[loop - 1].m_radius_rf_leg.value() < 0.000001) {
            results[counter - 1].m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
         } else {  // RFLeg
            results[counter - 1].m_segment_type = HorizontalPath::SegmentType::TURN;
            results[counter - 1].m_turn_info.x_position_meters = m_waypoint_vector[loop - 1].m_rf_leg_center_x.value();
            results[counter - 1].m_turn_info.y_position_meters = m_waypoint_vector[loop - 1].m_rf_leg_center_y.value();
            results[counter - 1].m_turn_info.radius = Units::MetersLength(m_waypoint_vector[loop - 1].m_radius_rf_leg);
            results[counter - 1].m_turn_info.groundspeed = m_waypoint_vector[loop - 1].m_ground_speed;
            results[counter - 1].m_turn_info.bankAngle = m_waypoint_vector[loop - 1].m_bank_angle;
            results[counter - 1].m_turn_info.turn_type = HorizontalTurnPath::TURN_TYPE::RADIUS_FIXED;

            results[counter - 1].m_turn_info.q_start = Units::UnsignedRadiansAngle(atan2(
                  (results[counter - 1].GetYPositionMeters() - results[counter - 1].m_turn_info.y_position_meters),
                  (results[counter - 1].GetXPositionMeters() - results[counter - 1].m_turn_info.x_position_meters)));
            results[counter - 1].m_turn_info.q_end = Units::UnsignedRadiansAngle(
                  atan2((results[counter].GetYPositionMeters() - results[counter - 1].m_turn_info.y_position_meters),
                        (results[counter].GetXPositionMeters() - results[counter - 1].m_turn_info.x_position_meters)));
         }
         if (m_waypoint_vector[loop].m_radius_rf_leg.value() < 0.000001) {
            remainingLegDistance = Units::MetersLength(m_waypoint_vector[loop].m_leg_length).value();
         } else {
            remainingLegDistance = 0.0;
         }

         results[counter - 1].m_path_course = m_waypoint_vector[loop - 1].m_course_angle.value();
      } else {  // turn segment
         course_change = AircraftCalculations::ConvertPitoPi(m_waypoint_vector[loop].m_course_angle -
                                                             m_waypoint_vector[loop - 1].m_course_angle)
                               .value();

         if (makeconstturn == 2)  // continue previous turn (KITE or LawOfSines algorithm)
                                  // radius and turnDist are set in previous loop
         {
            // first check to see if next turn will fit.  If not, then we need to do a KITE or Law of Sines turn from
            // here
            if (turnAnticipation[loop].distance + turnAnticipation[loop + 1].distance >
                Units::MetersLength(m_waypoint_vector[loop].m_leg_length).value()) {
               // if next leg is straight, then KITE or Law of Sines is not tight enough
               if (turnAnticipation[loop + 1].distance == 0) {
                  turnAnticipation[loop].distance = Units::MetersLength(m_waypoint_vector[loop].m_leg_length).value();
                  turn_radius = turnAnticipation[loop].distance / tan(fabs(course_change) / 2.0);
                  makeconstturn = 1;
               } else {
                  // law of sines
                  double next_course_change =
                        AircraftCalculations::ConvertPitoPi(m_waypoint_vector[loop + 1].m_course_angle -
                                                            m_waypoint_vector[loop].m_course_angle)
                              .value();

                  // CalculateTrajUsingKite(course_change, next_course_change, waypoint_vector[loop].leg_length, radius,
                  // turnDist);
                  CalculateTrajUsingLawOfSines(course_change, next_course_change,
                                               Units::MetersLength(m_waypoint_vector[loop].m_leg_length).value(),
                                               radius, turnDist);

                  if (radius < 0)  // should not happen with Law-of-Sines
                  {
                     turnAnticipation[loop].distance =
                           Units::MetersLength(m_waypoint_vector[loop].m_leg_length / 2.0).value();
                     if (turnAnticipation[loop + 1].distance > turnAnticipation[loop].distance) {
                        turnAnticipation[loop + 1].distance = turnAnticipation[loop].distance;
                     }
                     turn_radius = turnAnticipation[loop].distance / tan(fabs(course_change) / 2.0);
                  } else {
                     turn_radius = radius;
                     if (turnDist > remainingLegDistance) {  // need tighter turn than Law Of Sines
                        turnAnticipation[loop].distance = remainingLegDistance;
                        turn_radius = turnAnticipation[loop].distance / tan(fabs(course_change) / 2.0);
                        makeconstturn = 1;
                     } else {
                        // probably need a straight segment_type here; turn anticipation for Law Of Sines < remaining
                        // distance
                        turn_radius = radius;
                        turnAnticipation[loop].distance = turnDist;

                        // for second part of turn
                        turnDist = Units::MetersLength(m_waypoint_vector[loop].m_leg_length).value() - turnDist;
                        remainingLegDistance = turnDist;
                        makeconstturn = 2;
                     }
                  }
               }

            } else {
               turn_radius = radius;
               turnAnticipation[loop].distance = turnDist;
               makeconstturn = 3;
            }
         }
         // makeconstturn == 1
         else if (turnAnticipation[loop].distance + turnAnticipation[loop + 1].distance >
                  Units::MetersLength(m_waypoint_vector[loop].m_leg_length).value()) {
            if (turnAnticipation[loop + 1].distance == 0) {
               turnAnticipation[loop].distance = Units::MetersLength(m_waypoint_vector[loop].m_leg_length).value();
               turn_radius = turnAnticipation[loop].distance / tan(fabs(course_change) / 2.0);
               turnAnticipation[loop].radius = turn_radius;
               turnAnticipation[loop].bankAngle =
                     atan(pow(turnAnticipation[loop].groundspeed, 2) / (GRAVITY_METERS_PER_SECOND * turn_radius));
               // To Do, reset bank angle
            } else {
               double next_course_change =
                     AircraftCalculations::ConvertPitoPi(m_waypoint_vector[loop + 1].m_course_angle -
                                                         m_waypoint_vector[loop].m_course_angle)
                           .value();

               // CalculateTrajUsingKite(course_change, next_course_change, waypoint_vector[loop].leg_length, radius,
               // turnDist);
               CalculateTrajUsingLawOfSines(course_change, next_course_change,
                                            Units::MetersLength(m_waypoint_vector[loop].m_leg_length).value(), radius,
                                            turnDist);

               if (radius < 0)  // should not happen with Law-of-Sines
               {
                  turnAnticipation[loop].distance =
                        Units::MetersLength(m_waypoint_vector[loop].m_leg_length / 2.0).value();
                  if (turnAnticipation[loop + 1].distance > turnAnticipation[loop].distance) {
                     turnAnticipation[loop + 1].distance = turnAnticipation[loop].distance;
                  }
                  turn_radius = turnAnticipation[loop].distance / tan(fabs(course_change) / 2.0);
               } else {
                  if (turnDist > remainingLegDistance) {
                     turnAnticipation[loop].distance = remainingLegDistance;
                     turnDist = remainingLegDistance;
                     turn_radius = turnAnticipation[loop].distance / tan(fabs(course_change) / 2.0);
                     turnAnticipation[loop].radius = turn_radius;
                     turnAnticipation[loop].bankAngle =
                           atan(pow(turnAnticipation[loop].groundspeed, 2) / (GRAVITY_METERS_PER_SECOND * turn_radius));
                  } else {
                     // probably need a straight segment_type here
                     turn_radius = radius;
                     turnAnticipation[loop].radius = turn_radius;
                     turnAnticipation[loop].distance = turnDist;
                     turnAnticipation[loop].bankAngle =
                           atan(pow(turnAnticipation[loop].groundspeed, 2) / (GRAVITY_METERS_PER_SECOND * turn_radius));

                     // for second part of turn
                     turnDist = Units::MetersLength(m_waypoint_vector[loop].m_leg_length).value() - turnDist;
                     remainingLegDistance = turnDist;
                     makeconstturn = 2;
                  }
               }
            }
         } else  // normal turn
         {
            if (turnAnticipation[loop].distance > remainingLegDistance)  // not enough room to make planned turn
            {
               turnAnticipation[loop].distance = remainingLegDistance;
            }
            turn_radius = turnAnticipation[loop].distance / tan(fabs(course_change) / 2.0);
         }

         // Calculate X, Y positions for start and end of turn
         double startTurnx = m_waypoint_vector[loop].m_x_pos_meters.value() -
                             turnAnticipation[loop].distance * cos(m_waypoint_vector[loop - 1].m_course_angle);
         double startTurny = m_waypoint_vector[loop].m_y_pos_meters.value() -
                             turnAnticipation[loop].distance * sin(m_waypoint_vector[loop - 1].m_course_angle);
         double stopTurnx = m_waypoint_vector[loop].m_x_pos_meters.value() +
                            turnAnticipation[loop].distance * cos(m_waypoint_vector[loop].m_course_angle);
         double stopTurny = m_waypoint_vector[loop].m_y_pos_meters.value() +
                            turnAnticipation[loop].distance * sin(m_waypoint_vector[loop].m_course_angle);

         // std::ostringstream strs;
         // strs << "calculateHorizontalTrajectory turn segment_type:\n  start_turn GetXPositionMeters(): " <<
         // startTurnx << "  GetYPositionMeters(): " << startTurny
         //         << "\n  stop_turn  GetXPositionMeters(): " << stopTurnx << "  GetYPositionMeters(): " << stopTurny;
         // std::string str = strs.str();
         // LOG4CPLUS_DEBUG(logger, str);
         // strs.str(std::string());
         // Check if last trajectory point is within 5 meters of start of this turn
         if (pow(results[counter - 1].GetXPositionMeters() - startTurnx, 2) +
                   pow(results[counter - 1].GetYPositionMeters() - startTurny, 2) <
             25) {  // do not add a straight segment_type and move previous trajectory point to start turn point
            counter = counter - 1;
         } else  // insert a straight leg
         {
            results[counter - 1].m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
            results[counter].SetXYPositionMeters(startTurnx, startTurny);
            results[counter - 1].m_path_course = m_waypoint_vector[loop - 1].m_course_angle.value();
            results.push_back(temp);
            // printf("results[%d].GetXPositionMeters(): %f  .GetYPositionMeters(): %f  straight\n", counter,
            // results[counter].GetXPositionMeters(), results[counter].GetYPositionMeters());
         }

         // process end of turn
         counter++;
         results[counter - 1].m_segment_type = HorizontalPath::SegmentType::TURN;
         results[counter - 1].m_turn_info.turn_type = HorizontalTurnPath::TURN_TYPE::PERFORMANCE;
         results[counter].SetXYPositionMeters(stopTurnx, stopTurny);
         results[counter - 1].m_path_course = (m_waypoint_vector[loop - 1].m_course_angle).value();
         results[counter - 1].m_turn_info.radius = Units::MetersLength(turn_radius);

         double gs = m_waypoint_vector[loop].m_ground_speed.value();
         results[counter - 1].m_turn_info.groundspeed = m_waypoint_vector[loop].m_ground_speed;
         results[counter - 1].m_turn_info.bankAngle =
               Units::UnsignedRadiansAngle(atan(gs * gs / (GRAVITY_METERS_PER_SECOND * turn_radius)));

         // Check for bank angle in excess of max bank angle (option 2 only)
         if (option == SECOND_PASS) {
            if (results[counter - 1].m_turn_info.bankAngle > m_bank_angle + epsilonAngle / 2) {
               // Getting close to maximum epsilon. Start warning user.
               ostringstream msg_strm;
               msg_strm << "Turn at waypoint " << loop << " uses bank angle "
                        << Units::DegreesAngle(results[counter - 1].m_turn_info.bankAngle)
                        << " which exceeds maximum allowable bank: " << Units::DegreesAngle(m_bank_angle);
               string msg = msg_strm.str();
               LOG4CPLUS_WARN(m_logger, msg);
            } else if (results[counter - 1].m_turn_info.bankAngle > (m_bank_angle + epsilonAngle)) {
               // beyond epsilon. Throw.
               ostringstream msg_strm;
               msg_strm << "Turn at waypoint " << loop << " uses bank angle "
                        << Units::DegreesAngle(results[counter - 1].m_turn_info.bankAngle)
                        << " which exceeds our internal tolerance of allowable bank error: "
                        << Units::DegreesAngle(m_bank_angle + epsilonAngle)
                        << ". This looks like an unflyable waypoint sequence.";
               string msg = msg_strm.str();
               LOG4CPLUS_FATAL(m_logger, msg);
               throw logic_error(msg);
            }
         }
         if (course_change < 0)  // left turn
         {
            results[counter - 1].m_turn_info.q_start = AircraftCalculations::Convert0to2Pi(
                  m_waypoint_vector[loop - 1].m_course_angle + Units::PI_RADIANS_ANGLE / 2.0);
            results[counter - 1].m_turn_info.q_end = AircraftCalculations::Convert0to2Pi(
                  m_waypoint_vector[loop].m_course_angle + Units::PI_RADIANS_ANGLE / 2.0);
            results[counter - 1].m_turn_info.x_position_meters =
                  results[counter - 1].GetXPositionMeters() +
                  turn_radius * cos(m_waypoint_vector[loop - 1].m_course_angle - (Units::PI_RADIANS_ANGLE / 2.0));
            results[counter - 1].m_turn_info.y_position_meters =
                  results[counter - 1].GetYPositionMeters() +
                  turn_radius * sin(m_waypoint_vector[loop - 1].m_course_angle - (Units::PI_RADIANS_ANGLE / 2.0));
            results[counter].SetXYPositionMeters(
                  results[counter - 1].m_turn_info.x_position_meters +
                        turn_radius * cos(m_waypoint_vector[loop].m_course_angle + (Units::PI_RADIANS_ANGLE / 2.0)),
                  results[counter - 1].m_turn_info.y_position_meters +
                        turn_radius * sin(m_waypoint_vector[loop].m_course_angle + (Units::PI_RADIANS_ANGLE / 2.0)));
         } else  // right turn
         {
            results[counter - 1].m_turn_info.q_start = AircraftCalculations::Convert0to2Pi(
                  m_waypoint_vector[loop - 1].m_course_angle - Units::PI_RADIANS_ANGLE / 2.0);
            results[counter - 1].m_turn_info.q_end = AircraftCalculations::Convert0to2Pi(
                  m_waypoint_vector[loop].m_course_angle - Units::PI_RADIANS_ANGLE / 2.0);
            results[counter - 1].m_turn_info.x_position_meters =
                  results[counter - 1].GetXPositionMeters() +
                  turn_radius * cos(m_waypoint_vector[loop - 1].m_course_angle + (Units::PI_RADIANS_ANGLE / 2.0));
            results[counter - 1].m_turn_info.y_position_meters =
                  results[counter - 1].GetYPositionMeters() +
                  turn_radius * sin(m_waypoint_vector[loop - 1].m_course_angle + (Units::PI_RADIANS_ANGLE / 2.0));
            results[counter].SetXYPositionMeters(
                  results[counter - 1].m_turn_info.x_position_meters +
                        turn_radius * cos(m_waypoint_vector[loop].m_course_angle - (Units::PI_RADIANS_ANGLE / 2.0)),
                  results[counter - 1].m_turn_info.y_position_meters +
                        turn_radius * sin(m_waypoint_vector[loop].m_course_angle - (Units::PI_RADIANS_ANGLE / 2.0)));
         }

         // prepare for next segment_type
         remainingLegDistance =
               sqrt(pow(results[counter].GetXPositionMeters() - m_waypoint_vector[loop + 1].m_x_pos_meters.value(), 2) +
                    pow(results[counter].GetYPositionMeters() - m_waypoint_vector[loop + 1].m_y_pos_meters.value(), 2));
      }  // END turn segment_type

      results.push_back(temp);
      counter++;

   }  // END for waypoint_vector[loop] for horizontal trajectory

   // delete the blank element at the end
   results.pop_back();
   counter--;
   // fill in remaining fields of previous element
   results[counter].m_segment_type = HorizontalPath::SegmentType::STRAIGHT;
   results[counter].m_path_course = m_waypoint_vector[m_waypoint_vector.size() - 1].m_course_angle.value();

   // calculate path length
   double seg_length = 0.0;
   results[0].m_path_length_cumulative_meters = 0.0;
   for (unsigned int loop = 0; loop < results.size() - 1; loop++) {
      if (results[loop].m_segment_type == HorizontalPath::SegmentType::STRAIGHT) {
         seg_length = sqrt(pow(results[loop + 1].GetXPositionMeters() - results[loop].GetXPositionMeters(), 2) +
                           pow(results[loop + 1].GetYPositionMeters() - results[loop].GetYPositionMeters(), 2));
         results[loop + 1].m_path_length_cumulative_meters = results[loop].m_path_length_cumulative_meters + seg_length;
      } else if (results[loop].m_segment_type == HorizontalPath::SegmentType::TURN) {
         Units::SignedRadiansAngle course_change = AircraftCalculations::ConvertPitoPi(
               Units::RadiansAngle(results[loop].m_turn_info.q_end - results[loop].m_turn_info.q_start));
         seg_length = Units::MetersLength(results[loop].m_turn_info.radius).value() * fabs(course_change.value());
         results[loop + 1].m_path_length_cumulative_meters = results[loop].m_path_length_cumulative_meters + seg_length;
      }
   }

   // set waypoint constraint distances
   // constraint distance is the distance from the next waypoint back to the FAF (waypoint_vector[0])
   // see Powerpoint "Constrained Distance Example.pptx" in JIRA Issue AAES-592 for description of this loop
   unsigned int iTraj = 0;
   for (unsigned int loop = 0; loop < m_waypoint_vector.size() - 1; loop++) {
      if (SamePoint(m_waypoint_vector[loop], results[iTraj])) {
         iTraj++;
         if (SamePoint(m_waypoint_vector[loop + 1], results[iTraj])) {
            // Cases A, C
            m_waypoint_vector[loop].m_precalc_constraints.constraint_along_path_distance =
                  Units::MetersLength(results[iTraj].m_path_length_cumulative_meters);
            continue;
         }
         if (results[iTraj - 1].m_segment_type == HorizontalPath::SegmentType::STRAIGHT) {
            // Case B
            m_waypoint_vector[loop].m_precalc_constraints.constraint_along_path_distance = Units::MetersLength(
                  results[iTraj].m_path_length_cumulative_meters + HalfTurn(results[iTraj].m_turn_info));
            iTraj++;
            continue;
         }
         // Case D
         m_waypoint_vector[loop].m_precalc_constraints.constraint_along_path_distance = Units::MetersLength(
               results[iTraj - 1].m_path_length_cumulative_meters + HalfTurn(results[iTraj - 1].m_turn_info));
         continue;
      }

      if (SamePoint(m_waypoint_vector[loop + 1], results[iTraj])) {
         // Case E
         m_waypoint_vector[loop].m_precalc_constraints.constraint_along_path_distance =
               Units::MetersLength(results[iTraj - 1].m_path_length_cumulative_meters);
         continue;
      }

      if (results[iTraj].m_segment_type == HorizontalPath::SegmentType::TURN) {
         // Case G
         m_waypoint_vector[loop].m_precalc_constraints.constraint_along_path_distance = Units::MetersLength(
               results[iTraj].m_path_length_cumulative_meters + HalfTurn(results[iTraj].m_turn_info));
         iTraj++;
         continue;
      }

      iTraj++;
      if (SamePoint(m_waypoint_vector[loop + 1], results[iTraj])) {
         // Case F
         m_waypoint_vector[loop].m_precalc_constraints.constraint_along_path_distance =
               Units::MetersLength(results[iTraj].m_path_length_cumulative_meters);
         continue;
      }

      // Case H
      m_waypoint_vector[loop].m_precalc_constraints.constraint_along_path_distance =
            Units::MetersLength(results[iTraj].m_path_length_cumulative_meters + HalfTurn(results[iTraj].m_turn_info));
      iTraj++;
   }
   m_waypoint_vector[m_waypoint_vector.size() - 1].m_precalc_constraints.constraint_along_path_distance =
         m_waypoint_vector[m_waypoint_vector.size() - 2].m_precalc_constraints.constraint_along_path_distance;

   // Set the class member as the result of the horizontal planning operation
   m_horizontal_path = results;
   DoHorizontalPathLogging(m_logger, option);
}

// See issue AAES-961  Need aircraft distance to go to calculate vertical prediction
// All of the vertical predictors, except kinematic (Constrained) do not need nor use distance to go.
// aircraft_distance_to_go defaults to infinity
// Cannot use default parameter values in a virtual method.
void EuclideanTrajectoryPredictor::BuildTrajectoryPrediction(aaesim::open_source::WeatherPrediction &weather,
                                                             Units::Length start_altitude) {
   BuildTrajectoryPrediction(weather, start_altitude, Units::infinity());
}

// main method to precalculate the 4D trajectory
void EuclideanTrajectoryPredictor::BuildTrajectoryPrediction(aaesim::open_source::WeatherPrediction &weather,
                                                             Units::Length start_altitude,
                                                             Units::Length aircraft_distance_to_go) {
   m_aircraft_distance_to_go = aircraft_distance_to_go;
   SetAtmosphere(weather.getAtmosphere());

   // calculation the positions of the Precalculation Waypoints
   DefineRoute();

   if (m_vertical_predictor != NULL) {
      CalculateHorizontalTrajectory(FIRST_PASS);
   } else {
      string msg = string("NULL vertical predictor encountered calling Calculate_Horizontal_Traj(FIRST_PASS)\n") +
                   string("Check trajectory class constructor");
      LOG4CPLUS_FATAL(m_logger, msg);
      throw logic_error(msg);
   }

   if (m_vertical_predictor != NULL) {
      m_vertical_predictor->BuildVerticalPrediction(m_horizontal_path, m_waypoint_vector, weather, start_altitude,
                                                    aircraft_distance_to_go);
      DoVerticalPathLogging(m_logger, FIRST_PASS);
   } else {
      string msg = string("NULL vertical predictor encountered calling buildVerticalPrediction (first pass)\n") +
                   string("Check trajectory class constructor");
      LOG4CPLUS_FATAL(m_logger, msg);
      throw logic_error(msg);
   }

   UpdateWeatherPrediction(weather);

   if (m_vertical_predictor != NULL) {
      CalculateHorizontalTrajectory(SECOND_PASS);
   } else {
      string msg = string("NULL vertical predictor encountered calling Calculate_Horizontal_Traj(SECOND_PASS)\n") +
                   string("Check trajectory class constructor");
      LOG4CPLUS_FATAL(EuclideanTrajectoryPredictor::m_logger, msg);
      throw logic_error(msg);
   }

   if (m_vertical_predictor != NULL) {
      m_vertical_predictor->BuildVerticalPrediction(m_horizontal_path, m_waypoint_vector, weather, start_altitude,
                                                    aircraft_distance_to_go);
      DoVerticalPathLogging(m_logger, SECOND_PASS);
   } else {
      string msg = string("NULL vertical predictor encountered calling buildVerticalPrediction (second pass)") +
                   string("Check trajectory class constructor");

      LOG4CPLUS_FATAL(EuclideanTrajectoryPredictor::m_logger, msg);
      throw logic_error(msg);
   }

   m_distance_calculator =
         AlongPathDistanceCalculator(m_horizontal_path, TrajectoryIndexProgressionDirection::UNDEFINED);
   m_position_calculator = PositionCalculator(m_horizontal_path, TrajectoryIndexProgressionDirection::UNDEFINED);
}

// the method to calculate Guidance based on the 4D Trajectory
aaesim::open_source::Guidance EuclideanTrajectoryPredictor::Update(
      const aaesim::open_source::AircraftState &state, const aaesim::open_source::Guidance &current_guidance) {
   aaesim::open_source::Guidance result;
   static const Units::DegreesPerSecondAngularSpeed roll_rate(3.0);  // roll rate for guidance (AAES-668)

   // Calculate Psi command and cross-track error if the aircraft is turning
   // get the distance based on aircraft position
   Units::MetersLength distance_to_go;
   Units::UnsignedRadiansAngle course_at_dtg;
   m_distance_calculator.CalculateAlongPathDistanceFromPosition(
         Units::FeetLength(state.m_x), Units::FeetLength(state.m_y), distance_to_go, course_at_dtg);
   if (!m_distance_calculator.IsPassedEndOfRoute()) {

      result = m_vertical_predictor->Update(
            state, current_guidance,
            distance_to_go);  // calls the 4D Precalculated Descent and issues altitude and speed Guidance

      // get aircraft position based on that calculated distance
      Units::MetersLength x_pos;
      Units::MetersLength y_pos;
      Units::UnsignedRadiansAngle course_at_position;
      m_position_calculator.CalculatePositionFromAlongPathDistance(distance_to_go, x_pos, y_pos, course_at_position);
      std::vector<HorizontalPath>::size_type traj_index = m_position_calculator.GetCurrentTrajectoryIndex();
      if (traj_index == m_horizontal_path.size() - 1) {
         traj_index--;
      }

      // error  check
      Units::SignedRadiansAngle check_cross_track =
            AircraftCalculations::ConvertPitoPi(course_at_position - course_at_dtg);
      if (fabs(check_cross_track.value()) > 0.001) {
         LOG4CPLUS_WARN(m_logger, "AC" << state.m_id << " Course angles from getPosFromPathLength("
                                       << course_at_position << ") and getPathLengthFromPos (" << course_at_dtg
                                       << ") do not agree.  DTG: " << distance_to_go);
      }

      Units::FeetLength distToTrajPoint =
            distance_to_go - Units::MetersLength(m_horizontal_path[traj_index].m_path_length_cumulative_meters);
      Units::SecondsTime timeToTrajPoint = distToTrajPoint / result.m_ground_speed;

      result.m_enu_track_angle = course_at_position;  // set the m_path_course command result

      // calculate cross track as difference between actual and precalculated position
      Units::MetersLength cross_track =
            sqrt(Units::sqr(Units::FeetLength(state.m_x) - x_pos) + Units::sqr(Units::FeetLength(state.m_y) - y_pos));

      // generate cross-track sign based on distance from turn center and change in m_path_course
      double center_dist =
            sqrt(pow(state.m_x * FEET_TO_METERS - m_horizontal_path[traj_index].m_turn_info.x_position_meters, 2) +
                 pow(state.m_y * FEET_TO_METERS - m_horizontal_path[traj_index].m_turn_info.y_position_meters, 2));

      // calculate the cross track error based on distance from center point and m_path_course change if turning
      if (m_horizontal_path[traj_index].m_segment_type == HorizontalPath::SegmentType::TURN) {
         // see if need to roll out
         double rollFactor = 1;                                                                       // dimensionless
         if (traj_index > 0 && (m_horizontal_path[traj_index - 1].m_turn_info.radius.value() < 1)) {  // next leg is
                                                                                                      // straight
            Units::SecondsTime timeToBank = m_horizontal_path[traj_index].m_turn_info.bankAngle / roll_rate;
            // double timeToBank = h_traj[traj_index].m_turn_info.bankAngle.value() / 0.05235980;
            if (timeToTrajPoint <= timeToBank) {
               rollFactor = timeToTrajPoint / timeToBank;
            }
         }
         Units::UnsignedRadiansAngle acCourse = Units::UnsignedRadiansAngle(
               Units::RadiansAngle(m_horizontal_path[traj_index].m_path_course) + Units::PI_RADIANS_ANGLE);
         Units::SignedRadiansAngle courseChange = AircraftCalculations::Convert0to2Pi(course_at_dtg - acCourse);
         // courseChange - positive is left turn, neg is right turn
         // if left turn, distance < radius is left of m_path_course, distance > radius is right of m_path_course
         // if right turn, distance < radius is right of m_path_course, distance > radius is left of m_path_course
         if (courseChange > Units::SignedRadiansAngle(0.0))  // left turn (AC is actually turning right)
         {
            // added for guidance in turns
            result.m_reference_bank_angle = rollFactor * m_horizontal_path[traj_index].m_turn_info.bankAngle;

            if (center_dist <
                Units::MetersLength(m_horizontal_path[traj_index].m_turn_info.radius).value())  // left of m_path_course
            {
               result.m_cross_track_error = cross_track;
            } else  // right of m_path_course
            {
               result.m_cross_track_error = -cross_track;
            }
         } else {
            result.m_reference_bank_angle = -rollFactor * m_horizontal_path[traj_index].m_turn_info.bankAngle;

            if (center_dist < Units::MetersLength(m_horizontal_path[traj_index].m_turn_info.radius).value()) {
               result.m_cross_track_error = -cross_track;
            } else {
               result.m_cross_track_error = cross_track;
            }
         }

         result.m_use_cross_track = true;  // redundant
      }
      // else do straight track trajectory cross-track calculation
      else {
         if (traj_index + 1 >= m_horizontal_path.size()) {
            std::vector<HorizontalPath>::size_type traj_index1 = m_horizontal_path.size() - 2;
            LOG4CPLUS_WARN(m_logger, "traj_index reset from " << traj_index << " to " << traj_index1);
            traj_index = traj_index1;
         }

         // if next segment_type a turn, calculate roll-in time at 3 degrees bank per second
         // then calculate distance to next point and apply roll in to guidance

         if (traj_index > 0 && (m_horizontal_path[traj_index - 1].m_turn_info.radius.value() >
                                1)) {  // roll rate is 3 degrees per second or 0.05235988 radians per second
            Units::SecondsTime timeToBank = m_horizontal_path[traj_index - 1].m_turn_info.bankAngle / roll_rate;
            if (timeToTrajPoint <= timeToBank)  // roll in
            {
               Units::UnsignedRadiansAngle turnAmount = (m_horizontal_path[traj_index - 1].m_turn_info.q_start -
                                                         m_horizontal_path[traj_index - 1].m_turn_info.q_end);
               Units::SignedRadiansAngle courseChange = AircraftCalculations::Convert0to2Pi(turnAmount);
               // right turn is positive, left turn is negative
               double rollFactor = (timeToTrajPoint / timeToBank);
               if (courseChange > Units::ZERO_ANGLE) {  // right turn
                  result.m_reference_bank_angle =
                        -(1.0 - rollFactor) * m_horizontal_path[traj_index - 1].m_turn_info.bankAngle;
               } else {  // left turn
                  result.m_reference_bank_angle =
                        (1.0 - rollFactor) * m_horizontal_path[traj_index - 1].m_turn_info.bankAngle;
               }
            }
         }

         result.m_cross_track_error = Units::MetersLength(
               -(state.m_y * FEET_TO_METERS - m_horizontal_path[traj_index + 1].GetYPositionMeters()) *
                     cos(course_at_dtg) +
               (state.m_x * FEET_TO_METERS - m_horizontal_path[traj_index + 1].GetXPositionMeters()) *
                     sin(course_at_dtg));
      }
      result.m_use_cross_track = true;
   }  // not passed end of route
   else {
      // off the guidance path so no guidance can be calculated. Set the references to reasonable values from current
      // state.
      result.m_reference_altitude = Units::FeetLength(state.m_z);
      result.m_ground_speed = Units::FeetPerSecondSpeed(sqrt(pow(state.m_xd, 2) + pow(state.m_yd, 2)));
      result.m_ias_command = m_vertical_predictor->GetIasAtEndOfRoute();
      result.SetValid(true);
   }

   return result;
}

void EuclideanTrajectoryPredictor::DefineRoute() {
   // loop to process all of the waypoint positions
   for (unsigned int loop = 0; loop < m_waypoint_vector.size(); ++loop) {
      m_waypoint_vector[loop].m_course_angle =
            AircraftCalculations::Convert0to2Pi(m_waypoint_vector[loop].m_course_angle);
   }
}

void EuclideanTrajectoryPredictor::AdjustConstraints(Units::Speed start_speed) {
   // if the constraint for first leg is "unconstrained" then set to start speed
   if (m_waypoint_vector[m_waypoint_vector.size() - 1].m_precalc_constraints.constraint_speedHi >=
       Waypoint::MAX_SPEED_CONSTRAINT - Units::MetersPerSecondSpeed(1))
      m_waypoint_vector[m_waypoint_vector.size() - 1].m_precalc_constraints.constraint_speedHi = start_speed;
   for (auto loop = m_waypoint_vector.size() - 1; loop > 0; --loop) {
      Units::MetersLength H1_High = m_waypoint_vector[loop].m_precalc_constraints.constraint_altHi;
      Units::MetersLength H0_High = m_waypoint_vector[loop - 1].m_precalc_constraints.constraint_altHi;
      if (H1_High < H0_High) {
         m_waypoint_vector[loop - 1].m_precalc_constraints.constraint_altHi = H1_High;
      }
      Units::MetersPerSecondSpeed s1_high = m_waypoint_vector[loop].m_precalc_constraints.constraint_speedHi;
      Units::MetersPerSecondSpeed s0_high = m_waypoint_vector[loop - 1].m_precalc_constraints.constraint_speedHi;
      if (s1_high < s0_high) {
         m_waypoint_vector[loop - 1].m_precalc_constraints.constraint_speedHi = s1_high;
      }
   }

   for (auto loop = 1; loop < m_waypoint_vector.size(); ++loop) {
      Units::MetersLength H1_Low = m_waypoint_vector[loop].m_precalc_constraints.constraint_altLow;
      Units::MetersLength H0_Low = m_waypoint_vector[loop - 1].m_precalc_constraints.constraint_altLow;
      if (H1_Low < H0_Low) {
         m_waypoint_vector[loop].m_precalc_constraints.constraint_altLow = H0_Low;
      }
   }
}

EuclideanTrajectoryPredictor &EuclideanTrajectoryPredictor::operator=(const EuclideanTrajectoryPredictor &obj) {
   if (this != &obj) {
      Copy(obj);
   }

   return *this;
}

void EuclideanTrajectoryPredictor::Copy(const EuclideanTrajectoryPredictor &obj) {

   if (this != &obj) {
      m_waypoint_vector = obj.m_waypoint_vector;
      m_horizontal_path = obj.m_horizontal_path;
      m_vertical_predictor = obj.m_vertical_predictor;

      m_bank_angle = obj.m_bank_angle;
      m_altitude_at_final_waypoint = obj.m_altitude_at_final_waypoint;

      m_aircraft_intent = obj.m_aircraft_intent;
      m_atmosphere = obj.m_atmosphere;

      m_distance_calculator = obj.m_distance_calculator;
      m_position_calculator = obj.m_position_calculator;

      // probably not necessary as this attributte is only set and used by BuildInitialTrajectoryPredictions
      // m_aircraft_distance_to_go = obj.m_aircraft_distance_to_go;
   }
}

bool EuclideanTrajectoryPredictor::operator==(const EuclideanTrajectoryPredictor &obj) const {

   bool match = (m_waypoint_vector == obj.m_waypoint_vector);

   match = match && (m_horizontal_path == obj.m_horizontal_path);

   match = match && (m_bank_angle == obj.m_bank_angle);
   match = match && (m_altitude_at_final_waypoint == obj.m_altitude_at_final_waypoint);

   match = match && (m_aircraft_intent == obj.m_aircraft_intent);
   //  m_aircraft_distance_to_go is used temporarily during BuildInitialTrajectoryPredictions, do not test for equality

   return match;
}

bool EuclideanTrajectoryPredictor::operator!=(const EuclideanTrajectoryPredictor &obj) const {

   // Generic not equals operator.
   //
   // obj:comparison object.
   // returns true if obj doesn't match.
   //         false if obj matches.

   return !operator==(obj);
}

const AircraftIntent &EuclideanTrajectoryPredictor::GetAircraftIntent() const { return m_aircraft_intent; }

const vector<HorizontalPath> &EuclideanTrajectoryPredictor::GetHorizontalPath() const { return m_horizontal_path; }

void EuclideanTrajectoryPredictor::UpdateWeatherPrediction(aaesim::open_source::WeatherPrediction &weather) const {

   // Get wind for option 2
   if (weather.GetPredictedWindOption() == MULTIPLE_DTG_ALONG_ROUTE) {
      // weather.dump();

      // skip if this is not the first time
      if (weather.GetUpdateCount() > 0) {
         LOG4CPLUS_TRACE(m_logger, "Skipping weather update for AC " << m_aircraft_intent.GetId()
                                                                     << " ptr=" << &(weather.east_west));
         return;
      }
      LOG4CPLUS_TRACE(m_logger,
                      "Doing weather update for AC " << m_aircraft_intent.GetId() << " ptr=" << &(weather.east_west));

      // save initial point
      Units::MetersLength alt0 = weather.east_west.GetAltitude(weather.east_west.GetMaxRow());
      Units::Speed windX0 = weather.east_west.GetSpeed(weather.east_west.GetMaxRow());
      Units::Speed windY0 = weather.north_south.GetSpeed(weather.north_south.GetMaxRow());

      const VerticalPath verticalPath(m_vertical_predictor->GetVerticalPath());
      int iVert(0);
      auto iHoriz = m_horizontal_path.begin();
      auto prevHoriz = iHoriz;
      iHoriz++;

      vector<Units::MetersLength> x, y, h;

      // go through waypoints
      for (auto iWaypoint = m_waypoint_vector.begin(); iWaypoint != m_waypoint_vector.end(); iWaypoint++) {

         double dx = prevHoriz->GetXPositionMeters() - iWaypoint->m_x_pos_meters.value();
         double dy = prevHoriz->GetYPositionMeters() - iWaypoint->m_y_pos_meters.value();
         double prevDist2 = dx * dx + dy * dy;

         while (iHoriz != m_horizontal_path.end()) {
            dx = iHoriz->GetXPositionMeters() - iWaypoint->m_x_pos_meters.value();
            dy = iHoriz->GetYPositionMeters() - iWaypoint->m_y_pos_meters.value();
            double dist2 = dx * dx + dy * dy;
            if (dist2 > prevDist2 + 100) {
               // first significant movement away from waypoint == passed it
               break;
            }
            prevDist2 = dist2;
            prevHoriz = iHoriz;
            iHoriz++;
         }

         // find altitude for prev dtg
         double dtg = prevHoriz->m_path_length_cumulative_meters;
         while (verticalPath.along_path_distance_m[iVert] < dtg) {
            iVert++;
         }

         double alt = verticalPath.altitude_m[iVert];
         if (iVert > 0) {
            // interpolate to get alt for horiz dtg
            double weight = (dtg - verticalPath.along_path_distance_m[iVert - 1]) /
                            (verticalPath.along_path_distance_m[iVert] - verticalPath.along_path_distance_m[iVert - 1]);
            alt = weight * verticalPath.altitude_m[iVert] + (1 - weight) * verticalPath.altitude_m[iVert - 1];
         }

         if (alt < alt0.value()) {
            if (h.empty() || h.back().value() + 30 < alt) {
               x.push_back(Units::MetersLength(prevHoriz->GetXPositionMeters()));
               y.push_back(Units::MetersLength(prevHoriz->GetYPositionMeters()));
               h.push_back(Units::MetersLength(alt));
            }
         }
      }

      // set bounds of weather
      weather.east_west.SetBounds(1, h.size() + 1);
      weather.north_south.SetBounds(1, h.size() + 1);

      // look up the wind for each point
      for (int i = 0; i < h.size(); i++) {
         Units::KnotsSpeed wind_x, wind_y;
         weather.GetForecastWind()->InterpolateForecastWind(m_aircraft_intent.GetTangentPlaneSequence(), x[i], y[i],
                                                            h[i], wind_x, wind_y);
         weather.east_west.Insert(i + 1, h[i], wind_x);
         weather.north_south.Insert(i + 1, h[i], wind_y);
      }
      weather.east_west.Insert(h.size() + 1, alt0, windX0);
      weather.north_south.Insert(h.size() + 1, alt0, windY0);

      weather.IncrementUpdateCount();
      // weather.dump();
   }
}

void EuclideanTrajectoryPredictor::SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere) {
   m_atmosphere = atmosphere;
   m_vertical_predictor->SetAtmosphere(atmosphere);
}

const std::vector<HorizontalPath> EuclideanTrajectoryPredictor::EstimateHorizontalTrajectory(
      aaesim::open_source::WeatherPrediction weather_prediction) {
   SetAtmosphere(weather_prediction.getAtmosphere());
   CalculateHorizontalTrajectory(FIRST_PASS);
   return GetHorizontalPath();
}
