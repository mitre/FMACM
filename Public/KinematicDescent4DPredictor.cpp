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

#include "public/Waypoint.h"
#include <cmath>
#include "public/KinematicDescent4DPredictor.h"
#include "public/SimulationTime.h"

using namespace std;
using namespace aaesim::open_source;
using namespace aaesim::open_source::constants;

log4cplus::Logger KinematicDescent4DPredictor::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("KinematicDescent4DPredictor"));

const Units::Length KinematicDescent4DPredictor::m_vertical_tolerance_distance = Units::FeetLength(400);

KinematicDescent4DPredictor::KinematicDescent4DPredictor()
   : m_kinematic_descent_type(CONSTRAINED),
     m_altitude_at_end_of_route(Units::zero()),
     m_deceleration_mps(0.5 * KNOTS_TO_METERS_PER_SECOND),
     m_deceleration_level_mps(0.75 * KNOTS_TO_METERS_PER_SECOND),
     m_deceleration_fpa_mps(0.3 * KNOTS_TO_METERS_PER_SECOND),
     m_const_gamma_cas_term_rad(2.9 * DEGREES_TO_RADIAN),
     m_const_gamma_cas_er_rad(3.1 * DEGREES_TO_RADIAN),
     m_const_gamma_mach_rad(4.0 * DEGREES_TO_RADIAN),
     m_prediction_too_low(false),
     m_prediction_too_high(false) {}

KinematicDescent4DPredictor::~KinematicDescent4DPredictor() = default;

void KinematicDescent4DPredictor::SetMembers(const double &mach_descent, const Units::Speed ias_descent,
                                             const Units::Length cruise_altitude,
                                             const Units::Length transition_altitude) {
   m_transition_mach = mach_descent;
   m_transition_ias = ias_descent;
   m_cruise_altitude_msl = cruise_altitude;

   if (IsCruiseMachValid()) {
      m_transition_altitude_msl = transition_altitude;
   } else {
      m_transition_altitude_msl = Units::Infinity();
   }
}

void KinematicDescent4DPredictor::BuildVerticalPrediction(vector<HorizontalPath> &horizontal_path,
                                                          vector<PrecalcWaypoint> &precalc_waypoints,
                                                          const WeatherPrediction &weather_prediction,
                                                          const Units::Length &start_altitude,
                                                          const Units::Length &aircraft_distance_to_go) {
   m_start_altitude_msl = start_altitude;
   m_prediction_too_low = false;
   m_prediction_too_high = false;
   HorizontalPath start_pos(horizontal_path.back());
   LOG4CPLUS_TRACE(m_logger, "Building vertical prediction from ("
                                   << start_pos.GetXPositionMeters() << "," << start_pos.GetYPositionMeters()
                                   << "), alt=" << m_start_altitude_msl
                                   << " dtg: " << Units::MetersLength(aircraft_distance_to_go).value());
   m_course_calculator =
         DirectionOfFlightCourseCalculator(horizontal_path, TrajectoryIndexProgressionDirection::UNDEFINED);

   ConstrainedVerticalPath(horizontal_path, precalc_waypoints, m_deceleration_mps, m_const_gamma_cas_term_rad,
                           m_const_gamma_cas_er_rad, m_const_gamma_mach_rad, weather_prediction,
                           aircraft_distance_to_go);

   TrimDuplicatesFromVerticalPath();

   m_current_trajectory_index = m_vertical_path.along_path_distance_m.size() - 1;
}

/*
 *  Build prediction to current position and then level flight.  Use aircraft distance to go to determine
 *  the segment that contains aircraft position.  If the prediction is above or below the aircraft altitude
 *  by more than the allowable amount at the same distance to go as the aircraft, then do an FPA from the
 *  last waypoint of the prediction that has a positive FPA angle.
 */
void KinematicDescent4DPredictor::ConstrainedVerticalPath(vector<HorizontalPath> &horizontal_path,
                                                          vector<PrecalcWaypoint> &precalc_waypoints,
                                                          double deceleration, double const_gamma_cas_term,
                                                          double const_gamma_cas_er, double const_gamma_mach,
                                                          const WeatherPrediction &weather_prediction,
                                                          const Units::Length &aircraft_distance_to_go) {

   m_vertical_path_waypoint_index.clear();
   VerticalPath trajTemp;
   trajTemp.mass_kg.push_back(-1.0);
   trajTemp.time_to_go_sec.push_back(Units::SecondsTime(m_descent_start_time).value());
   trajTemp.along_path_distance_m.push_back(0);
   trajTemp.altitude_m.push_back(Units::MetersLength(m_altitude_at_end_of_route).value());
   trajTemp.cas_mps.push_back(Units::MetersPerSecondSpeed(m_ias_at_end_of_route).value());
   trajTemp.mach.push_back(Units::MetersPerSecondSpeed(weather_prediction.GetForecastAtmosphere()->IASToMach(
                                                             Units::MetersPerSecondSpeed(m_ias_at_end_of_route),
                                                             Units::MetersLength(m_altitude_at_end_of_route)))
                                 .value());
   trajTemp.altitude_rate_mps.push_back(0);
   trajTemp.true_airspeed.push_back(weather_prediction.CAS2TAS(m_ias_at_end_of_route, m_altitude_at_end_of_route));
   trajTemp.tas_rate_mps.push_back(0);
   trajTemp.theta_radians.push_back(0);
   trajTemp.flap_setting.push_back(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED);

   Units::Speed vwpara;
   Units::Speed vwperp;
   Units::Speed Vwx, Vwy;
   Units::UnsignedAngle course = m_course_calculator.GetCourseAtPathEnd();
   ComputeWindCoefficients(m_altitude_at_end_of_route, Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                           Vwx, Vwy);

   Units::Speed initialgs = sqrt(Units::sqr(weather_prediction.getAtmosphere()->CAS2TAS(m_ias_at_end_of_route,
                                                                                        m_altitude_at_end_of_route)) -
                                 Units::sqr(vwperp)) +
                            vwpara;

   trajTemp.gs_mps.push_back(Units::MetersPerSecondSpeed(initialgs).value());
   trajTemp.wind_velocity_east.push_back(Vwx);
   trajTemp.wind_velocity_north.push_back(Vwy);

   trajTemp.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::UNDETERMINED);

   m_vertical_path = trajTemp;
   m_vertical_path_waypoint_index.push_back(0);  // index for vertical path at first precalc waypoint

   VerticalPath last_state = m_vertical_path;
   VerticalPath last_waypoint_state = m_vertical_path;

   double FPA;
   double alt1 = min(Units::MetersLength(m_transition_altitude_msl).value(),
                     Units::MetersLength(m_cruise_altitude_msl).value());

   if (aircraft_distance_to_go < Units::NauticalMilesLength(1)) {
      LOG4CPLUS_WARN(m_logger,
                     "Attempting to perform constrained vertical path with less than 1 nautical mile to go.  "
                     "Calculating level path, instead.");
      m_vertical_path = LevelVerticalPath(
            m_vertical_path,
            Units::MetersLength(
                  precalc_waypoints[precalc_waypoints.size() - 1].m_precalc_constraints.constraint_along_path_distance)
                  .value(),
            horizontal_path, weather_prediction, aircraft_distance_to_go);
      return;
   }

   while (m_vertical_path.altitude_m.back() < alt1) {
      if (m_vertical_path.along_path_distance_m.back() > horizontal_path.back().m_path_length_cumulative_meters) {
         break;
      }

      if (m_vertical_path.altitude_m.back() < 10000 * FEET_TO_METERS) {
         m_vertical_path = ConstantCasVerticalPath(m_vertical_path, alt1, horizontal_path, precalc_waypoints,
                                                   const_gamma_cas_term, weather_prediction, aircraft_distance_to_go);
      } else {
         m_vertical_path = ConstantCasVerticalPath(m_vertical_path, alt1, horizontal_path, precalc_waypoints,
                                                   const_gamma_cas_er, weather_prediction, aircraft_distance_to_go);
      }
      if (m_prediction_too_low || m_prediction_too_high) {
         break;
      }

      if (m_precalculated_constraints.violation_flag) {
         if (m_precalculated_constraints.active_flag == ActiveFlagType::SEG_END_LOW_ALT) {
            FPA = atan2((Units::MetersLength(m_precalculated_constraints.constraint_altLow).value() -
                         last_waypoint_state.altitude_m.back()),
                        (Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value() -
                         last_waypoint_state.along_path_distance_m.back()));
            m_vertical_path = ConstantFpaDecelerationVerticalPath(
                  last_waypoint_state, Units::MetersLength(m_precalculated_constraints.constraint_altLow).value(),
                  m_deceleration_fpa_mps,
                  Units::MetersPerSecondSpeed(m_precalculated_constraints.constraint_speedHi).value(), FPA,
                  horizontal_path, precalc_waypoints, weather_prediction, aircraft_distance_to_go);
            if (m_prediction_too_low || m_prediction_too_high) {
               break;
            }

            m_vertical_path = ConstantGeometricFpaVerticalPath(
                  m_vertical_path, Units::MetersLength(m_precalculated_constraints.constraint_altLow).value(), FPA,
                  horizontal_path, precalc_waypoints, weather_prediction, aircraft_distance_to_go);

         } else if (m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_ON_SPEED) {
            FPA = atan2(Units::MetersLength(m_precalculated_constraints.constraint_altHi).value() -
                              last_state.altitude_m.back(),
                        Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value() -
                              last_state.along_path_distance_m.back());

            if (FPA > 0.10 * PI / 180.0) {
               m_vertical_path = ConstantGeometricFpaVerticalPath(
                     last_state, Units::MetersLength(m_precalculated_constraints.constraint_altHi).value(), FPA,
                     horizontal_path, precalc_waypoints, weather_prediction, aircraft_distance_to_go);
               if (m_prediction_too_low || m_prediction_too_high) {
                  break;
               }
            }

            m_vertical_path = LevelVerticalPath(
                  m_vertical_path,
                  Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value(),
                  horizontal_path, weather_prediction, aircraft_distance_to_go);

         } else if (m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_SLOW) {

            if (m_precalculated_constraints.index < precalc_waypoints.size()) {
               m_vertical_path = ConstantDecelerationVerticalPath(
                     m_vertical_path, m_precalculated_constraints.constraint_along_path_distance,
                     m_precalculated_constraints.constraint_altHi, deceleration,
                     Units::MetersPerSecondSpeed(m_precalculated_constraints.constraint_speedHi).value(),
                     horizontal_path, weather_prediction, aircraft_distance_to_go);
            }

            if (m_prediction_too_low || m_prediction_too_high) {
               break;
            }

            if ((m_vertical_path.along_path_distance_m.back() >
                 Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value()) &&
                (m_vertical_path.altitude_m.back() <
                 Units::MetersLength(m_precalculated_constraints.constraint_altLow).value())) {
               // If idle-descent acceleration is below low altitude constraint-
               // redo with with a constant FPA deceleration trajectory.
               FPA = atan2((Units::MetersLength(m_precalculated_constraints.constraint_altLow).value() -
                            last_state.altitude_m.back()),
                           (Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value() -
                            last_state.along_path_distance_m.back()));
               Units::DegreesAngle uFPA = Units::RadiansAngle(FPA);
               if (FPA > Units::RadiansAngle(m_descent_angle_max).value())
                  LOG4CPLUS_WARN(m_logger, "prediction FPA is " << uFPA.value() << " which is greater than "
                                                                << m_descent_angle_warning.value());
               if (uFPA < m_descent_angle_max) {
                  m_vertical_path = ConstantFpaDecelerationVerticalPath(
                        last_state, Units::MetersLength(m_precalculated_constraints.constraint_altLow).value(),
                        m_deceleration_fpa_mps,
                        Units::MetersPerSecondSpeed(m_precalculated_constraints.constraint_speedHi).value(), FPA,
                        horizontal_path, precalc_waypoints, weather_prediction, aircraft_distance_to_go);
               }
            }
         } else if (m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_SLOW) {
            if (m_precalculated_constraints.index < precalc_waypoints.size()) {
               m_vertical_path = LevelDecelerationVerticalPath(
                     m_vertical_path, m_precalculated_constraints.constraint_along_path_distance,
                     m_deceleration_level_mps,
                     Units::MetersPerSecondSpeed(m_precalculated_constraints.constraint_speedHi).value(),
                     horizontal_path, weather_prediction, aircraft_distance_to_go);
            }
         }

         if (m_prediction_too_low || m_prediction_too_high) {
            break;
         }
      }

      last_state = m_vertical_path;

      if (m_vertical_path.along_path_distance_m.back() >
          Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value()) {
         if (last_waypoint_state == m_vertical_path) {
            LOG4CPLUS_ERROR(m_logger, "Infinite loop detected...trying to progress");
            return;
         }

         last_waypoint_state = m_vertical_path;
         while (m_vertical_path_waypoint_index.back() >= last_waypoint_state.altitude_m.size()) {
            m_vertical_path_waypoint_index.pop_back();
         }
         if (m_vertical_path_waypoint_index.back() < last_waypoint_state.altitude_m.size() - 1) {
            m_vertical_path_waypoint_index.push_back(last_waypoint_state.altitude_m.size() - 1);
         }

         if (m_vertical_path.altitude_m.back() > m_start_altitude_msl.value()) {
            break;
         }
      }
   }

   // Constant Mach segment
   last_state = m_vertical_path;

   if (!m_prediction_too_low && !m_prediction_too_high) {
      while ((m_vertical_path.altitude_m.back() < m_start_altitude_msl.value()) &&
             (m_vertical_path.along_path_distance_m.back() <= horizontal_path.back().m_path_length_cumulative_meters)) {
         m_vertical_path =
               ConstantMachVerticalPath(last_state, m_start_altitude_msl.value(), horizontal_path, precalc_waypoints,
                                        const_gamma_mach, weather_prediction, aircraft_distance_to_go);

         if (m_prediction_too_low || m_prediction_too_high) {
            break;
         }

         if (m_precalculated_constraints.violation_flag) {
            if (m_precalculated_constraints.active_flag == ActiveFlagType::SEG_END_LOW_ALT) {
               FPA = atan2(Units::MetersLength(m_precalculated_constraints.constraint_altLow).value() -
                                 last_state.altitude_m.back(),
                           Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value() -
                                 last_state.along_path_distance_m.back());

               // if unable to reach low altitude constraint due to excessive FPA, then continue
               // constantMachVerticalPath
               if (FPA > 10 * PI / 180.0) {
                  LOG4CPLUS_WARN(
                        m_logger,
                        "constrainedVerticalPath prediction in mach segment cannot reach low altitude constraint");
               } else if (FPA > 0.10 * PI / 180.0) {
                  m_vertical_path = ConstantGeometricFpaVerticalPath(
                        last_state, Units::MetersLength(m_precalculated_constraints.constraint_altLow).value(), FPA,
                        horizontal_path, precalc_waypoints, weather_prediction, aircraft_distance_to_go);
               } else {
                  m_vertical_path = LevelVerticalPath(
                        last_state,
                        Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value(),
                        horizontal_path, weather_prediction, aircraft_distance_to_go);
               }

               if (m_prediction_too_low || m_prediction_too_high) {
                  break;
               }

               m_vertical_path = LevelVerticalPath(
                     m_vertical_path,
                     Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value(),
                     horizontal_path, weather_prediction, aircraft_distance_to_go);
            } else if (m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_ON_SPEED) {
               FPA = atan2(Units::MetersLength(m_precalculated_constraints.constraint_altHi).value() -
                                 last_state.altitude_m.back(),
                           Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value() -
                                 last_state.along_path_distance_m.back());

               if (FPA > 0.10 * PI / 180.0) {
                  m_vertical_path = ConstantGeometricFpaVerticalPath(
                        last_state, Units::MetersLength(m_precalculated_constraints.constraint_altHi).value(), FPA,
                        horizontal_path, precalc_waypoints, weather_prediction, aircraft_distance_to_go);
               } else {
                  m_vertical_path = LevelVerticalPath(
                        last_state,
                        Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value(),
                        horizontal_path, weather_prediction, aircraft_distance_to_go);
               }
               m_vertical_path = LevelVerticalPath(
                     m_vertical_path,
                     Units::MetersLength(m_precalculated_constraints.constraint_along_path_distance).value(),
                     horizontal_path, weather_prediction, aircraft_distance_to_go);
            }
         }
         last_state = m_vertical_path;
      }
   }

   if (m_prediction_too_low) {
      LOG4CPLUS_TRACE(m_logger, "Constrained prediction too low.  Replanning with FPA.");
   }

   if (m_prediction_too_high) {
      LOG4CPLUS_TRACE(m_logger, "Constrained prediction too high.  Replanning with FPA.");
   }

   if (m_prediction_too_low || m_prediction_too_high) {
      // Prediction is at aircraft position but not at aircraft altitude.  Need to replan using FPA.
      int start_index = -1;
      bool fpa_start_found = false;
      while (!m_vertical_path_waypoint_index.empty()) {
         // Find a waypoint at which the predicted altitude allows an FPA to the current position of the aircraft
         start_index = m_vertical_path_waypoint_index.back();
         m_vertical_path_waypoint_index.pop_back();
         double distance_to_aircraft_m = Units::MetersLength(aircraft_distance_to_go).value() -
                                         m_vertical_path.along_path_distance_m[start_index];
         double altitude_to_aircraft_m =
               Units::MetersLength(m_start_altitude_msl).value() - m_vertical_path.altitude_m[start_index];
         double FPA = atan2(altitude_to_aircraft_m, distance_to_aircraft_m);
         if (FPA < Units::RadiansAngle(m_descent_angle_max).value()) {
            // found a point where the FPA is not too large.  Check if FPA will violate altitude constraints
            fpa_start_found = true;
            for (unsigned int loop = start_index + 1; loop < precalc_waypoints.size(); loop++) {
               if (aircraft_distance_to_go <
                   precalc_waypoints[loop].m_precalc_constraints.constraint_along_path_distance)
                  break;
               double pred_alt_m =
                     m_vertical_path.altitude_m[start_index] +
                     Units::MetersLength(altitude_to_aircraft_m / distance_to_aircraft_m).value() *
                           Units::MetersLength(
                                 precalc_waypoints[loop].m_precalc_constraints.constraint_along_path_distance)
                                 .value();
               if (pred_alt_m >
                         Units::MetersLength(precalc_waypoints[loop].m_precalc_constraints.constraint_altHi).value() ||
                   pred_alt_m <
                         Units::MetersLength(precalc_waypoints[loop].m_precalc_constraints.constraint_altLow).value()) {
                  fpa_start_found = false;
                  break;
               }
            }
            if (fpa_start_found) break;
         }
      }

      if (!fpa_start_found) {
         LOG4CPLUS_WARN(m_logger, "Constrained prediction unable to reach aircraft altitude at aircraft position");
      } else {
         TrimVerticalPath(m_vertical_path, start_index);
         m_vertical_path =
               ConstantFpaToCurrentPositionVerticalPath(m_vertical_path, horizontal_path, precalc_waypoints,
                                                        const_gamma_mach, weather_prediction, aircraft_distance_to_go);
      }

      if (m_vertical_path.altitude_m.back() < Units::MetersLength(m_start_altitude_msl).value()) {

         LOG4CPLUS_TRACE(m_logger,
                         "Did not complete FPA above transition altitude. " << m_vertical_path.altitude_m.back());
      }

      if (m_vertical_path.along_path_distance_m.back() < Units::MetersLength(aircraft_distance_to_go).value()) {
         LOG4CPLUS_TRACE(m_logger, "Did not complete FPA above transition altitude");
      }
   }

   // Level segment to end of prediction
   if (m_vertical_path.altitude_m.back() < Units::MetersLength(m_transition_altitude_msl).value()) {
      while (m_vertical_path.along_path_distance_m.back() < horizontal_path.back().m_path_length_cumulative_meters) {
         m_precalculated_constraints =
               FindActiveConstraint(m_vertical_path.along_path_distance_m.back(), precalc_waypoints);
         m_precalculated_constraints =
               CheckActiveConstraint(m_vertical_path.along_path_distance_m.back(), m_vertical_path.altitude_m.back(),
                                     m_vertical_path.cas_mps.back(), m_precalculated_constraints,
                                     Units::MetersLength(m_transition_altitude_msl).value());
         if (m_precalculated_constraints.violation_flag &&
             (m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_SLOW ||
              m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_SLOW)) {
            m_vertical_path = LevelDecelerationVerticalPath(
                  m_vertical_path, m_deceleration_level_mps,
                  Units::MetersPerSecondSpeed(m_precalculated_constraints.constraint_speedHi).value(), horizontal_path,
                  weather_prediction, aircraft_distance_to_go);
         }
         m_vertical_path = LevelVerticalPath(m_vertical_path, horizontal_path.back().m_path_length_cumulative_meters,
                                             horizontal_path, weather_prediction, aircraft_distance_to_go);
      }
   }
   m_vertical_path = LevelVerticalPath(m_vertical_path, horizontal_path.back().m_path_length_cumulative_meters,
                                       horizontal_path, weather_prediction, aircraft_distance_to_go);
}

VerticalPath KinematicDescent4DPredictor::ConstantCasVerticalPath(VerticalPath vertical_path, double altitude_at_end,
                                                                  vector<HorizontalPath> &horizontal_path,
                                                                  vector<PrecalcWaypoint> &precalc_waypoints,
                                                                  double CAS_gamma,
                                                                  const WeatherPrediction &weather_prediction,
                                                                  const Units::Length &aircraft_distance_to_go) {
   VerticalPath result;

   const double delta_t = -TIME_STEP_SECONDS;
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   bool bracket_found = false;

   result = vertical_path;

   m_precalculated_constraints = FindActiveConstraint(dist, precalc_waypoints);
   m_precalculated_constraints = CheckActiveConstraint(dist, h, v_cas, m_precalculated_constraints,
                                                       Units::MetersLength(m_transition_altitude_msl).value());

   while (h < altitude_at_end && m_precalculated_constraints.active_flag <= ActiveFlagType::BELOW_ALT_ON_SPEED) {
      double v_tas = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->CAS2TAS(
                                                       Units::MetersPerSecondSpeed(v_cas), Units::MetersLength(h)))
                           .value();

      double esf;
      double dh_dt;
      double dv_dh;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;

      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);

      esf = CalculateEsfUsingConstantCAS(v_tas, h,
                                         weather_prediction.getAtmosphere()->GetTemperature(Units::MetersLength(h)));

      // climb/descent rate
      dh_dt = -v_tas * sin(CAS_gamma);

      // change in speed with respect to altitude
      dv_dh = (1 / esf - 1) * (GRAVITY_METERS_PER_SECOND / v_tas);

      // acceleration rate
      dv_dt = dv_dh * dh_dt;

      h_new = dh_dt * delta_t + h;
      v_tas_new = dv_dt * delta_t + v_tas;

      v_cas_new = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->TAS2CAS(
                                                    Units::MetersPerSecondSpeed(v_tas_new), Units::MetersLength(h_new)))
                        .value();

      double gsnew = sqrt(pow(v_tas_new * cos(CAS_gamma), 2) - pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
                     Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = dist - delta_t * gsnew;

      const double mach = weather_prediction.GetForecastAtmosphere()->IASToMach(Units::MetersPerSecondSpeed(v_cas_new),
                                                                                Units::MetersLength(h_new));

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.mach.push_back(mach);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(CAS_gamma);
      result.gs_mps.push_back(gsnew);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::CONSTANT_CAS);
      result.mass_kg.push_back(-1.0);
      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t));
      result.flap_setting.push_back(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED);

      if (!bracket_found && dist_new > Units::MetersLength(aircraft_distance_to_go).value()) {
         bracket_found = true;

         if ((h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_TRACE(m_logger, "Prediction alt too high. pred: "
                                            << h_new
                                            << " start_alt: " << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_high = true;
            return result;
         }
         if ((h_new + Units::MetersLength(m_vertical_tolerance_distance).value()) <
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_TRACE(m_logger, "Prediction alt too low. pred: "
                                            << h_new
                                            << " start_alt: " << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_low = true;
            return result;
         }
      }

      dist = dist_new;    // in meters
      h = h_new;          // in meters
      v_cas = v_cas_new;  // in meters per second

      if (m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_ON_SPEED) {
         m_precalculated_constraints = CheckActiveConstraint(dist, h, v_cas, m_precalculated_constraints,
                                                             Units::MetersLength(m_transition_altitude_msl).value());
      }
   }

   if ((aircraft_distance_to_go < Units::MetersLength(Units::Infinity())) &&
       (h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
      LOG4CPLUS_TRACE(m_logger, "Prediction alt too high. pred: " << h << " start_alt: "
                                                                  << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_high = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::ConstantMachVerticalPath(VerticalPath vertical_path, double altitude_at_end,
                                                                   vector<HorizontalPath> &horizontal_path,
                                                                   vector<PrecalcWaypoint> &precalc_waypoints,
                                                                   double gamma,
                                                                   const WeatherPrediction &weather_prediction,
                                                                   const Units::Length &aircraft_distance_to_go) {
   VerticalPath result;

   const double delta_t = -TIME_STEP_SECONDS;
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   result = vertical_path;

   m_precalculated_constraints = FindActiveConstraint(dist, precalc_waypoints);
   m_precalculated_constraints = CheckActiveConstraint(dist, h, v_cas, m_precalculated_constraints,
                                                       Units::MetersLength(m_transition_altitude_msl).value());

   while ((h < altitude_at_end) && (m_precalculated_constraints.active_flag <= ActiveFlagType::BELOW_ALT_ON_SPEED ||
                                    m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_SLOW ||
                                    m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_SLOW)) {
      double v_tas = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->CAS2TAS(
                                                       Units::MetersPerSecondSpeed(v_cas), Units::MetersLength(h)))
                           .value();
      double esf;
      double dh_dt;
      double dv_dh;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      bool bracket_found = false;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);

      esf = CalculateEsfUsingConstantMach(v_tas, h,
                                          weather_prediction.getAtmosphere()->GetTemperature(Units::MetersLength(h)));

      // climb/descent rate
      dh_dt = -v_tas * sin(gamma);

      // change in speed with respect to altitude
      dv_dh = (1 / esf - 1) * (GRAVITY_METERS_PER_SECOND / v_tas);

      // acceleration rate
      dv_dt = dv_dh * dh_dt;

      h_new = dh_dt * delta_t + h;

      v_tas_new = dv_dt * delta_t + v_tas;

      v_cas_new = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->TAS2CAS(
                                                    Units::MetersPerSecondSpeed(v_tas_new), Units::MetersLength(h_new)))
                        .value();

      double gsnew = sqrt(pow(v_tas_new * cos(gamma), 2) - pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
                     Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = dist - delta_t * gsnew;

      const double mach = weather_prediction.GetForecastAtmosphere()->IASToMach(Units::MetersPerSecondSpeed(v_cas_new),
                                                                                Units::MetersLength(h_new));

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.mach.push_back(mach);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(gamma);
      result.gs_mps.push_back(gsnew);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::CONSTANT_MACH);
      result.mass_kg.push_back(-1.0);
      result.flap_setting.push_back(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED);

      curr_time = result.time_to_go_sec.back();

      result.time_to_go_sec.push_back(curr_time + fabs(delta_t));  // adds last time +0.5 to the end since
                                                                   // fabs(delta_t) is 0.5

      if (!bracket_found && dist_new > Units::MetersLength(aircraft_distance_to_go).value()) {
         bracket_found = true;
         if ((h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_TRACE(m_logger, "Prediction alt too high. pred: "
                                            << h_new
                                            << " start_alt: " << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_high = true;
            return result;
         }
         if ((h_new + Units::MetersLength(m_vertical_tolerance_distance).value()) <
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_TRACE(m_logger, "Prediction alt too low. pred: "
                                            << h_new
                                            << " start_alt: " << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_low = true;
            return result;
         }
      }

      dist = dist_new;    // in meters
      h = h_new;          // in meters
      v_cas = v_cas_new;  // in meters per second

      if (m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_ON_SPEED ||
          m_precalculated_constraints.active_flag == ActiveFlagType::BELOW_ALT_SLOW ||
          m_precalculated_constraints.active_flag == ActiveFlagType::AT_ALT_SLOW) {
         m_precalculated_constraints = CheckActiveConstraint(dist, h, v_cas, m_precalculated_constraints,
                                                             Units::MetersLength(m_transition_altitude_msl).value());
      }
   }

   if ((aircraft_distance_to_go < Units::MetersLength(Units::Infinity())) &&
       (h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
      LOG4CPLUS_TRACE(m_logger, "Prediction alt too high. pred: " << h << " start_alt: "
                                                                  << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_high = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::ConstantGeometricFpaVerticalPath(
      VerticalPath vertical_path, double altitude_at_end, double flight_path_angle,
      vector<HorizontalPath> &horizontal_path, vector<PrecalcWaypoint> &precalc_waypoints,
      const WeatherPrediction &weather_prediction, const Units::Length &aircraft_distance_to_go) {
   VerticalPath result = vertical_path;

   const double delta_t = -TIME_STEP_SECONDS;
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];
   double theta = vertical_path.theta_radians[vertical_path.theta_radians.size() - 1];

   bool bracket_found = false;

   while (h < altitude_at_end) {
      double v_tas;
      double esf;
      double dh_dt;
      double dv_dh;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);
      v_tas = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->CAS2TAS(
                                                Units::MetersPerSecondSpeed(v_cas), Units::MetersLength(h)))
                    .value();

      if (h <= Units::MetersLength(m_transition_altitude_msl).value()) {
         esf = CalculateEsfUsingConstantCAS(v_tas, h,
                                            weather_prediction.getAtmosphere()->GetTemperature(Units::MetersLength(h)));
      } else {
         esf = CalculateEsfUsingConstantMach(
               v_tas, h, weather_prediction.getAtmosphere()->GetTemperature(Units::MetersLength(h)));
      }

      // climb/descent rate
      dh_dt = -v_tas * sin(theta);

      // change in speed with respect to altitude
      dv_dh = (1 / esf - 1) * (GRAVITY_METERS_PER_SECOND / v_tas);

      // acceleration rate
      dv_dt = dv_dh * dh_dt;

      h_new = dh_dt * delta_t + h;

      v_tas_new = dv_dt * delta_t + v_tas;

      v_cas_new = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->TAS2CAS(
                                                    Units::MetersPerSecondSpeed(v_tas_new), Units::MetersLength(h_new)))
                        .value();

      double gsnew = sqrt(pow(v_tas_new * cos(theta), 2) - pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
                     Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = gsnew * (-delta_t) + dist;

      double dh_dt_update = -gsnew * tan(flight_path_angle);

      if (fabs(dh_dt_update) > v_tas_new) {
         string msg = string("ConstantGeometricVerticalPath is about to take asin() of a number greater than 1.0\n") +
                      string("This will result in theta becoming NaN");
         LOG4CPLUS_FATAL(m_logger, msg);
         throw logic_error(msg);
      }
      double theta_new = asin(-dh_dt_update / v_tas_new);

      const double mach = weather_prediction.GetForecastAtmosphere()->IASToMach(Units::MetersPerSecondSpeed(v_cas_new),
                                                                                Units::MetersLength(h_new));

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.mach.push_back(mach);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta_new);
      result.gs_mps.push_back(gsnew);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::FPA);
      result.mass_kg.push_back(-1.0);
      result.flap_setting.push_back(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED);

      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t));  // adds last time +0.5 to the end since
                                                                   // fabs(delta_t) is 0.5

      if (!bracket_found && dist_new > Units::MetersLength(aircraft_distance_to_go).value()) {
         bracket_found = true;
         if (h - Units::MetersLength(m_vertical_tolerance_distance).value() >
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_TRACE(m_logger, "Prediction alt too high. pred: "
                                            << h_new
                                            << " start_alt: " << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_high = true;
            return result;
         }
         if ((h_new + Units::MetersLength(m_vertical_tolerance_distance).value()) <
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_TRACE(m_logger, "Prediction alt too low. pred: "
                                            << h_new
                                            << " start_alt: " << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_low = true;
            return result;
         }
      }

      dist = dist_new;    // in meters
      h = h_new;          // in meters
      v_cas = v_cas_new;  // in meters per second
      theta = theta_new;
   }

   if ((aircraft_distance_to_go < Units::MetersLength(Units::Infinity())) &&
       (h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
      LOG4CPLUS_TRACE(m_logger, "Prediction alt too high. pred: " << h << " start_alt: "
                                                                  << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_high = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::ConstantFpaDecelerationVerticalPath(
      VerticalPath vertical_path, double altitude_at_end, double deceleration_mps, double velocity_cas_end,
      double flight_path_angle, vector<HorizontalPath> &horizontal_path, vector<PrecalcWaypoint> &precalc_waypoints,
      const WeatherPrediction &weather_prediction, const Units::Length &aircraft_distance_to_go) {
   VerticalPath result = vertical_path;

   const double delta_t = -TIME_STEP_SECONDS;
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];
   double theta = vertical_path.theta_radians[vertical_path.theta_radians.size() - 1];

   while ((h < altitude_at_end) && (v_cas < velocity_cas_end)) {
      double v_tas = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->CAS2TAS(
                                                       Units::MetersPerSecondSpeed(v_cas), Units::MetersLength(h)))
                           .value();
      Units::KilogramsMeterDensity rho;
      Units::Pressure p;
      double dh_dt;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      bool bracket_found = false;

      Units::MetersPerSecondSpeed Vwx, Vwy;
      Units::HertzFrequency dVwx_dh, dVwy_dh;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      m_wind_calculator.ComputeWindGradients(Units::MetersLength(h), weather_prediction, Vwx, Vwy, dVwx_dh, dVwy_dh);

      double Vw_para = Vwx.value() * cos(course) + Vwy.value() * sin(course);
      double Vw_perp = -Vwx.value() * sin(course) + Vwy.value() * cos(course);

      weather_prediction.getAtmosphere()->AirDensity(Units::MetersLength(h), rho, p);

      double gs_new = sqrt(pow(v_tas * cos(theta), 2) - pow(Vw_perp, 2)) + Vw_para;

      // climb/descent rate
      dh_dt = -gs_new * tan(flight_path_angle);
      dv_dt = -deceleration_mps;

      h_new = dh_dt * delta_t + h;
      v_tas_new = dv_dt * delta_t + v_tas;
      v_cas_new = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->TAS2CAS(
                                                    Units::MetersPerSecondSpeed(v_tas_new), Units::MetersLength(h_new)))
                        .value();
      dist_new = gs_new * (-delta_t) + dist;
      if (fabs(dh_dt) > v_tas_new) {
         string msg = string(
                            "ConstantFpaDecelerationVerticalPath is about to take asin() of a number greater "
                            "than 1.0\n") +
                      string("This will result in theta becoming NaN");
         LOG4CPLUS_FATAL(m_logger, msg);
         throw logic_error(msg);
      }
      theta = asin((-dh_dt) / v_tas_new);

      const double mach = weather_prediction.GetForecastAtmosphere()->IASToMach(Units::MetersPerSecondSpeed(v_cas_new),
                                                                                Units::MetersLength(h_new));

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.mach.push_back(mach);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta);
      result.gs_mps.push_back(gs_new);
      result.mass_kg.push_back(-1.0);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::FPA_DECEL);
      result.flap_setting.push_back(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED);
      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t));  // adds last time +0.5 to the end since
                                                                   // fabs(delta_t) is 0.5

      if (!bracket_found && dist_new > Units::MetersLength(aircraft_distance_to_go).value()) {
         bracket_found = true;
         if ((h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_TRACE(m_logger, "Prediction alt too high. pred: "
                                            << h_new
                                            << " start_alt: " << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_high = true;
            return result;
         }
         if ((h_new + Units::MetersLength(m_vertical_tolerance_distance).value()) <
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_TRACE(m_logger, "Prediction alt too low. pred: "
                                            << h_new
                                            << " start_alt: " << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_low = true;
            return result;
         }
      }

      dist = dist_new;    // in meters
      h = h_new;          // in meters
      v_cas = v_cas_new;  // in meters per second
   }

   if ((aircraft_distance_to_go < Units::MetersLength(Units::Infinity())) &&
       (h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
      LOG4CPLUS_TRACE(m_logger, "Prediction alt too high. pred: " << h << " start_alt: "
                                                                  << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_high = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::LevelDecelerationVerticalPath(VerticalPath vertical_path, double deceleration,
                                                                        double velocity_cas_end,
                                                                        vector<HorizontalPath> &horizontal_path,
                                                                        const WeatherPrediction &weather_prediction,
                                                                        const Units::Length &aircraft_distance_to_go) {

   VerticalPath result = vertical_path;

   const double delta_t = -TIME_STEP_SECONDS;
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   while (v_cas < velocity_cas_end) {
      double v_tas = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->CAS2TAS(
                                                       Units::MetersPerSecondSpeed(v_cas), Units::MetersLength(h)))
                           .value();
      double dh_dt;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);

      // climb/descent rate
      dh_dt = 0.0;
      double theta_new = 0.0;

      // acceleration rate
      dv_dt = -deceleration;

      h_new = dh_dt * delta_t + h;
      v_tas_new = dv_dt * delta_t + v_tas;
      v_cas_new = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->TAS2CAS(
                                                    Units::MetersPerSecondSpeed(v_tas_new), Units::MetersLength(h_new)))
                        .value();

      double gsnew = sqrt(pow(v_tas_new * cos(theta_new), 2) - pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
                     Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = dist - delta_t * gsnew;

      const double mach = weather_prediction.GetForecastAtmosphere()->IASToMach(Units::MetersPerSecondSpeed(v_cas_new),
                                                                                Units::MetersLength(h_new));

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.mach.push_back(mach);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta_new);
      result.gs_mps.push_back(gsnew);
      result.mass_kg.push_back(-1.0);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::LEVEL_DECEL1);
      result.flap_setting.push_back(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED);

      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t));  // adds last time +0.5 to the end since
                                                                   // fabs(delta_t) is 0.5

      dist = dist_new;    // in meters
      h = h_new;          // in meters
      v_cas = v_cas_new;  // in meters per second
   }

   if ((result.along_path_distance_m.back() > Units::MetersLength(aircraft_distance_to_go).value()) &&
       ((h + Units::MetersLength(m_vertical_tolerance_distance).value()) <
        Units::MetersLength(m_start_altitude_msl).value())) {
      LOG4CPLUS_TRACE(m_logger, "Prediction alt too low. pred: " << h << " start_alt: "
                                                                 << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_low = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::LevelDecelerationVerticalPath(VerticalPath vertical_path,
                                                                        Units::Length distance_to_go,
                                                                        double deceleration, double velocity_cas_end,
                                                                        vector<HorizontalPath> &horizontal_path,
                                                                        const WeatherPrediction &weather_prediction,
                                                                        const Units::Length &aircraft_distance_to_go) {

   VerticalPath result = vertical_path;

   const double delta_t = -TIME_STEP_SECONDS;
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   double distEnd = Units::MetersLength(distance_to_go).value();

   while (v_cas < velocity_cas_end && dist <= distEnd) {
      double v_tas = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->CAS2TAS(
                                                       Units::MetersPerSecondSpeed(v_cas), Units::MetersLength(h)))
                           .value();
      double dh_dt;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);

      // climb/descent rate
      dh_dt = 0.0;
      double theta_new = 0.0;

      // acceleration rate
      dv_dt = -deceleration;

      h_new = dh_dt * delta_t + h;
      v_tas_new = dv_dt * delta_t + v_tas;
      v_cas_new = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->TAS2CAS(
                                                    Units::MetersPerSecondSpeed(v_tas_new), Units::MetersLength(h_new)))
                        .value();

      double gsnew = sqrt(pow(v_tas_new * cos(theta_new), 2) - pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
                     Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = dist - delta_t * gsnew;

      const double mach = weather_prediction.GetForecastAtmosphere()->IASToMach(Units::MetersPerSecondSpeed(v_cas_new),
                                                                                Units::MetersLength(h_new));

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.mach.push_back(mach);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta_new);
      result.gs_mps.push_back(gsnew);
      result.mass_kg.push_back(-1.0);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::LEVEL_DECEL2);
      result.flap_setting.push_back(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED);

      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t));  // adds last time +0.5 to the end since
                                                                   // fabs(delta_t) is 0.5

      dist = dist_new;    // in meters
      h = h_new;          // in meters
      v_cas = v_cas_new;  // in meters per second
   }

   if ((result.along_path_distance_m.back() > Units::MetersLength(aircraft_distance_to_go).value()) &&
       ((h + Units::MetersLength(m_vertical_tolerance_distance).value()) <
        Units::MetersLength(m_start_altitude_msl).value())) {
      LOG4CPLUS_TRACE(m_logger, "Prediction alt too low. pred: " << h << " start_alt: "
                                                                 << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_low = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::LevelVerticalPath(VerticalPath vertical_path, double x_end,
                                                            vector<HorizontalPath> &horizontal_path,
                                                            const WeatherPrediction &weather_prediction,
                                                            const Units::Length &aircraft_distance_to_go) {
   VerticalPath result;

   const double delta_t = -TIME_STEP_SECONDS;
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];

   result = vertical_path;

   while (fabs(dist) < fabs(x_end)) {
      double v_tas = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->CAS2TAS(
                                                       Units::MetersPerSecondSpeed(v_cas), Units::MetersLength(h)))
                           .value();
      double dh_dt;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double theta_new;
      double curr_time;
      double gsnew;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);

      // climb/descent rate
      dh_dt = 0.0;

      // acceleration rate
      dv_dt = 0.0;

      theta_new = 0.0;

      h_new = dh_dt * delta_t + h;

      v_tas_new = dv_dt * delta_t + v_tas;

      v_cas_new = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->TAS2CAS(
                                                    Units::MetersPerSecondSpeed(v_tas_new), Units::MetersLength(h_new)))
                        .value();

      gsnew = sqrt(pow(v_tas_new * cos(theta_new), 2) - pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
              Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = gsnew * (-delta_t) + dist;

      const double mach = weather_prediction.GetForecastAtmosphere()->IASToMach(Units::MetersPerSecondSpeed(v_cas_new),
                                                                                Units::MetersLength(h_new));

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.mach.push_back(mach);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta_new);
      result.gs_mps.push_back(gsnew);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::LEVEL);
      result.flap_setting.push_back(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED);
      result.mass_kg.push_back(-1.0);

      curr_time = result.time_to_go_sec.back();
      result.time_to_go_sec.push_back(curr_time + fabs(delta_t));  // adds last time +0.5 to the end since
                                                                   // fabs(delta_t) is 0.5

      // set values for next iteration of the loop
      dist = dist_new;
      h = h_new;
      v_cas = v_cas_new;
   }

   if ((result.along_path_distance_m.back() > Units::MetersLength(aircraft_distance_to_go).value()) &&
       ((h + Units::MetersLength(m_vertical_tolerance_distance).value()) <
        Units::MetersLength(m_start_altitude_msl).value())) {
      LOG4CPLUS_TRACE(m_logger, "Prediction alt too low. pred: " << h << " start_alt: "
                                                                 << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_low = true;
   }

   return result;
}

// TODO: change velocity_cas_end to Units
VerticalPath KinematicDescent4DPredictor::ConstantDecelerationVerticalPath(
      VerticalPath vertical_path, Units::Length distance_to_go, Units::Length altitude_high, double deceleration,
      double velocity_cas_end, vector<HorizontalPath> &horizontal_path, const WeatherPrediction &weather_prediction,
      const Units::Length &aircraft_distance_to_go) {
   VerticalPath result = vertical_path;

   const double delta_t = -TIME_STEP_SECONDS;
   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   double distEnd = Units::MetersLength(distance_to_go).value();
   double altEnd = Units::MetersLength(altitude_high).value();

   bool bracket_found = false;

   while (v_cas < velocity_cas_end && h < altEnd && dist < distEnd) {
      double v_tas = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->CAS2TAS(
                                                       Units::MetersPerSecondSpeed(v_cas), Units::MetersLength(h)))
                           .value();
      double dh_dt;
      double dv_dt;
      double h_new;
      double dist_new;
      double v_tas_new;
      double v_cas_new;
      double curr_time;

      Units::Speed vwpara;
      Units::Speed vwperp;
      Units::Speed Vwx, Vwy;
      Units::UnsignedRadiansAngle course;
      m_course_calculator.CalculateCourseAtAlongPathDistance(Units::MetersLength(dist), course);
      ComputeWindCoefficients(Units::MetersLength(h), Units::RadiansAngle(course), weather_prediction, vwpara, vwperp,
                              Vwx, Vwy);

      // One degree descent
      double theta_new = PI / 180.0;

      // climb/descent rate
      dh_dt = -v_tas * sin(theta_new);

      // acceleration rate
      dv_dt = -deceleration;

      h_new = dh_dt * delta_t + h;

      v_tas_new = dv_dt * delta_t + v_tas;

      v_cas_new = Units::MetersPerSecondSpeed(weather_prediction.getAtmosphere()->TAS2CAS(
                                                    Units::MetersPerSecondSpeed(v_tas_new), Units::MetersLength(h_new)))
                        .value();

      double gsnew = sqrt(pow(v_tas_new * cos(theta_new), 2) - pow(Units::MetersPerSecondSpeed(vwperp).value(), 2)) +
                     Units::MetersPerSecondSpeed(vwpara).value();

      dist_new = dist - delta_t * gsnew;

      const double mach = weather_prediction.GetForecastAtmosphere()->IASToMach(Units::MetersPerSecondSpeed(v_cas_new),
                                                                                Units::MetersLength(h_new));

      result.along_path_distance_m.push_back(dist_new);
      result.cas_mps.push_back(v_cas_new);
      result.mach.push_back(mach);
      result.altitude_m.push_back(h_new);
      result.altitude_rate_mps.push_back(dh_dt);
      result.true_airspeed.push_back(Units::MetersPerSecondSpeed(v_tas_new));
      result.tas_rate_mps.push_back(dv_dt);
      result.theta_radians.push_back(theta_new);
      result.gs_mps.push_back(gsnew);
      result.wind_velocity_east.push_back(Vwx);
      result.wind_velocity_north.push_back(Vwy);
      result.algorithm_type.push_back(VerticalPath::PredictionAlgorithmType::CONSTANT_DECEL);
      result.flap_setting.push_back(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED);
      result.mass_kg.push_back(-1.0);

      curr_time = result.time_to_go_sec.back();

      result.time_to_go_sec.push_back(curr_time + fabs(delta_t));  // adds last time +0.5 to the end since
                                                                   // fabs(delta_t) is 0.5

      if (!bracket_found && dist_new > Units::MetersLength(aircraft_distance_to_go).value()) {
         bracket_found = true;
         if ((h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_TRACE(m_logger, "Prediction alt too high. pred: "
                                            << h_new
                                            << " start_alt: " << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_high = true;
            return result;
         }
         if ((h_new + Units::MetersLength(m_vertical_tolerance_distance).value()) <
             Units::MetersLength(m_start_altitude_msl).value()) {
            LOG4CPLUS_TRACE(m_logger, "Prediction alt too low. pred: "
                                            << h_new
                                            << " start_alt: " << Units::MetersLength(m_start_altitude_msl).value());
            m_prediction_too_low = true;
            return result;
         }
      }

      // set values for next iteration of the loop
      dist = dist_new;    // in meters
      h = h_new;          // in meters
      v_cas = v_cas_new;  // in meters per second
   }

   if ((aircraft_distance_to_go < Units::MetersLength(Units::Infinity())) &&
       (h - Units::MetersLength(m_vertical_tolerance_distance).value()) >
             Units::MetersLength(m_start_altitude_msl).value()) {
      LOG4CPLUS_TRACE(m_logger, "Prediction alt too high. pred: " << h << " start_alt: "
                                                                  << Units::MetersLength(m_start_altitude_msl).value());
      m_prediction_too_high = true;
   }

   return result;
}

VerticalPath KinematicDescent4DPredictor::ConstantFpaToCurrentPositionVerticalPath(
      VerticalPath vertical_path, std::vector<HorizontalPath> &horizontal_path,
      std::vector<PrecalcWaypoint> &precalc_waypoints, double const_gamma_mach,
      const WeatherPrediction &weather_prediction, const Units::Length &aircraft_distance_to_go) {

   Units::Length distance_to_plan = aircraft_distance_to_go;

   // Should probably throw exception if trying to re-predict with aircraft distance to go set to infinite
   if (aircraft_distance_to_go == Units::MetersLength(Units::infinity())) {
      LOG4CPLUS_ERROR(m_logger,
                      "Attempting to re-predict constrained vertical path with infinite aircraft distance to go.");
      distance_to_plan = Units::MetersLength(horizontal_path.back().m_path_length_cumulative_meters);
   }

   VerticalPath result = vertical_path;
   result.algorithm_type.back() = VerticalPath::FPA_TO_CURRENT_POS;

   double dist = vertical_path.along_path_distance_m[vertical_path.along_path_distance_m.size() - 1];
   double v_cas = vertical_path.cas_mps[vertical_path.cas_mps.size() - 1];
   double h = vertical_path.altitude_m[vertical_path.altitude_m.size() - 1];

   const double descent_ratio =
         (m_start_altitude_msl.value() - h) / (Units::MetersLength(aircraft_distance_to_go).value() - dist);
   double fpa = atan2(m_start_altitude_msl.value() - h, Units::MetersLength(aircraft_distance_to_go).value() - dist);

   while (dist < Units::MetersLength(aircraft_distance_to_go).value() &&
          h < Units::MetersLength(m_transition_altitude_msl).value()) {
      PrecalcConstraint constraints = FindActiveConstraint(dist, precalc_waypoints);
      constraints =
            CheckActiveConstraint(dist, h, v_cas, constraints, Units::MetersLength(m_transition_altitude_msl).value());

      double distance_left = Units::MetersLength(constraints.constraint_along_path_distance).value();
      if (distance_left > Units::MetersLength(aircraft_distance_to_go).value()) {
         distance_left = Units::MetersLength(aircraft_distance_to_go).value();
      }

      const double altitude_at_end = h + (distance_left - dist) * descent_ratio;
      // The first step in ConstantFPADecelerationVerticalPath() and ConstantGeometricVerticalPath() uses the
      // previous value of theta.  A new theta is not calculated until the second step.  If the aircraft
      // position is close to the waypoint calculation of theta can result in NaN or an FPA angle too high.
      // See Issue AAES-1025.  This should rarely occur.

      if (descent_ratio > 0.2) {
         LOG4CPLUS_TRACE(m_logger, "Aircraft position too close to waypoint for adequate FPA.  See Issue AAES-1025");
         LOG4CPLUS_TRACE(m_logger, "dist_left: " << (distance_left - dist) << ", alt_change: "
                                                 << (m_start_altitude_msl.value() - h) << ", fpa: " << fpa);
         if (fpa > Units::RadiansAngle(m_descent_angle_max).value()) {
            fpa = Units::RadiansAngle(m_descent_angle_max).value();
         }
         result = ConstantGeometricFpaVerticalPath(result, altitude_at_end, fpa, horizontal_path, precalc_waypoints,
                                                   weather_prediction, Units::Length(Units::infinity()));
         result.altitude_m.back() = altitude_at_end;
         if (result.altitude_m.size() > 2) {
            result.altitude_m[result.altitude_m.size() - 2] = altitude_at_end;
         }
         h = result.altitude_m.back();
         dist = result.along_path_distance_m.back();
      }

      while (dist < Units::MetersLength(constraints.constraint_along_path_distance).value() && h < altitude_at_end) {
         if (v_cas < Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value()) {
            if (m_prediction_too_high || m_prediction_too_low) {
               result = ConstantFpaDecelerationVerticalPath(
                     result, altitude_at_end, m_deceleration_fpa_mps,
                     Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value(), fpa, horizontal_path,
                     precalc_waypoints, weather_prediction, Units::Length(Units::infinity()));
            } else {
               result = ConstantFpaDecelerationVerticalPath(
                     result, altitude_at_end, m_deceleration_mps,
                     Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value(), fpa, horizontal_path,
                     precalc_waypoints, weather_prediction, Units::Length(Units::infinity()));
            }
         }
         result = ConstantGeometricFpaVerticalPath(result, altitude_at_end, fpa, horizontal_path, precalc_waypoints,
                                                   weather_prediction, Units::Length(Units::infinity()));

         dist = result.along_path_distance_m.back();
         v_cas = result.cas_mps.back();
         h = result.altitude_m.back();
         if (dist > Units::MetersLength(aircraft_distance_to_go).value()) {
            break;
         }
      }
      if (h > altitude_at_end && dist < Units::MetersLength(constraints.constraint_along_path_distance).value()) {
         result = LevelVerticalPath(result, Units::MetersLength(constraints.constraint_along_path_distance).value(),
                                    horizontal_path, weather_prediction, Units::Length(Units::infinity()));
         dist = result.along_path_distance_m.back();
         v_cas = result.cas_mps.back();
         h = result.altitude_m.back();
      }
   }

   // either at transition altitude or prediction goes past aircraft distance to go
   if (result.altitude_m.back() < Units::MetersLength(m_start_altitude_msl).value()) {
      result = ConstantMachVerticalPath(result, m_start_altitude_msl.value(), horizontal_path, precalc_waypoints,
                                        const_gamma_mach, weather_prediction, Units::Length(Units::infinity()));
   }

   double prediction_dist =
         Units::MetersLength(
               precalc_waypoints[precalc_waypoints.size() - 1].m_precalc_constraints.constraint_along_path_distance)
               .value();
   result = LevelVerticalPath(result, prediction_dist, horizontal_path, weather_prediction,
                              Units::Length(Units::infinity()));
   return result;
};

void KinematicDescent4DPredictor::ComputeWindCoefficients(Units::Length altitude, Units::Angle course,
                                                          const WeatherPrediction &weather_prediction,
                                                          Units::Speed &parallel_wind_velocity,
                                                          Units::Speed &perpendicular_wind_velocity,
                                                          Units::Speed &wind_velocity_x,
                                                          Units::Speed &wind_velocity_y) {
   Units::HertzFrequency dVwx_dh, dVwy_dh;

   m_wind_calculator.ComputeWindGradients(altitude, weather_prediction, wind_velocity_x, wind_velocity_y, dVwx_dh,
                                          dVwy_dh);

   parallel_wind_velocity = wind_velocity_x * cos(course) + wind_velocity_y * sin(course);
   perpendicular_wind_velocity = -wind_velocity_x * sin(course) + wind_velocity_y * cos(course);
};

void KinematicDescent4DPredictor::TrimVerticalPath(VerticalPath &vertical_path, int path_index) {
   if (path_index >= vertical_path.along_path_distance_m.size()) {
      LOG4CPLUS_WARN(m_logger, "path_index greater than vertical path size: unable to trim it");
      return;
   }
   vertical_path.along_path_distance_m.erase(vertical_path.along_path_distance_m.begin() + path_index + 1,
                                             vertical_path.along_path_distance_m.end());
   vertical_path.algorithm_type.erase(vertical_path.algorithm_type.begin() + path_index + 1,
                                      vertical_path.algorithm_type.end());
   vertical_path.altitude_m.erase(vertical_path.altitude_m.begin() + path_index + 1, vertical_path.altitude_m.end());
   vertical_path.altitude_rate_mps.erase(vertical_path.altitude_rate_mps.begin() + path_index + 1,
                                         vertical_path.altitude_rate_mps.end());
   vertical_path.tas_rate_mps.erase(vertical_path.tas_rate_mps.begin() + path_index + 1,
                                    vertical_path.tas_rate_mps.end());
   vertical_path.cas_mps.erase(vertical_path.cas_mps.begin() + path_index + 1, vertical_path.cas_mps.end());
   vertical_path.gs_mps.erase(vertical_path.gs_mps.begin() + path_index + 1, vertical_path.gs_mps.end());
   vertical_path.mass_kg.erase(vertical_path.mass_kg.begin() + path_index + 1, vertical_path.mass_kg.end());
   vertical_path.theta_radians.erase(vertical_path.theta_radians.begin() + path_index + 1,
                                     vertical_path.theta_radians.end());
   vertical_path.time_to_go_sec.erase(vertical_path.time_to_go_sec.begin() + path_index + 1,
                                      vertical_path.time_to_go_sec.end());
   vertical_path.true_airspeed.erase(vertical_path.true_airspeed.begin() + path_index + 1,
                                     vertical_path.true_airspeed.end());
   vertical_path.wind_velocity_east.erase(vertical_path.wind_velocity_east.begin() + path_index + 1,
                                          vertical_path.wind_velocity_east.end());
   vertical_path.wind_velocity_north.erase(vertical_path.wind_velocity_north.begin() + path_index + 1,
                                           vertical_path.wind_velocity_north.end());
   vertical_path.mass_kg.erase(vertical_path.mass_kg.begin() + path_index + 1, vertical_path.mass_kg.end());
}
