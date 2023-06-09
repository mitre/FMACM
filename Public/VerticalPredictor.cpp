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

#include "public/VerticalPredictor.h"
#include "public/AircraftCalculations.h"
#include "public/CoreUtils.h"

using namespace std;
using namespace aaesim::open_source;

VerticalPredictor::VerticalPredictor()
   : m_low_groundspeed_warning(50),
     m_low_groundspeed_fatal(0.1),
     m_descent_angle_max(6.0),
     m_descent_angle_warning(4.0) {

   m_transition_altitude_msl = Units::FeetLength(0.0);
   m_cruise_altitude_msl = Units::FeetLength(37000);
   m_transition_ias = Units::KnotsSpeed(310);
   m_cruise_mach = m_transition_mach = 0.8;
   m_ias_in_tracon = Units::KnotsSpeed(250.0);
   m_altitude_msl_in_tracon = Units::FeetLength(10000.0);
   m_current_trajectory_index = 0;
   m_descent_start_time = Units::SecondsTime(0.0);
}

void VerticalPredictor::SetMembers(const VerticalPredictor &vertical_predictor) {
   m_descent_start_time = vertical_predictor.m_descent_start_time;
   m_ias_in_tracon = vertical_predictor.m_ias_in_tracon;
   m_altitude_msl_in_tracon = vertical_predictor.m_altitude_msl_in_tracon;
   m_transition_ias = vertical_predictor.m_transition_ias;
   m_transition_mach = vertical_predictor.m_transition_mach;
   m_cruise_altitude_msl = vertical_predictor.m_cruise_altitude_msl;
   m_transition_altitude_msl = vertical_predictor.m_transition_altitude_msl;
   m_current_trajectory_index = vertical_predictor.m_current_trajectory_index;
}

Guidance VerticalPredictor::Update(const AircraftState &current_state, const Guidance &current_guidance,
                                   const Units::Length distance_to_go) {

   Guidance guidanceout = CalculateGuidanceCommands(current_state, distance_to_go, current_guidance);

   return guidanceout;
}

PrecalcConstraint VerticalPredictor::CheckActiveConstraint(double along_path_distance_to_go_meter,
                                                           double altitude_msl_meter, double calibrated_airspeed_mps,
                                                           const PrecalcConstraint &constraints,
                                                           double transition_altitude_meter) {

   //   const double SPEED_DIFFERENCE_THRESHOLD = -0.1;
   //   const double HI_SPEED_CONSTRAINT_THRESHOLD = (1000.0 * KNOTS_TO_METERS_PER_SECOND);
   //   const double ALT_DIFFERENCE_THRESHOLD = (100.0 * FEET_TO_METERS);

   PrecalcConstraint result = constraints;

   // if distance is greater than the constraint distance process the constraint values
   if (along_path_distance_to_go_meter > Units::MetersLength(constraints.constraint_along_path_distance).value()) {
      if (altitude_msl_meter >= Units::MetersLength(constraints.constraint_altLow).value() &&
          altitude_msl_meter <= Units::MetersLength(constraints.constraint_altHi).value()) {
         // constraints are not violated
         result.violation_flag = false;
         result.active_flag = ActiveFlagType::SEG_END_MID_ALT;
      } else if (altitude_msl_meter <= Units::MetersLength(constraints.constraint_altLow).value()) {
         // Minimum Altitude Constraint is violated
         result.violation_flag = true;
         result.active_flag = ActiveFlagType::SEG_END_LOW_ALT;
      } else if (altitude_msl_meter >= Units::MetersLength(constraints.constraint_altHi).value()) {
         // constraints not violated
         result.violation_flag = true;
         result.active_flag = ActiveFlagType::SEG_END_AT_ALT;
      }
   }
   // else if altitude is greater than the altitude constraint
   else if (altitude_msl_meter >= Units::MetersLength(constraints.constraint_altHi).value()) {
      // Maximum Altitude Constraint is violated
      result.violation_flag = true;

      if ((calibrated_airspeed_mps - Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value()) <
                Units::MetersPerSecondSpeed(SPEED_DIFFERENCE_THRESHOLD).value() &&
          Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value() <
                Units::MetersPerSecondSpeed(HIGH_SPEED_CONSTRAINT_THRESHOLD).value() &&
          (Units::MetersLength(constraints.constraint_altHi).value() - altitude_msl_meter) <=
                Units::MetersLength(ALT_DIFFERENCE_THRESHOLD).value() &&
          altitude_msl_meter < transition_altitude_meter) {
         result.active_flag = ActiveFlagType::AT_ALT_SLOW;
      } else {
         result.active_flag = ActiveFlagType::AT_ALT_ON_SPEED;
      }
   }
   // else accelerate to upper speed constraint
   else if ((calibrated_airspeed_mps - Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value()) <
                  Units::MetersPerSecondSpeed(SPEED_DIFFERENCE_THRESHOLD).value() &&
            // Numerical precision tolerance
            calibrated_airspeed_mps < Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value() &&
            Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value() <
                  Units::MetersPerSecondSpeed(HIGH_SPEED_CONSTRAINT_THRESHOLD).value() &&
            (Units::MetersLength(constraints.constraint_altHi).value() - altitude_msl_meter) >
                  Units::MetersLength(ALT_DIFFERENCE_THRESHOLD).value()) {
      result.violation_flag = true;
      result.active_flag = ActiveFlagType::BELOW_ALT_SLOW;
   } else if ((calibrated_airspeed_mps - Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value()) <
                    Units::MetersPerSecondSpeed(SPEED_DIFFERENCE_THRESHOLD).value() &&
              // Numerical precision tolerance
              calibrated_airspeed_mps < Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value() &&
              Units::MetersPerSecondSpeed(constraints.constraint_speedHi).value() <
                    Units::MetersPerSecondSpeed(HIGH_SPEED_CONSTRAINT_THRESHOLD).value() &&
              (Units::MetersLength(constraints.constraint_altHi).value() - altitude_msl_meter) <=
                    Units::MetersLength(ALT_DIFFERENCE_THRESHOLD).value() &&
              altitude_msl_meter < transition_altitude_meter) {
      result.violation_flag = true;
      result.active_flag = ActiveFlagType::AT_ALT_SLOW;
   } else {
      // constraints not violated
      result.violation_flag = false;
   }

   return result;
}

PrecalcConstraint VerticalPredictor::FindActiveConstraint(const double &along_path_distance_to_go_meters,
                                                          const vector<PrecalcWaypoint> &precalculated_waypoints) {
   PrecalcConstraint result;
   unsigned int index = 0;

   // loop to find the current constraint being used
   bool found = false;
   for (unsigned int loop = 0; loop < precalculated_waypoints.size() && found == false; loop++) {
      if (along_path_distance_to_go_meters <
          Units::MetersLength(precalculated_waypoints[loop].m_precalc_constraints.constraint_along_path_distance)
                .value()) {
         found = true;
         index = loop + 1;
      }
   }

   // find the desired constraint
   if (index < precalculated_waypoints.size() && found == true) {
      result.active_flag = ActiveFlagType::BELOW_ALT_ON_SPEED;
      result.constraint_along_path_distance =
            precalculated_waypoints[index - 1].m_precalc_constraints.constraint_along_path_distance;
      result.constraint_altLow = precalculated_waypoints[index - 1].m_precalc_constraints.constraint_altLow;
      result.constraint_altHi = precalculated_waypoints[index - 1].m_precalc_constraints.constraint_altHi;
      result.constraint_speedHi = precalculated_waypoints[index - 1].m_precalc_constraints.constraint_speedHi;
      result.constraint_speedLow = precalculated_waypoints[index - 1].m_precalc_constraints.constraint_speedLow;
      result.index = (int)index;
   }

   return result;
}

Guidance VerticalPredictor::CalculateGuidanceCommands(const AircraftState &state, const Units::Length distance_to_go,
                                                      const Guidance &current_guidance) {
   Guidance result = current_guidance;

   if (result.GetSelectedSpeed().GetSpeedType() == UNSPECIFIED_SPEED) {
      // no selected speed, so set it to Mach or IAS depending on altitude
      if (Units::FeetLength(state.m_z) > m_transition_altitude_msl) {
         // Mach
         result.SetSelectedSpeed(AircraftSpeed(MACH_SPEED, m_transition_mach));
      } else {
         // IAS
         result.SetSelectedSpeed(AircraftSpeed(
               INDICATED_AIR_SPEED, Units::MetersPerSecondSpeed(m_vertical_path.cas_mps[m_current_trajectory_index])));
      }
   }

   const Units::MetersLength distance_remaining(distance_to_go);
   double h_next;
   double v_next;
   double h_dot_next;
   double gs_next;

   // if the check if the distance left is <= the start of the precalculated descent distance
   if (distance_remaining.value() <= fabs(m_vertical_path.along_path_distance_m.back())) {
      // Get index.
      m_current_trajectory_index =
            CoreUtils::FindNearestIndex(distance_remaining.value(), m_vertical_path.along_path_distance_m);

      // Set _next values.
      if (m_current_trajectory_index == 0) {
         // Below lowest distance-take values at end of route.

         h_next = m_vertical_path.altitude_m[m_current_trajectory_index];
         v_next = m_vertical_path.cas_mps[m_current_trajectory_index];
         h_dot_next = m_vertical_path.altitude_rate_mps[m_current_trajectory_index];
         gs_next = m_vertical_path.gs_mps[m_current_trajectory_index];

      } else {
         // Interpolate values using distance.
         h_next = CoreUtils::LinearlyInterpolate(m_current_trajectory_index, distance_remaining.value(),
                                                 m_vertical_path.along_path_distance_m, m_vertical_path.altitude_m);

         v_next = CoreUtils::LinearlyInterpolate(m_current_trajectory_index, distance_remaining.value(),
                                                 m_vertical_path.along_path_distance_m, m_vertical_path.cas_mps);

         h_dot_next =
               CoreUtils::LinearlyInterpolate(m_current_trajectory_index, distance_remaining.value(),
                                              m_vertical_path.along_path_distance_m, m_vertical_path.altitude_rate_mps);
         gs_next = CoreUtils::LinearlyInterpolate(m_current_trajectory_index, distance_remaining.value(),
                                                  m_vertical_path.along_path_distance_m, m_vertical_path.gs_mps);
      }

      // Set result
      result.m_reference_altitude = Units::MetersLength(h_next);
      result.m_vertical_speed = Units::MetersPerSecondSpeed(h_dot_next);
      result.m_ias_command = Units::MetersPerSecondSpeed(v_next);
      result.m_ground_speed = Units::MetersPerSecondSpeed(gs_next);
   }

   return result;
}

double VerticalPredictor::CalculateEsfUsingConstantCAS(const double true_airspeed_mps, const double altitude_msl_meter,
                                                       const Units::Temperature temperature) {

   double esf;
   const Units::KelvinTemperature temperature_kelvin(temperature);
   double mach;
   double temp1, temp2, temp3;

   mach = true_airspeed_mps / sqrt(GAMMA * R.value() * temperature_kelvin.value());

   temp1 = 1.0 + (GAMMA - 1.0) / 2 * pow(mach, 2);
   temp2 = (pow(temp1, (-1.0 / (GAMMA - 1)))) * (pow(temp1, (GAMMA / (GAMMA - 1))) - 1.0);

   if (altitude_msl_meter <= GetAtmosphere()->GetTropopauseHeight().value()) {
      temp3 = 1.0 + (GAMMA * R.value() * K_T.value()) / (2 * GRAVITY_METERS_PER_SECOND) * pow(mach, 2) + temp2;
   } else {
      temp3 = 1.0 + temp2;
   }

   esf = 1.0 / temp3;
   return esf;
}

double VerticalPredictor::CalculateEsfUsingConstantMach(const double true_airspeed_mps, const double altitude_msl_meter,
                                                        const Units::Temperature temperature) {
   double esf = 1.0;
   const Units::KelvinTemperature temperature_kelvin(temperature);
   double mach;

   mach = true_airspeed_mps / sqrt(GAMMA * R.value() * temperature_kelvin.value());

   if (altitude_msl_meter <= GetAtmosphere()->GetTropopauseHeight().value()) {
      esf = 1.0 / (1.0 + (GAMMA * R.value() * K_T.value()) / (2 * GRAVITY_METERS_PER_SECOND) * pow(mach, 2));
   }

   return esf;
}

void VerticalPredictor::TrimDuplicatesFromVerticalPath() {
   // Trims duplicate records from the trajectory.
   vector<double>::iterator time_it = m_vertical_path.time_to_go_sec.begin();
   vector<double>::iterator x_it = m_vertical_path.along_path_distance_m.begin();
   vector<double>::iterator alt_it = m_vertical_path.altitude_m.begin();
   vector<double>::iterator speed_it = m_vertical_path.cas_mps.begin();
   vector<double>::iterator alt_delta_it = m_vertical_path.altitude_rate_mps.begin();
   vector<double>::iterator speed_delta_it = m_vertical_path.tas_rate_mps.begin();
   vector<Units::Speed>::iterator tas_it = m_vertical_path.true_airspeed.begin();
   vector<double>::iterator theta_it = m_vertical_path.theta_radians.begin();
   vector<double>::iterator gs_it = m_vertical_path.gs_mps.begin();
   vector<double>::iterator mass_it = m_vertical_path.mass_kg.begin();

   // double to store previous time stamp
   double prev_time = -1;

   while (time_it != m_vertical_path.time_to_go_sec.end()) {
      // if time doesn't match previous time iterate position
      if (prev_time != (*time_it)) {
         prev_time = (*time_it);  // set new previous time value

         // iterate position in lists

         ++time_it;
         ++x_it;
         ++alt_it;
         ++speed_it;
         ++alt_delta_it;
         ++speed_delta_it;
         ++theta_it;
         ++gs_it;
         ++mass_it;
         ++tas_it;
      }
      // else remove duplicate value
      else {
         time_it = m_vertical_path.time_to_go_sec.erase(time_it);
         x_it = m_vertical_path.along_path_distance_m.erase(x_it);
         alt_it = m_vertical_path.altitude_m.erase(alt_it);
         speed_it = m_vertical_path.cas_mps.erase(speed_it);
         alt_delta_it = m_vertical_path.altitude_rate_mps.erase(alt_delta_it);
         tas_it = m_vertical_path.true_airspeed.erase(tas_it);
         speed_delta_it = m_vertical_path.tas_rate_mps.erase(speed_delta_it);
         theta_it = m_vertical_path.theta_radians.erase(theta_it);
         gs_it = m_vertical_path.gs_mps.erase(gs_it);
         mass_it = m_vertical_path.mass_kg.erase(mass_it);
      }
   }
}

VerticalPredictor &VerticalPredictor::operator=(const VerticalPredictor &obj) {

   if (this != &obj) {
      this->m_wind_calculator = obj.m_wind_calculator;
      this->m_transition_altitude_msl = obj.m_transition_altitude_msl;
      this->m_cruise_altitude_msl = obj.m_cruise_altitude_msl;
      this->m_transition_ias = obj.m_transition_ias;
      this->m_transition_mach = obj.m_transition_mach;
      this->m_cruise_mach = obj.m_cruise_mach;
      this->m_ias_in_tracon = obj.m_ias_in_tracon;
      this->m_altitude_msl_in_tracon = obj.m_altitude_msl_in_tracon;
      this->m_descent_start_time = obj.m_descent_start_time;
      this->m_precalculated_constraints = obj.m_precalculated_constraints;
      this->m_vertical_path = obj.m_vertical_path;
      this->m_current_trajectory_index = obj.m_current_trajectory_index;
      this->m_course_calculator = obj.m_course_calculator;
      this->m_atmosphere = obj.m_atmosphere;
   }

   return *this;
}

bool VerticalPredictor::operator==(const VerticalPredictor &obj) const {

   // Note: not including mCalcWind comparison here because differences
   // between this->mCalcWind and obj.mCalcWind should not affect
   // processing.

   bool match = this->m_transition_altitude_msl == obj.m_transition_altitude_msl;
   match = match && (this->m_cruise_altitude_msl == obj.m_cruise_altitude_msl);
   match = match && (this->m_transition_ias == obj.m_transition_ias);
   match = match && (this->m_transition_mach == obj.m_transition_mach);
   match = match && (this->m_cruise_mach == obj.m_cruise_mach);
   match = match && (this->m_ias_in_tracon == obj.m_ias_in_tracon);
   match = match && (this->m_altitude_msl_in_tracon == obj.m_altitude_msl_in_tracon);
   match = match && (this->m_descent_start_time == obj.m_descent_start_time);
   match = match && (this->m_precalculated_constraints == obj.m_precalculated_constraints);
   match = match && (this->m_vertical_path == obj.m_vertical_path);
   match = match && (this->m_current_trajectory_index == obj.m_current_trajectory_index);
   match = match && (this->m_atmosphere == obj.m_atmosphere);

   return match;
}

bool VerticalPredictor::operator!=(const VerticalPredictor &obj) const { return !this->operator==(obj); }

const Units::MetersPerSecondSpeed VerticalPredictor::SPEED_DIFFERENCE_THRESHOLD(-0.1);
const Units::KnotsSpeed VerticalPredictor::HIGH_SPEED_CONSTRAINT_THRESHOLD(1000);
const Units::FeetLength VerticalPredictor::ALT_DIFFERENCE_THRESHOLD(100);
