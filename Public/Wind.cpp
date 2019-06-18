// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000.
//
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <public/Wind.h>


#include "aaesim/Bada.h"
#include "public/Wind.h"
#include "public/WindZero.h"
#include "public/AircraftCalculations.h"
#include "public/InternalObserver.h"

using std::cout;
using std::shared_ptr;
using std::string;

log4cplus::Logger Wind::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("Wind"));

const Units::NauticalMilesLength Wind::SAMPLING_DISTANCE_FROM_END_OF_ROUTE(60);
const Units::FeetLength Wind::MAXIMUM_ALTITUDE_LIMIT(45000);
const Units::FeetLength Wind::MINIMUM_ALTITUDE_LIMIT(0);

Units::Length Wind::m_blending_altitude_limit = Units::FeetLength(5000.0);

shared_ptr<Wind> Wind::m_wind_truth_instance;

bool Wind::m_use_wind = false;

Wind::Wind() = default;

Wind::~Wind() = default;

void Wind::UpdatePredictedWindsAtAltitudeFromSensedWind(const AircraftState &current_state,
                                                        WeatherPrediction &weather_prediction) {
   /*
    * Algorithm Description:
    * For all altitudes above and below the current_state altitude by less than
    * Wind::windBlendingAltitudeLimit, the incoming predicted wind velocities will be updated
    * to smoothly blend sensed (from current_state) into predicted. At the current_state
    * altitude, the predicted wind will be the same as the sensed wind. Above and below
    * it will smoothly transition to the forecast velocity values using a linear weight
    * algorithm.  Note that altitude lines must not be repeated in the output.  This is
    * checked comparing the current altitude with the blended wind matrix to set the size
    * of the predicted wind matrices and to set the altitudes and velocities in the predicted
    * wind matrices accordingly.
    */

   // Local predicted wind WindStack to operate on. The returned matrices
   // will be updated just prior to return
   WindStack local_blended_x = weather_prediction.east_west;
   WindStack local_blended_y = weather_prediction.north_south;

   // Define the limits that we need to use for wind blending

   Units::Length currentAlt = Units::FeetLength(current_state.m_z);
   Units::Length maxAlt = currentAlt + Wind::m_blending_altitude_limit;


   if (maxAlt > MAXIMUM_ALTITUDE_LIMIT) {
      maxAlt = MAXIMUM_ALTITUDE_LIMIT;
   }

   Units::Length minAlt = (currentAlt - Wind::m_blending_altitude_limit);

   if (Units::FeetLength(minAlt) < MINIMUM_ALTITUDE_LIMIT) {
      minAlt = MINIMUM_ALTITUDE_LIMIT;
   }

   // Loop over the predicted matrices. The altitude values between
   // the two should always be in-sync, so we can write one loop
   // to iterate over both.

   const Units::Speed Vwx_sensed = Units::MetersPerSecondSpeed(current_state.m_Vwx);
   const Units::Speed Vwy_sensed = Units::MetersPerSecondSpeed(current_state.m_Vwy);

   int iRow;

   Units::Length altFromPrediction;

   for (iRow = local_blended_x.GetMaxRow(); iRow >= 1; iRow--) {

      altFromPrediction = local_blended_x.GetAltitude(
            iRow); // this will go down in altitude from highest to lowest value stored in local_blended_x

      if (altFromPrediction > maxAlt || altFromPrediction < minAlt) {
         continue;
      } // above max altitude or below minAlt, no need to blend winds

      // Blend winds
      // NOTE: local_blended_x,y store velocity in knots. The aircraft state object stores sensed wind in meters per second. Unit conversions are important.

      Units::Length altDiff = currentAlt - altFromPrediction;

      const double weightValue = 1.0 - (abs(altDiff) /
                                        Wind::m_blending_altitude_limit);

      const double unityMinusWeightValue = 1.0 - weightValue;

      const Units::Speed Vwx_predicted = local_blended_x.GetSpeed(iRow);
      const Units::Speed Vwy_predicted = local_blended_y.GetSpeed(iRow);
      const Units::Speed Vwx_update((weightValue * Vwx_sensed) + (unityMinusWeightValue * Vwx_predicted));
      const Units::Speed Vwy_update((weightValue * Vwy_sensed) + (unityMinusWeightValue * Vwy_predicted));
      local_blended_x.Set(iRow, altFromPrediction, Vwx_update);
      local_blended_y.Set(iRow, altFromPrediction, Vwy_update);
   }

   // Add the current location and sensed wind to the matrices also
   // The below loop is a very verbose way of updating the predicted wind matrix. However,
   // WindStack does not contain update/append operations. Remove the below when WindStack
   // is updated.

   // Get correct new bounds.

   int currentAltIx = -1;

   for (iRow = local_blended_x.GetMinRow(); iRow <= local_blended_x.GetMaxRow() && currentAltIx == -1; iRow++) {
      if (abs(currentAlt - local_blended_x.GetAltitude(iRow)) < Units::FeetLength(0.1)) {
         // Current altitude found in blended wind matrix.

         currentAltIx = iRow;
      }
   }

   const int newMaxBound = ((currentAltIx == -1) ? (local_blended_x.GetMaxRow() + 1) : local_blended_x.GetMaxRow());

   weather_prediction.east_west.SetBounds(1, newMaxBound); // this will delete all data
   weather_prediction.north_south.SetBounds(1, newMaxBound); // this will delete all data
   for (iRow = local_blended_x.GetMinRow(); iRow <= local_blended_x.GetMaxRow(); iRow++) {

      if (iRow != currentAltIx) {

         // Take winds from blended matrix.

         weather_prediction.east_west.Set(iRow, local_blended_x.GetAltitude(iRow), local_blended_x.GetSpeed(iRow));
         weather_prediction.north_south.Set(iRow, local_blended_y.GetAltitude(iRow), local_blended_y.GetSpeed(iRow));

      } else {

         // Take winds from current altitude.

         weather_prediction.east_west.Set(iRow, currentAlt, Vwx_sensed);
         weather_prediction.north_south.Set(iRow, currentAlt, Vwy_sensed);
      }

   }

   if (currentAltIx == -1) {
      // Add current wind to end

      weather_prediction.east_west.Set(newMaxBound, currentAlt, Vwx_sensed);
      weather_prediction.north_south.Set(newMaxBound, currentAlt, Vwy_sensed);
   }


   // Sort before returning
   weather_prediction.east_west.AscendSort();
   weather_prediction.north_south.AscendSort();
}

void Wind::PopulatePredictedWindMatrices(const AircraftIntent &intent_in,
                                         const vector<Units::Length> &predicted_wind_altitudes_in,
                                         WeatherPrediction &weather_prediction) {
   double altitude_at_beginning_of_route_ft = Units::FeetLength(intent_in.GetFms().m_altitude[0]).value() + 1000;

   Units::FeetLength altitude_at_beginning_of_route(altitude_at_beginning_of_route_ft);
   Units::FeetLength altitude_at_end_of_route = intent_in.GetFms().m_altitude[intent_in.GetNumberOfWaypoints() - 1];

   std::vector<Units::FeetLength> end_point_altitudes;
   end_point_altitudes.push_back(altitude_at_beginning_of_route);
   end_point_altitudes.push_back(altitude_at_end_of_route);

   std::set<Units::Length> input_wind_altitudes = ValidateWindAltitudeInputs(predicted_wind_altitudes_in);

   Units::FeetLength adjusted_altitude_at_beginning_of_route =
         GetAdjustedStartPointAltitude(altitude_at_beginning_of_route);
   Units::FeetLength adjusted_altitude_at_end_of_route =
         GetAdjustedEndPointAltitude(altitude_at_end_of_route);

   std::set<Units::Length> wind_forecast_altitudes = AddRouteAltitudesToList(input_wind_altitudes,
                                                                             adjusted_altitude_at_end_of_route,
                                                                             adjusted_altitude_at_beginning_of_route);

   if (wind_forecast_altitudes.size() < 5) {
      AddIntermediateWindAltitudes(wind_forecast_altitudes);
   }

   int current_wind_index = 1;
   int minimum_wind_index = 1;
   int maximum_wind_index = static_cast<int>(wind_forecast_altitudes.size());

   weather_prediction.east_west.SetBounds(minimum_wind_index, maximum_wind_index);
   weather_prediction.north_south.SetBounds(minimum_wind_index, maximum_wind_index);

   AddSensedWindsToWindStack(intent_in.GetTangentPlaneSequence(), intent_in.GetFms(),
                             adjusted_altitude_at_beginning_of_route, wind_forecast_altitudes, weather_prediction,
                             current_wind_index);

   PredictedWindOption predicted_wind_option = weather_prediction.GetPredictedWindOption();

   if (predicted_wind_option == SINGLE_DTG || predicted_wind_option == MULTIPLE_DTG_ALONG_ROUTE) {
      CreatePredictionUsingCurrentWindOption(intent_in, adjusted_altitude_at_end_of_route,
                                             maximum_wind_index, wind_forecast_altitudes, current_wind_index,
                                             weather_prediction);
   } else if (predicted_wind_option == MULTIPLE_DTG_LEGACY) {
      CreatePredictionUsingLegacyWindOption(SINGLE_DTG, wind_forecast_altitudes, intent_in,
                                            altitude_at_beginning_of_route, current_wind_index, maximum_wind_index,
                                            weather_prediction);
   }

   weather_prediction.east_west.AscendSort();
   weather_prediction.north_south.AscendSort();
}

Units::FeetLength Wind::GetAdjustedStartPointAltitude(Units::FeetLength altitude_at_beginning_of_route) {
   int altitude_at_beginning_of_route_ft =
         static_cast<int>(std::round(Units::FeetLength(altitude_at_beginning_of_route).value()));
   if (altitude_at_beginning_of_route_ft % 100 == 0) {
      return altitude_at_beginning_of_route;
   } else {
      int rounded_altitude_at_beginning_of_route_ft = altitude_at_beginning_of_route_ft +
                                                      (100 - (altitude_at_beginning_of_route_ft % 100));

      std::stringstream ss;
      ss << "Route start point altitude " << altitude_at_beginning_of_route_ft <<
         " is not a multiple of 100. Adjusting to " << rounded_altitude_at_beginning_of_route_ft <<
         " to ensure that route is bounded by wind prediction.";
      LOG4CPLUS_ERROR(m_logger, ss.str());

      return Units::FeetLength(rounded_altitude_at_beginning_of_route_ft);
   }
}

Units::FeetLength Wind::GetAdjustedEndPointAltitude(Units::FeetLength altitude_at_end_of_route) {
   int altitude_at_end_of_route_ft = static_cast<int>(std::round(Units::FeetLength(altitude_at_end_of_route).value()));
   if (altitude_at_end_of_route_ft % 100 == 0) {
      return altitude_at_end_of_route;
   } else {
      int rounded_altitude_at_end_of_route_ft = altitude_at_end_of_route_ft - (altitude_at_end_of_route_ft % 100);

      std::stringstream ss;
      ss << "Route end point altitude " << altitude_at_end_of_route_ft << " is not a multiple of 100. Adjusting to " <<
         rounded_altitude_at_end_of_route_ft << " to ensure that route is bounded by wind prediction.";
      LOG4CPLUS_ERROR(m_logger, ss.str());

      return Units::FeetLength(rounded_altitude_at_end_of_route_ft);
   }
}

std::set<Units::Length> Wind::AddRouteAltitudesToList(const std::set<Units::Length> &wind_altitudes_in,
                                                      Units::FeetLength altitude_at_end_of_route,
                                                      Units::FeetLength altitude_at_beginning_of_route) {
   std::set<Units::Length> all_valid_altitudes = wind_altitudes_in;

   auto result = all_valid_altitudes.insert(altitude_at_end_of_route);
   if (!result.second) {
      std::stringstream ss;
      ss << "Route end point altitude " << altitude_at_end_of_route << " already in the list. Discarding "
                                                                       "this altitude.";
      LOG4CPLUS_ERROR(m_logger, ss.str());
   }

   result = all_valid_altitudes.insert(altitude_at_beginning_of_route);
   if (!result.second) {
      std::stringstream ss;
      ss << "Route start point altitude " << altitude_at_beginning_of_route << " already in the list. Discarding "
                                                                               "this altitude.";
      LOG4CPLUS_ERROR(m_logger, ss.str());
   }

   return all_valid_altitudes;
}

std::set<Units::Length> Wind::ValidateWindAltitudeInputs(const std::vector<Units::Length> &wind_altitudes_in) {
   std::set<Units::Length> valid_wind_prediction_altitudes;
   for (auto &predicted_wind_altitude : wind_altitudes_in) {
      int wind_altitude_ft = static_cast<int>(std::round(Units::FeetLength(predicted_wind_altitude).value()));

      if (wind_altitude_ft % 100 == 0) {
         auto result = valid_wind_prediction_altitudes.insert(Units::FeetLength(wind_altitude_ft));
         if (!result.second) {
            std::stringstream ss;
            ss << "Predicted wind altitude " << wind_altitude_ft << " already in the list. Discarding this altitude.";
            LOG4CPLUS_ERROR(m_logger, ss.str());
         }
      } else {
         std::stringstream ss;
         ss << "Predicted wind altitude " << wind_altitude_ft << " is not a multiple of 100. Discarding this altitude.";
         LOG4CPLUS_ERROR(m_logger, ss.str());
      }
   }

   if (valid_wind_prediction_altitudes.size() < 3) {
      string str = "Cannot create wind stack with less than 3 prediction altitudes";
      LOG4CPLUS_FATAL(m_logger, str);
   }

   return valid_wind_prediction_altitudes;
}

void Wind::AddSensedWindsToWindStack(const std::shared_ptr<TangentPlaneSequence> &tangent_plane_sequence,
                                     const AircraftIntent::RouteData &fms,
                                     const Units::FeetLength altitude_at_beginning_of_route,
                                     std::set<Units::Length> &forecast_wind_altitudes,
                                     WeatherPrediction &weather_prediction,
                                     int &current_wind_index) {
   Units::Length x_position = fms.m_x[0];
   Units::Length y_position = fms.m_y[0];

   WindStack x_true_wind(1, 5);
   WindStack y_true_wind(1, 5);

   EarthModel::GeodeticPosition geoPosition;

   // do lat/lon conversion only if using wind
   if (UseWind()) {
      EarthModel::LocalPositionEnu localPosition;
      localPosition.x = x_position;
      localPosition.y = y_position;
      localPosition.z = altitude_at_beginning_of_route;
      tangent_plane_sequence->convertLocalToGeodetic(localPosition, geoPosition);
   }


   m_wind_truth_instance->InterpolateTrueWind(geoPosition.latitude, geoPosition.longitude,
         altitude_at_beginning_of_route, x_true_wind, y_true_wind);
   Units::MetersPerSecondSpeed Vwx, Vwy;
   Units::HertzFrequency dVwx_dh, dVwy_dh;
   weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(altitude_at_beginning_of_route, x_true_wind, Vwx,
                                                                       dVwx_dh);
   weather_prediction.getAtmosphere()->CalculateWindGradientAtAltitude(altitude_at_beginning_of_route, y_true_wind, Vwy,
                                                                       dVwy_dh);

   weather_prediction.east_west.Set(current_wind_index, altitude_at_beginning_of_route, Vwx);
   weather_prediction.north_south.Set(current_wind_index, altitude_at_beginning_of_route, Vwy);
   ++current_wind_index;

   if (!forecast_wind_altitudes.erase(altitude_at_beginning_of_route)) {
      LOG4CPLUS_WARN(m_logger, "Didn't erase altitude at beginning of route ("
            << Units::FeetLength(altitude_at_beginning_of_route)
            << ") after loading sensed winds. Something may have gone wrong.");
   }
}

void Wind::CreatePredictionUsingCurrentWindOption(const AircraftIntent &aircraft_intent,
                                                  const Units::FeetLength altitude_at_end_of_route,
                                                  const int maximum_wind_index,
                                                  std::set<Units::Length> &forecast_wind_altitudes,
                                                  int current_wind_index_in,
                                                  WeatherPrediction &weather_prediction) {
   Units::NauticalMilesLength total_linear_route_length(0.0);
   int last_ix = aircraft_intent.GetNumberOfWaypoints() - 1;

   Units::FeetLength x_position = Units::MetersLength(aircraft_intent.GetFms().m_x[last_ix]);
   Units::FeetLength y_position = Units::MetersLength(aircraft_intent.GetFms().m_y[last_ix]);

   AddPredictedWindAtPtpToWindStack(aircraft_intent.GetTangentPlaneSequence(), x_position, y_position,
                                    altitude_at_end_of_route, forecast_wind_altitudes, weather_prediction,
                                    current_wind_index_in);

   for (int ix = (last_ix - 1); ix >= 0; ix--) {
      Units::Length x_next = Units::MetersLength(aircraft_intent.GetFms().m_x[ix]);
      Units::Length y_next = Units::MetersLength(aircraft_intent.GetFms().m_y[ix]);

      Units::Length distance_between_points = AircraftCalculations::PtToPtDist(x_position, y_position, x_next, y_next);
      Units::Length d_total_thru_next = total_linear_route_length + distance_between_points;

      if (abs(d_total_thru_next - SAMPLING_DISTANCE_FROM_END_OF_ROUTE) <
          abs(total_linear_route_length - SAMPLING_DISTANCE_FROM_END_OF_ROUTE)) {

         x_position = x_next;
         y_position = y_next;
         total_linear_route_length = d_total_thru_next;
      } else {
         break;
      }
   }

   int current_wind_index = current_wind_index_in;
   for (auto wind_altitude : forecast_wind_altitudes) {
      if (current_wind_index <= maximum_wind_index) {
         const bool prediction_needed_at_current_altitude =
               weather_prediction.north_south.GetAltitude(current_wind_index) == Units::Infinity() &&
               weather_prediction.east_west.GetAltitude(current_wind_index) == Units::Infinity();

         if (prediction_needed_at_current_altitude) {
            Units::KnotsSpeed wind_x, wind_y;
            InterpolateForecastWind(aircraft_intent.GetTangentPlaneSequence(), x_position, y_position, wind_altitude,
                                    wind_x, wind_y);
            weather_prediction.east_west.Set(current_wind_index, wind_altitude, wind_x);
            weather_prediction.north_south.Set(current_wind_index, wind_altitude, wind_y);
            ++current_wind_index;
         }
      }
   }
}

void Wind::AddPredictedWindAtPtpToWindStack(const std::shared_ptr<TangentPlaneSequence> &tangent_plane_sequence,
                                            const Units::FeetLength x_position,
                                            const Units::FeetLength y_position,
                                            const Units::FeetLength altitude_at_end_of_route,
                                            std::set<Units::Length> &forecast_wind_altitudes,
                                            WeatherPrediction &weather_prediction,
                                            int &current_wind_index) {
   Units::KnotsSpeed wind_x, wind_y;

   InterpolateForecastWind(tangent_plane_sequence, x_position, y_position, altitude_at_end_of_route, wind_x, wind_y);

   weather_prediction.east_west.Set(current_wind_index, altitude_at_end_of_route, wind_x);
   weather_prediction.north_south.Set(current_wind_index, altitude_at_end_of_route, wind_y);
   ++current_wind_index;

   if (!forecast_wind_altitudes.erase(altitude_at_end_of_route)) {
      LOG4CPLUS_WARN(m_logger, "Didn't erase altitude at end of route ("
            << Units::FeetLength(altitude_at_end_of_route) <<
            ") after loading PTP winds. Something may have gone wrong.");
   }
}


void Wind::CreatePredictionUsingLegacyWindOption(PredictedWindOption predicted_wind_option_in,
                                                 const std::set<Units::Length> &wind_altitudes,
                                                 const AircraftIntent &aircraft_intent,
                                                 const Units::Length altitude_at_beginning_of_route,
                                                 const int current_wind_index_in,
                                                 const int maximum_wind_index,
                                                 WeatherPrediction &weather_prediction) {
   double altitude_coefficient;
   Units::NauticalMilesLength distance_constant;
   std::vector<Units::NauticalMilesLength> dtg;

   PredictedWindOption predicted_wind_option = predicted_wind_option_in;
   int last_ix = aircraft_intent.GetNumberOfWaypoints() - 1;

   Wind::ValidatePredictedOptOne(aircraft_intent, predicted_wind_option, altitude_coefficient, distance_constant);

   if (predicted_wind_option != weather_prediction.GetPredictedWindOption()) {
      string msg = "Wind option 1 invalid!";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw std::logic_error(msg);
   }

   for (auto wind_altitude : wind_altitudes) {
      dtg.emplace_back(wind_altitude * altitude_coefficient + distance_constant);
   }

   int current_wind_index = current_wind_index_in;
   auto altitude_iter = wind_altitudes.begin();
   for (int jx = 0; jx < wind_altitudes.size(); jx++, ++altitude_iter) {
      Units::NauticalMilesLength total_dtg_from_end_of_route(0.0);

      Units::Length x_position = Units::MetersLength(aircraft_intent.GetFms().m_x[last_ix]);
      Units::Length y_position = Units::MetersLength(aircraft_intent.GetFms().m_y[last_ix]);

      for (int ix = (last_ix - 1); ix >= 0; ix--) {
         Units::FeetLength next_x_position = Units::MetersLength(aircraft_intent.GetFms().m_x[ix]);
         Units::FeetLength next_y_position = Units::MetersLength(aircraft_intent.GetFms().m_y[ix]);

         Units::FeetLength distance_between_points = AircraftCalculations::PtToPtDist(x_position, y_position,
                                                                                      next_x_position,
                                                                                      next_y_position);

         Units::NauticalMilesLength total_dtg_thru_next_point = total_dtg_from_end_of_route +
                                                                distance_between_points;

         if (abs(total_dtg_thru_next_point) < abs(dtg[jx])) {
            x_position = next_x_position;
            y_position = next_y_position;
            total_dtg_from_end_of_route = total_dtg_thru_next_point;
         } else {
            Units::NauticalMilesLength delta = dtg[jx] - total_dtg_from_end_of_route;

            Units::Angle crs = Units::arctan2(Units::FeetLength(next_y_position - y_position).value(),
                                              Units::FeetLength(next_x_position - x_position).value());

            x_position = x_position + (delta * cos(crs));
            y_position = y_position + (delta * sin(crs));
            break;
         }
      }

      Units::KnotsSpeed wind_speed_x;
      Units::KnotsSpeed wind_speed_y;

      Units::FeetLength current_altitude = *altitude_iter;
      if (altitude_at_beginning_of_route - current_altitude > Units::FeetLength(100.0)) {
         Wind::InterpolateForecastWind(aircraft_intent.GetTangentPlaneSequence(), x_position, y_position,
                                       current_altitude, wind_speed_x, wind_speed_y);

         weather_prediction.east_west.Set(current_wind_index, current_altitude, wind_speed_x);
         weather_prediction.north_south.Set(current_wind_index, current_altitude, wind_speed_y);
         ++current_wind_index;
      }
   }
}

void Wind::AddIntermediateWindAltitudes(std::set<Units::Length> &wind_altitudes_ft) {
   std::ostringstream oss;
   bool first = true;
   Units::FeetLength previous_altitude_ft = Units::FeetLength(0);

   for (auto wind_altitude : wind_altitudes_ft) {
      if (!first && wind_altitudes_ft.size() < 5) {
         int intermediate_altitude_ft =
               static_cast<int>(std::round((0.5 * Units::FeetLength(previous_altitude_ft + wind_altitude).value())));
         if (intermediate_altitude_ft % 100 != 0) {
            // The mod of the average of two altitudes that are multiples of 100 feet is either 50 or 0.
            intermediate_altitude_ft += 50;
         }

         wind_altitudes_ft.insert(Units::FeetLength(intermediate_altitude_ft));
         oss << "Not enough altitudes in the wind stack: adding " << Units::FeetLength(intermediate_altitude_ft)
             << " feet";
         string str = oss.str();
         oss.str("");
         LOG4CPLUS_WARN(m_logger, str);
      }
      first = false;
      previous_altitude_ft = wind_altitude;
   }
}

void Wind::ValidatePredictedOptOne(const AircraftIntent &aircraft_intent,
                                   PredictedWindOption &predicted_wind_option,
                                   double &altitude_coefficient,
                                   Units::Length &distance_constant) {

   // Validates whether predicated winds option one can be used based on the
   // the route in the intent waypoints.  Option one samples for Denver and
   // Phoenix.
   //
   // intent:Input intent containing route waypoints.
   // useOpt:Option to use.
   //        1-use option 1 if a sampling waypoint found in route.
   //        0-use default option.
   // altCoef:Output altitude coefficient -- was (nmi/ft),
   //    erroneously described as nmi/m, now unitless inverted slope (m/m)
   // distConst:Output distance constant (nmi).


   predicted_wind_option = SINGLE_DTG;

   string samplingWaypoints[] = {"EAGUL", "KOOLY", "KAILE", "KIPPR", "TSHNR", "WAHUU"};
   double altSamp[] = {0.00326316, 0.00380133, 0.00395813, 0.00363655, 0.00423597, 0.00362616};
   double distSamp[] = {5.36717, 11.8737, 23.951, 20.105, 28.6911, 19.8141};

   auto numSampWaypoints = (unsigned int) (sizeof(altSamp) / sizeof(altSamp[0]));

   bool is_valid = false;

   for (unsigned int ix = 0; (ix < aircraft_intent.GetNumberOfWaypoints()) && !is_valid; ix++) {
      for (unsigned int jx = 0; jx < numSampWaypoints; jx++) {
         if (samplingWaypoints[jx] == aircraft_intent.GetFms().m_name[ix]) {
            // We found validating waypoint-set outputs and terminate loops.

            predicted_wind_option = MULTIPLE_DTG_LEGACY;
            altitude_coefficient = altSamp[jx] * NAUTICAL_MILES_TO_METERS / FEET_TO_METERS;
            distance_constant = Units::NauticalMilesLength(distSamp[jx]);
            is_valid = true;
            break;
         }
      }
   }

   if (!is_valid) {
      string msg = "predicted_wind_opt 1 is invalid for this scenario.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw std::runtime_error(msg);
   }
}

/**
 * Interpolates wind into a pair of WindStack objects,
 * but only if m_use_wind is true.  Also includes temperature.
 */
void Wind::InterpolateTrueWind(const Units::Angle lat_in,
                               const Units::Angle lon_in,
                               const Units::Length altitude,
                               WindStack &east_west,
                               WindStack &north_south) {
   if (!m_use_wind) {
      for (int i = east_west.GetMinRow(); i <= east_west.GetMaxRow(); i++) {
         east_west.Set(i, Units::FeetLength((i - 1) * 1000), Units::KnotsSpeed(0));
      }

      for (int i = north_south.GetMinRow(); i <= north_south.GetMaxRow(); i++) {
         north_south.Set(i, Units::FeetLength((i - 1) * 1000), Units::KnotsSpeed(0));
      }
      return;
   }

   InterpolateWindMatrix(lat_in, lon_in, altitude, east_west, north_south);
}

void Wind::InterpolateForecastWind(const shared_ptr<TangentPlaneSequence> &tangentPlaneSequence,
                                   const Units::Length x_in,
                                   const Units::Length y_in,
                                   const Units::Length altitude,
                                   Units::Speed &east_west,
                                   Units::Speed &north_south) {
   if (!m_use_wind) {
      east_west = Units::KnotsSpeed(0.);
      north_south = Units::KnotsSpeed(0.);
      return;
   }

   Units::RadiansAngle lat(0.), lon(0.);

   EarthModel::LocalPositionEnu localPosition;
   localPosition.x = x_in;
   localPosition.y = y_in;
   localPosition.z = altitude;
   EarthModel::GeodeticPosition wpnt;
   tangentPlaneSequence->convertLocalToGeodetic(localPosition, wpnt);
   lat = wpnt.latitude;
   lon = wpnt.longitude;

   InterpolateWindScalar(lat, lon, altitude, east_west, north_south);
}

WeatherPrediction Wind::CreateZeroWindPrediction() {

   // Create zero winds data and provide that to the x and y values.
   WindStack zeroWinds(1, 5);
   zeroWinds.Set(1, Units::FeetLength(0.), Units::KnotsSpeed(0.));
   zeroWinds.Set(2, Units::FeetLength(10000.), Units::KnotsSpeed(0.));
   zeroWinds.Set(3, Units::FeetLength(20000.), Units::KnotsSpeed(0.));
   zeroWinds.Set(4, Units::FeetLength(30000.), Units::KnotsSpeed(0.));
   zeroWinds.Set(5, Units::FeetLength(50000.), Units::KnotsSpeed(0.));
   WeatherPrediction zeroWeather;
   zeroWeather.east_west = zeroWinds;
   zeroWeather.north_south = zeroWinds;

   return zeroWeather;
}
