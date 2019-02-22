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
#include "public/Wind.h"
#include "public/WindZero.h"
#include "public/AircraftCalculations.h"
#include "public/InternalObserver.h"
#include <stdexcept>

using namespace std;

log4cplus::Logger Wind::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("Wind"));

Wind::Wind() {}

Wind::~Wind() {}

shared_ptr<Wind> Wind::m_wind_truth_instance;

bool Wind::m_use_wind = false;

Units::Length Wind::m_blending_altitude_limit = Units::FeetLength(5000.0);

void Wind::SetWindTruthInstance(std::shared_ptr<Wind> truth_instance) {
   m_wind_truth_instance = truth_instance;
}

bool Wind::IsUseWind() {
   return m_use_wind;
}

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


   if (maxAlt > Units::FeetLength(FLIGHT_LEVEL_UPPER_BOUND * 1000.0)) {
      maxAlt = Units::FeetLength(FLIGHT_LEVEL_UPPER_BOUND * 1000.0);
   }

   Units::Length minAlt = (currentAlt - Wind::m_blending_altitude_limit);

   if (Units::FeetLength(minAlt) < Units::FeetLength(FLIGHT_LEVEL_LOWER_BOUND * 1000.0)) {
      minAlt = Units::FeetLength(FLIGHT_LEVEL_LOWER_BOUND * 1000.0);
   }

   // Loop over the predicted matrices. The altitude values between
   // the two should always be in-sync, so we can write one loop
   // to iterate over both.

   const Units::Speed Vwx_sensed = Units::MetersPerSecondSpeed(current_state.m_Vwx);
   const Units::Speed Vwy_sensed = Units::MetersPerSecondSpeed(current_state.m_Vwy);

   int iRow;

   Units::Length altFromPrediction;

   for (iRow = local_blended_x.get_max_row(); iRow >= 1; iRow--) {

      altFromPrediction = local_blended_x.getAltitude(
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

      const Units::Speed Vwx_predicted = local_blended_x.getSpeed(iRow);
      const Units::Speed Vwy_predicted = local_blended_y.getSpeed(iRow);
      const Units::Speed Vwx_update((weightValue * Vwx_sensed) + (unityMinusWeightValue * Vwx_predicted));
      const Units::Speed Vwy_update((weightValue * Vwy_sensed) + (unityMinusWeightValue * Vwy_predicted));
      local_blended_x.set(iRow, altFromPrediction, Vwx_update);
      local_blended_y.set(iRow, altFromPrediction, Vwy_update);
   }

   // Add the current location and sensed wind to the matrices also
   // The below loop is a very verbose way of updating the predicted wind matrix. However,
   // WindStack does not contain update/append operations. Remove the below when WindStack
   // is updated.

   // Get correct new bounds.

   int currentAltIx = -1;

   for (iRow = local_blended_x.get_min_row(); iRow <= local_blended_x.get_max_row() && currentAltIx == -1; iRow++) {
      if (abs(currentAlt - local_blended_x.getAltitude(iRow)) < Units::FeetLength(0.1)) {
         // Current altitude found in blended wind matrix.

         currentAltIx = iRow;
      }
   }

   const int newMaxBound = ((currentAltIx == -1) ? (local_blended_x.get_max_row() + 1) : local_blended_x.get_max_row());

   weather_prediction.east_west.setBounds(1, newMaxBound); // this will delete all data
   weather_prediction.north_south.setBounds(1, newMaxBound); // this will delete all data
   for (iRow = local_blended_x.get_min_row(); iRow <= local_blended_x.get_max_row(); iRow++) {

      if (iRow != currentAltIx) {

         // Take winds from blended matrix.

         weather_prediction.east_west.set(iRow, local_blended_x.getAltitude(iRow), local_blended_x.getSpeed(iRow));
         weather_prediction.north_south.set(iRow, local_blended_y.getAltitude(iRow), local_blended_y.getSpeed(iRow));

      } else {

         // Take winds from current altitude.

         weather_prediction.east_west.set(iRow, currentAlt, Vwx_sensed);
         weather_prediction.north_south.set(iRow, currentAlt, Vwy_sensed);
      }

   }

   if (currentAltIx == -1) {
      // Add current wind to end

      weather_prediction.east_west.set(newMaxBound, currentAlt, Vwx_sensed);
      weather_prediction.north_south.set(newMaxBound, currentAlt, Vwy_sensed);
   }


   // Sort before returning
   weather_prediction.east_west.ascendSort();
   weather_prediction.north_south.ascendSort();

   return;
}

void Wind::PopulatePredictedWindMatrices(
      const AircraftIntent &intent,
      const std::vector<Units::Length> &predicted_wind_altitudes,
      WeatherPrediction &weather_prediction) {

   // Populates wind matrices with predicted wind data.  Data is computed
   // at the initial aircraft position and altitude and for 30,000, 20,000, 10,000,
   // and ground altitude at the waypoint closest to 60 nmi from the initial waypoint.
   // An error message is output and the run is aborted if the altitude at the initial
   // point or altitude at the FAF point matches a descent altitude.  The FAF point is
   // defined as the last waypoint in the run scenario.
   //
   // intent:contains waypoint x,y,h information.
   // weatherPrediction:wind matrices containing altitude and velocity.

   // Get option to process predicted winds.

   PredictedWindOption useOpt = weather_prediction.getPredictedWindOption();

   // 0-default.
   // 1-first sampling option (Denver and Phoenix).

   double altCoef;
   Units::NauticalMilesLength distConst;

   set<Units::Length> descent_altitudes;
   for (auto it = predicted_wind_altitudes.begin(); it != predicted_wind_altitudes.end(); ++it) {
      descent_altitudes.insert(*it);
   }

   // descent_altitudes will be copied to wind_altitudes.  This allows us to check that
   // initial altitude and final altitude are not in descent_altitudes.  If either is
   // in the list, then InterpolateForecastWind will fail because there will be two
   // data points for the same altitude.
   set<Units::Length> wind_altitudes(descent_altitudes);
   ostringstream oss;

   if (useOpt == MULTIPLE_DTG_LEGACY) {
      Wind::ValidatePredictedOptOne(intent, useOpt, altCoef, distConst);
      if (useOpt != weather_prediction.getPredictedWindOption()) {
         string msg = "Wind option 1 invalid!";
         LOG4CPLUS_FATAL(m_logger, msg);
         throw logic_error(msg);
      }
   }

   int num_descents = descent_altitudes.size();

   if (num_descents < 3) {
      string str = "Cannot create wind stack with less than 3 altitudes in mPredictedAltitudes";
      LOG4CPLUS_FATAL(m_logger, str);
   }

   // if there are less than 5 descent_altitudes and init alt or final alt is in the list, we might not have
   // the necessary 5 altitudes.  In this case, add extra altitude half way between the existing altitudes to
   // bring the number up to (at least) 5.
   Units::MetersLength startAlt(intent.GetFms().AltWp[0]);
   Units::MetersLength endAlt(intent.GetFms().AltWp[intent.GetNumberOfWaypoints() - 1]);
   wind_altitudes.insert(startAlt);
   wind_altitudes.insert(endAlt);

   if (wind_altitudes.size() < 5) {
      // add altitudes between each of the incoming altitudes
      bool first = true;
      Units::Length prev;
      for (auto it = descent_altitudes.begin(); it != descent_altitudes.end(); ++it) {
         if (!first) {
            Units::Length alt = 0.5 * (prev + *it);
            wind_altitudes.insert(alt);
            oss << "Not enough altitudes in the wind stack: adding " << Units::FeetLength(alt).value() << " feet";
            string str = oss.str();
            oss.str("");
            LOG4CPLUS_WARN(m_logger, str);
         }
         first = false;
         prev = *it;
      }
   }

   int wind_count = wind_altitudes.size();
   weather_prediction.east_west.setBounds(1, wind_count);
   weather_prediction.north_south.setBounds(1, wind_count);

   // Initial point, initial altitude wind.

   Units::FeetLength x; // In feet.
   Units::FeetLength y; // In feet.
   Units::Length h;

   x = Units::MetersLength(intent.GetFms().xWp[0]); // In feet.
   y = Units::MetersLength(intent.GetFms().yWp[0]); // In feet.
   h = Units::MetersLength(intent.GetFms().AltWp[0]);


   // Compute wind and set matrices.  Use true wind at first point.

   WindStack trueWindX(1, 5);
   WindStack trueWindY(1, 5);

   m_wind_truth_instance->InterpolateTrueWind(intent.GetTangentPlaneSequence(),
                             x, y, h, trueWindX, trueWindY);

   Units::MetersPerSecondSpeed Vwx, Vwy;
   Units::HertzFrequency dVwx_dh, dVwy_dh;
   weather_prediction.getAtmosphere()->CalcWindGrad(h, trueWindX, Vwx, dVwx_dh);
   weather_prediction.getAtmosphere()->CalcWindGrad(h, trueWindY, Vwy, dVwy_dh);

   weather_prediction.east_west.set(1, h, Vwx);

   weather_prediction.north_south.set(1, h, Vwy);

   int last_ix = intent.GetNumberOfWaypoints() - 1;

   int ix_count = 2;

   if (useOpt == SINGLE_DTG || useOpt == MULTIPLE_DTG_ALONG_ROUTE) {

      // Selected waypoint, descent altitude winds.

      // Find point to compute descent points from.  The point is within a distance
      // of the FAF where the FAF is defined as the final waypoint.

      Units::NauticalMilesLength d_total(0.0); // Distance backwards through route from FAF summing
      // linear distance between each pair of points.  In nmi.

      x = Units::MetersLength(intent.GetFms().xWp[last_ix]); // In feet.
      y = Units::MetersLength(intent.GetFms().yWp[last_ix]); // In feet.
      h = Units::MetersLength(intent.GetFms().AltWp[last_ix]);

      for (int ix = (last_ix - 1); ix >= 0; ix--) {
         Units::Length x_next = Units::MetersLength(intent.GetFms().xWp[ix]); // In feet.
         Units::Length y_next = Units::MetersLength(intent.GetFms().yWp[ix]); // In feet.

         Units::Length d_between_pts = AircraftCalculations::PtToPtDist(x, y, x_next, y_next);

         Units::Length d_total_thru_next = d_total + d_between_pts; // In nmi.

         if (abs(d_total_thru_next - Units::NauticalMilesLength(DIST_FROM_FAF_NM))
             < abs(d_total - Units::NauticalMilesLength(DIST_FROM_FAF_NM))) {
            // Next point a better match.

            x = x_next; // In feet.
            y = y_next; // In feet.
            d_total = d_total_thru_next;
         } else {
            // Previous point is the point we want.

            break;
         }
      }

      // Compute wind and set matrices.

      //for (int ix=0;ix<wind_altitudes.size();ix++) {
      for (auto it = wind_altitudes.begin(); it != wind_altitudes.end(); ++it) {
         // skip altitudes that match startAlt or endAlt
         if ((startAlt == *it) ||
             (endAlt == *it)) {
            continue;
         }
         Units::KnotsSpeed wind_x, wind_y;
         InterpolateForecastWind(intent.GetTangentPlaneSequence(),
                                 x, y, *it, wind_x, wind_y);
         weather_prediction.east_west.set(ix_count, *it, wind_x);   // Altitude in feet.
         weather_prediction.north_south.set(ix_count, *it, wind_y);   // Altitude in feet.
         ix_count++;
      }

   } else if (useOpt == MULTIPLE_DTG_LEGACY) {
      // Compute distance to go for descent altitudes.

      vector<Units::NauticalMilesLength> dtg;

      for (auto it = wind_altitudes.begin(); it != wind_altitudes.end(); ++it) {
         dtg.push_back((*it) * altCoef + distConst);
      }

      // Compute winds for each descent altitude.

      auto it = wind_altitudes.begin();
      for (int jx = 0; jx < wind_altitudes.size(); jx++, ++it) {
         // Find x,y for winds interpolation

         Units::NauticalMilesLength d_total(0.0); // Distance backwards through route from FAF summing
         // linear distance between each pair of points.  In nmi.

         x = Units::MetersLength(intent.GetFms().xWp[last_ix]);  // feet.
         y = Units::MetersLength(intent.GetFms().yWp[last_ix]);  // feet.


         // Loop through waypoints to determine x,y

         for (int ix = (last_ix - 1); ix >= 0; ix--) {
            Units::FeetLength x_next = Units::MetersLength(intent.GetFms().xWp[ix]); // feet
            Units::FeetLength y_next = Units::MetersLength(intent.GetFms().yWp[ix]); // feet

            Units::FeetLength d_between_pts = AircraftCalculations::PtToPtDist(x, y, x_next, y_next);

            Units::NauticalMilesLength d_total_thru_next = d_total + d_between_pts; // nmi

            if (abs(d_total_thru_next) < abs(dtg[jx])) {
               // Setup for next point

               x = x_next; // feet
               y = y_next; // feet
               d_total = d_total_thru_next; // nmi
            } else {
               // Calculate x,y position from this waypoint to set predicted winds at this altitude.

               Units::NauticalMilesLength delta = dtg[jx] - d_total;

               Units::Angle crs = Units::arctan2(Units::FeetLength(y_next - y).value(),
                                                 Units::FeetLength(x_next - x).value());

               x = x + (delta * cos(crs));
               y = y + (delta * sin(crs));

               break;
            }
         }

         // Compute measured wind from computed x,y and set predicted winds for the descent altitude.
         Units::KnotsSpeed wind_x, wind_y;
         // skip altitudes that match startAlt or endAlt
         if ((startAlt == *it) || (endAlt == *it)) {
            continue;
         }

         Wind::InterpolateForecastWind(intent.GetTangentPlaneSequence(),
                                       x, y, *it, wind_x, wind_y);
         weather_prediction.east_west.set(ix_count, *it, wind_x); // altitude in feet.
         weather_prediction.north_south.set(ix_count, *it, wind_y); // altitude in feet.
         ix_count++;

      }

   }

   // Final point, final altitude wind.

   if (startAlt != endAlt) {
      x = Units::MetersLength(intent.GetFms().xWp[last_ix]);
      y = Units::MetersLength(intent.GetFms().yWp[last_ix]);
      h = Units::MetersLength(intent.GetFms().AltWp[last_ix]);

      // Compute wind and set matrices.
      Units::KnotsSpeed wind_x, wind_y;
      Wind::InterpolateForecastWind(intent.GetTangentPlaneSequence(),
                                    x, y, h, wind_x, wind_y);

      //int ix = num_descents + 2;

      // OLD FORMAT
      weather_prediction.east_west.set(ix_count, h, wind_x); // altitude in feet.
      weather_prediction.north_south.set(ix_count, h, wind_y); // wind_y speed in knots.

   }
   /* // SURFACE WINDS = 0
   weatherPrediction.east_west.set(ix,1,Units::FeetLength(h).value()-1800); // approximate Altitude of airport surface (in feet)
   weatherPrediction.east_west.set(ix,2,0); // assumed winds = 0 at surface
   weatherPrediction.north_south.set(ix,1,Units::FeetLength(h).value()-1800); // approximate Altitude of airport surface (in feet)
   weatherPrediction.north_south.set(ix,2,0); // assumed winds = 0 at surface */


   // Sort matrices

   weather_prediction.east_west.ascendSort();
   weather_prediction.north_south.ascendSort();

} // PopulatePredictedWindMatrices


void Wind::ValidatePredictedOptOne(const AircraftIntent &intent,
                                   PredictedWindOption &useOpt,
                                   double &altCoef,
                                   Units::Length &distConst) {

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


   useOpt = SINGLE_DTG; // default

   // Check if descent sampling valid.

   string samplingWaypoints[] = {"EAGUL", "KOOLY", "KAILE", "KIPPR", "TSHNR", "WAHUU"};
   double altSamp[] = {0.00326316, 0.00380133, 0.00395813, 0.00363655, 0.00423597, 0.00362616};
   double distSamp[] = {5.36717, 11.8737, 23.951, 20.105, 28.6911, 19.8141};

   unsigned int numSampWaypoints = (unsigned int) (sizeof(altSamp) / sizeof(altSamp[0]));

   bool isValid = false;

   for (unsigned int ix = 0; (ix < intent.GetNumberOfWaypoints()) && !isValid; ix++) {
      for (unsigned int jx = 0; jx < numSampWaypoints; jx++) {
         if (samplingWaypoints[jx] == intent.GetFms().Name[ix]) {
            // We found validating waypoint-set outputs and terminate loops.

            useOpt = MULTIPLE_DTG_LEGACY;
            altCoef = altSamp[jx] * NAUTICAL_MILES_TO_METERS / FEET_TO_METERS;
            distConst = Units::NauticalMilesLength(distSamp[jx]);
            isValid = true;
            break;
         }
      }
   }
   if (!isValid) {
      string msg = "predicted_wind_opt 1 is invalid for this scenario.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw runtime_error(msg);
   }
}

/**
 * Interpolates wind into a pair of WindStack objects,
 * but only if m_use_wind is true.
 */
void Wind::InterpolateTrueWind(const shared_ptr<TangentPlaneSequence> &tangentPlaneSequence,
                               const Units::Length x_in,
                               const Units::Length y_in,
                               const Units::Length altitude,
                               WindStack &east_west,
                               WindStack &north_south) {

   // TODO:Update this comment to correctly reflect the arguments.

   // lat_in: degrees
   // lon_in: degrees
   // x: feet
   // y: feet
   // altitude: feet
   // output: wind speeds (knots)
   // The return values are east_west and north_south. Both are two-dimensional WindStack: the first column contains the altitude (ft),
   // and the second column contains the wind speed (knots).
   if (!m_use_wind) {
      for (int i = east_west.get_min_row(); i <= east_west.get_max_row(); i++) {
         east_west.set(i, Units::FeetLength((i - 1) * 1000), Units::KnotsSpeed(0));
      }

      for (int i = north_south.get_min_row(); i <= north_south.get_max_row(); i++) {
         north_south.set(i, Units::FeetLength((i - 1) * 1000), Units::KnotsSpeed(0));
      }

      return;
   }

   Units::RadiansAngle lat_in(0), lon_in(0);

   EarthModel::LocalPositionEnu localPosition;
   localPosition.x = x_in;
   localPosition.y = y_in;
   localPosition.z = altitude;
   EarthModel::GeodeticPosition wpnt;
   tangentPlaneSequence->convertLocalToGeodetic(localPosition, wpnt);
   lat_in = wpnt.latitude;
   lon_in = wpnt.longitude;

   InterpolateWindMatrix(lat_in, lon_in, altitude, east_west, north_south);
}

void Wind::InterpolateForecastWind(const shared_ptr<TangentPlaneSequence> &tangentPlaneSequence,
                                   const Units::Length x_in,
                                   const Units::Length y_in,
                                   const Units::Length altitude,
                                   Units::Speed &east_west,
                                   Units::Speed &north_south) {

   // lat_in: degrees
   // lon_in: degrees
   // x: feet
   // y: feet
   // altitude
   // output: wind speeds (knots)
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

   /* std::cout << "(" << Units::DegreesAngle(lat) << ", " <<
         Units::DegreesAngle(lon) << ") " <<
         Units::FeetLength(altitude) << ":  (" <<
         Units::KnotsSpeed(east_west) << ", " <<
         Units::KnotsSpeed(north_south) << ")" << std::endl;
         */
}

void Wind::SetUseWind(const bool useWind) {
   m_use_wind = useWind;
}

void Wind::CheckWindIntegrity(const WindStack &wind) {

   // Checks integrity of wind matrix based on whether the entries have
   // undefined values, whether they are sorted by altitude (ascending),
   // and whether there are repeated altitudes and repeated lines.
   //
   // wind:wind matrix to check.


   // Undefined values.

   for (int i = wind.get_min_row(); i <= wind.get_max_row(); i++) {
      if (std::isnan(wind.getAltitude(i).value()) || std::isnan(wind.getSpeed(i).value())) {
         cout << "Undefined wind matrix value found" << endl;
         return;
      }
   }

   // Sort order.

   for (int i = wind.get_min_row() + 1; i <= wind.get_max_row(); i++) {
      if (wind.getAltitude(i) < wind.getAltitude(i - 1)) {
         cout << "Bad sort order in wind matrix found" << endl;
         return;
      }
   }

   // Repeated altitudes and lines.

   int numAltsRepeated = 0;
   int numLinesRepeated = 0;

   for (int i = wind.get_min_row() + 1; i <= wind.get_max_row(); i++) {
      if (wind.getAltitude(i) == wind.getAltitude(i - 1)) {
         numAltsRepeated++;

         if (wind.getSpeed(i) == wind.getSpeed(i - 1)) {
            numLinesRepeated++;
         }
      }
   }

   if (numAltsRepeated > 0) {
      cout << "Repeats found in wind matrix.  Alts " << numAltsRepeated
           << " lines " << numLinesRepeated << endl;
   }
}


void Wind::CheckWindIntegrity(const int id,
                              const string &str,
                              const WindStack &wind) {

   // Checks integrity of wind matrix based on whether the entries have
   // undefined values, whether they are sorted by altitude (ascending),
   // and whether there are repeated altitudes and repeated lines.
   //
   // id:aircraft id for output.
   // str:string for output.
   // wind:wind matrix to check.


   // Undefined values.
   for (int i = wind.get_min_row(); i <= wind.get_max_row(); i++) {
      if (std::isnan(wind.getAltitude(i).value()) || std::isnan(wind.getSpeed(i).value())) {
         cout << str << "  id " << id << "  Undefined wind matrix value found" << endl;
         return;
      }
   }

   // Sort order.
   for (int i = wind.get_min_row() + 1; i <= wind.get_max_row(); i++) {
      if (wind.getAltitude(i) < wind.getAltitude(i - 1)) {
         cout << str << "  id " << id << "  Bad sort order in wind matrix found" << endl;
         return;
      }
   }

   // Repeated altitudes and lines.

   int numAltsRepeated = 0;
   int numLinesRepeated = 0;
   for (int i = wind.get_min_row() + 1; i <= wind.get_max_row(); i++) {
      if (wind.getAltitude(i) == wind.getAltitude(i - 1)) {
         numAltsRepeated++;

         if (wind.getSpeed(i) == wind.getSpeed(i - 1)) {
            numLinesRepeated++;
         }
      }
   }

   if (numAltsRepeated > 0) {
      cout << str << "  id " << id << "  Repeats found in wind matrix.  Alts " << numAltsRepeated
           << " lines " << numLinesRepeated << endl;
   }
}

WeatherPrediction Wind::CreateZeroWindPrediction() {

   // Create zero winds data and provide that to the x and y values.
   WindStack zeroWinds(1, 5);
   zeroWinds.set(1, Units::FeetLength(0.), Units::KnotsSpeed(0.));
   zeroWinds.set(2, Units::FeetLength(10000.), Units::KnotsSpeed(0.));
   zeroWinds.set(3, Units::FeetLength(20000.), Units::KnotsSpeed(0.));
   zeroWinds.set(4, Units::FeetLength(30000.), Units::KnotsSpeed(0.));
   zeroWinds.set(5, Units::FeetLength(50000.), Units::KnotsSpeed(0.));
   WeatherPrediction zeroWeather;
   zeroWeather.east_west = zeroWinds;
   zeroWeather.north_south = zeroWinds;

   return zeroWeather;
}
