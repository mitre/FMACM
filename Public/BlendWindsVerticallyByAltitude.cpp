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

#include "public/BlendWindsVerticallyByAltitude.h"

#include "public/WindStack.h"

using namespace aaesim::open_source;

void BlendWindsVerticallyByAltitude::BlendSensedWithPredicted(
      const aaesim::open_source::AircraftState &current_state,
      aaesim::open_source::WeatherPrediction &weather_prediction) {
   /*
    * Algorithm Description:
    * For all altitudes above and below the current_state altitude by less than
    * BLEND_HEIGHT, the incoming predicted wind velocities will be updated
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
   aaesim::open_source::WindStack local_blended_x = weather_prediction.east_west();
   aaesim::open_source::WindStack local_blended_y = weather_prediction.north_south();

   // Define the limits that we need to use for wind blending

   Units::Length currentAlt = Units::FeetLength(current_state.GetAltitudeMsl());
   Units::Length maxAlt = currentAlt + BLEND_HEIGHT;

   if (maxAlt > MAXIMUM_ALTITUDE_LIMIT) {
      maxAlt = MAXIMUM_ALTITUDE_LIMIT;
   }

   Units::Length minAlt = (currentAlt - BLEND_HEIGHT);

   if (Units::FeetLength(minAlt) < MINIMUM_ALTITUDE_LIMIT) {
      minAlt = MINIMUM_ALTITUDE_LIMIT;
   }

   // Loop over the predicted matrices. The altitude values between
   // the two should always be in-sync, so we can write one loop
   // to iterate over both.

   const Units::Speed Vwx_sensed = current_state.GetSensedWindEast();
   const Units::Speed Vwy_sensed = current_state.GetSensedWindNorth();

   int iRow;

   Units::Length altFromPrediction;

   for (iRow = local_blended_x.GetMaxRow(); iRow >= 1; --iRow) {

      altFromPrediction = local_blended_x.GetAltitude(iRow);  // this will go down in altitude from highest to lowest
                                                              // value stored in local_blended_x

      if (altFromPrediction > maxAlt || altFromPrediction < minAlt) {
         continue;
      }  // above max altitude or below minAlt, no need to blend winds

      // Blend winds
      // NOTE: local_blended_x,y store velocity in knots. The aircraft state object stores sensed wind in meters per
      // second. Unit conversions are important.

      Units::Length altDiff = currentAlt - altFromPrediction;

      const double weightValue = 1.0 - (abs(altDiff) / BLEND_HEIGHT);

      const double unityMinusWeightValue = 1.0 - weightValue;

      const Units::Speed Vwx_predicted = local_blended_x.GetSpeed(iRow);
      const Units::Speed Vwy_predicted = local_blended_y.GetSpeed(iRow);
      const Units::Speed Vwx_update((weightValue * Vwx_sensed) + (unityMinusWeightValue * Vwx_predicted));
      const Units::Speed Vwy_update((weightValue * Vwy_sensed) + (unityMinusWeightValue * Vwy_predicted));
      local_blended_x.Insert(iRow, altFromPrediction, Vwx_update);
      local_blended_y.Insert(iRow, altFromPrediction, Vwy_update);
   }

   // Add the current location and sensed wind to the matrices also
   // The below loop is a very verbose way of updating the predicted wind matrix. However,
   // WindStack does not contain update/append operations. Remove the below when WindStack
   // is updated.

   // Get correct new bounds.
   int currentAltIx = -1;
   for (iRow = local_blended_x.GetMinRow(); iRow <= local_blended_x.GetMaxRow() && currentAltIx == -1; ++iRow) {
      if (abs(currentAlt - local_blended_x.GetAltitude(iRow)) < Units::FeetLength(0.1)) {
         // Current altitude found in blended wind matrix.
         currentAltIx = iRow;
      }
   }

   const int newMaxBound = ((currentAltIx == -1) ? (local_blended_x.GetMaxRow() + 1) : local_blended_x.GetMaxRow());

   weather_prediction.east_west().SetBounds(1, newMaxBound);    // this will delete all data
   weather_prediction.north_south().SetBounds(1, newMaxBound);  // this will delete all data
   for (iRow = local_blended_x.GetMinRow(); iRow <= local_blended_x.GetMaxRow(); iRow++) {

      if (iRow != currentAltIx) {
         // Take winds from blended matrix.
         weather_prediction.east_west().Insert(iRow, local_blended_x.GetAltitude(iRow), local_blended_x.GetSpeed(iRow));
         weather_prediction.north_south().Insert(iRow, local_blended_y.GetAltitude(iRow),
                                                 local_blended_y.GetSpeed(iRow));

      } else {
         // Take winds from current altitude.
         weather_prediction.east_west().Insert(iRow, currentAlt, Vwx_sensed);
         weather_prediction.north_south().Insert(iRow, currentAlt, Vwy_sensed);
      }
   }

   if (currentAltIx == -1) {
      // Add current wind to end
      weather_prediction.east_west().Insert(newMaxBound, currentAlt, Vwx_sensed);
      weather_prediction.north_south().Insert(newMaxBound, currentAlt, Vwy_sensed);
   }

   // Sort before returning
   weather_prediction.east_west().SortAltitudesAscending();
   weather_prediction.north_south().SortAltitudesAscending();
}
