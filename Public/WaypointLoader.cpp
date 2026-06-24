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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "WaypointLoader.h"

#include <array>
#include <cstdlib>
#include <string>

namespace aaesim {
namespace loaders {

bool WaypointLoader::load(DecodedStream *input) {
   set_stream(input);

   constexpr int kColumnCountV4Schema = 13;
   constexpr int kColumnCountNoRfLegsPreV4Schema = 11;
   constexpr int kColumnCountCompletePreV4Schema = 14;

   std::string name;
   Units::Angle latitude{Units::zero()};
   Units::Angle longitude{Units::zero()};
   Units::Length altitude{Units::zero()};

   bool load_successful = load_datum(name);
   if (!load_successful) {
      LoggingLoadable::report_error("could not load waypoint_name");
   }

   load_successful = loadAngleDegrees(latitude);
   if (!load_successful) {
      LoggingLoadable::report_error("could not load waypoint_Latitude");
   }

   load_successful = loadAngleDegrees(longitude);
   if (!load_successful) {
      LoggingLoadable::report_error("could not load waypoint_Longitude");
   }

   load_successful = loadLengthFeet(altitude);
   if (!load_successful) {
      LoggingLoadable::report_error("could not load waypoint_altitude");
   }

   std::array<double, kColumnCountCompletePreV4Schema> uninterpreted_loaded_values{};
   for (int i = 4; i < kColumnCountV4Schema - 1; ++i) {
      load_successful = load_datum(uninterpreted_loaded_values[i]);
      if (!load_successful) {
         input->push_back();
         if (i == kColumnCountNoRfLegsPreV4Schema) {
            Waypoint loaded_waypoint(name, latitude, longitude, Units::FeetLength(uninterpreted_loaded_values[7]),
                                     Units::FeetLength(uninterpreted_loaded_values[8]),
                                     Units::KnotsSpeed(uninterpreted_loaded_values[9]), altitude,
                                     Units::KnotsSpeed(uninterpreted_loaded_values[5]));
            loaded_waypoint.SetSpeedConstraintLow(Units::KnotsSpeed(uninterpreted_loaded_values[10]));
            waypoint_ = loaded_waypoint;
            return true;
         }
         LoggingLoadable::report_error("could not load a waypoint parameter");
         return false;
      }
   }

   std::string uninterpreted_next_value;
   load_successful = load_datum(uninterpreted_next_value);
   if (!load_successful) {
      LoggingLoadable::report_error("could not load a waypoint parameter...reason unknown");
      return false;
   }

   const auto loaded_character_length = uninterpreted_next_value.size();
   constexpr std::string::size_type kLegTypeIdentifierSize = 2;
   if (loaded_character_length == kLegTypeIdentifierSize) {
      Waypoint loaded_waypoint(name, latitude, longitude, Units::FeetLength(uninterpreted_loaded_values[5]),
                               Units::FeetLength(uninterpreted_loaded_values[6]),
                               Units::KnotsSpeed(uninterpreted_loaded_values[7]), altitude,
                               Units::KnotsSpeed(uninterpreted_loaded_values[4]), uninterpreted_next_value);
      loaded_waypoint.SetSpeedConstraintLow(Units::KnotsSpeed(uninterpreted_loaded_values[8]));
      loaded_waypoint.SetRfTurnArcRadius(Units::NauticalMilesLength(uninterpreted_loaded_values[9]));
      loaded_waypoint.SetRfTurnCenterLatitude(Units::DegreesAngle(uninterpreted_loaded_values[10]));
      loaded_waypoint.SetRfTurnCenterLongitude(Units::DegreesAngle(uninterpreted_loaded_values[11]));
      waypoint_ = loaded_waypoint;
      return true;
   }

   Waypoint loaded_waypoint(name, latitude, longitude, Units::FeetLength(uninterpreted_loaded_values[7]),
                            Units::FeetLength(uninterpreted_loaded_values[8]),
                            Units::KnotsSpeed(uninterpreted_loaded_values[9]), altitude,
                            Units::KnotsSpeed(uninterpreted_loaded_values[5]), "UNSET");
   loaded_waypoint.SetSpeedConstraintLow(Units::KnotsSpeed(uninterpreted_loaded_values[10]));
   loaded_waypoint.SetRfTurnArcRadius(Units::NauticalMilesLength(uninterpreted_loaded_values[11]));
   loaded_waypoint.SetRfTurnCenterLatitude(Units::DegreesAngle(std::strtod(uninterpreted_next_value.c_str(), nullptr)));

   load_successful = load_datum(uninterpreted_loaded_values[13]);
   if (!load_successful) {
      LoggingLoadable::report_error("could not load a waypoint parameter");
      return false;
   }
   loaded_waypoint.SetRfTurnCenterLongitude(Units::DegreesAngle(uninterpreted_loaded_values[13]));
   waypoint_ = loaded_waypoint;
   return true;
}

}  // namespace loaders
}  // namespace aaesim
