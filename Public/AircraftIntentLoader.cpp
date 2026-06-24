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

#include "public/AircraftIntentLoader.h"

#include <list>
#include <stdexcept>

#include "WaypointLoader.h"

namespace {
std::list<Waypoint> BuildWaypointList(const std::list<aaesim::loaders::WaypointLoader> &waypoint_loaders) {
   std::list<Waypoint> waypoints;
   for (const auto &waypoint_loader : waypoint_loaders) {
      waypoints.push_back(waypoint_loader.BuildWaypoint());
   }
   return waypoints;
}
}  // namespace

namespace aaesim {
namespace loaders {

bool AircraftIntentLoader::load(DecodedStream *input) {
   set_stream(input);

   std::list<WaypointLoader> descent_waypoint_loaders;
   std::list<WaypointLoader> ascent_waypoint_loaders, cruise_waypoint_loaders;
   double cruise_mach_loaded;
   Units::FeetLength cruise_alt_loaded(0);

   // register all the variables used by the Aircraft Intent
   register_var("planned_cruise_mach", &cruise_mach_loaded, true);
   register_var("planned_cruise_altitude", &cruise_alt_loaded, true);
   register_named_list("descent_waypoints", &descent_waypoint_loaders, false);
   register_named_list("cruise_waypoints", &cruise_waypoint_loaders, false);
   register_named_list("ascent_waypoints", &ascent_waypoint_loaders, false);

   // do the actual reading:
   aircraft_intent_.m_is_loaded = complete();

   aircraft_intent_.m_planned_cruise_altitude = cruise_alt_loaded;
   aircraft_intent_.planned_cruise_mach_ = cruise_mach_loaded;

   const auto descent_waypoints = BuildWaypointList(descent_waypoint_loaders);
   const auto ascent_waypoints = BuildWaypointList(ascent_waypoint_loaders);
   const auto cruise_waypoints = BuildWaypointList(cruise_waypoint_loaders);

   if (descent_waypoints.empty() && ascent_waypoints.empty() && cruise_waypoints.empty()) {
      LOG4CPLUS_ERROR(logger_,
                      "No waypoints were found in the scenario file. Check the aircraft_intent{} input block.");
      throw std::runtime_error("Must provide waypoints.");
   } else {
      aircraft_intent_.LoadWaypointsFromList(ascent_waypoints, cruise_waypoints, descent_waypoints);
   }

   return aircraft_intent_.m_is_loaded;
}

}  // namespace loaders
}  // namespace aaesim
