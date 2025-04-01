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

#pragma once

#include <string>
#include <vector>
#include <scalar/Time.h>

#include "public/Logging.h"
#include "public/SimulationTime.h"
#include "aaesim/AircraftEntity.h"
#include "aaesim/AircraftDataSimulationTimeWriter.h"
namespace interval_management {
class PrecalcWaypointWriter : public aaesim::AircraftDataSimulationTimeWriter {
  public:
   PrecalcWaypointWriter();

   // Writes out the captured data to the specified file.
   virtual void Finish();

   /* Gets the predicted trajectory data from the various sources and stores it
    * for writing.
    */
   void Gather(const int &iteration_number, const aaesim::open_source::SimulationTime &time,
               const aaesim::AircraftEntity &im_aircraft) override;

  private:
   struct WaypointData {
      enum WaypointDataSource {
         INVALID_SOURCE = -1,
         FMS_DESCENT = 0,
         IM_ALGO_OWNSHIP,
         IM_ALGO_TARGET,
         FMS_CLIMB,
      };

      WaypointData()
         : iteration_number(-1),
           simulation_time(Units::SecondsTime(-1.0)),
           acid(""),
           waypoint_data_source(WaypointData::INVALID_SOURCE),
           waypoint_name(),
           wp_index(0),
           x_meters(-INFINITY),
           y_meters(-INFINITY),
           lat_radians(-INFINITY),
           lon_radians(-INFINITY),
           leg_length_meters(0),
           course_radians(0),
           rf_center_lat_radians(-INFINITY),
           rf_center_lon_radians(-INFINITY),
           radius_meters(0),
           constraint_distance_meters(0),
           constraint_altitude_high_meters(0),
           constraint_altitude_low_meters(0),
           constraint_speed_high_mps(0),
           constraint_speed_low_mps(0){};

      int iteration_number;
      Units::Time simulation_time;
      std::string acid;

      WaypointDataSource waypoint_data_source;
      std::string waypoint_name;
      int wp_index;
      double x_meters;
      double y_meters;
      double lat_radians;
      double lon_radians;
      double leg_length_meters;
      double course_radians;
      double rf_center_lat_radians;
      double rf_center_lon_radians;
      double radius_meters;
      double constraint_distance_meters;
      double constraint_altitude_high_meters;
      double constraint_altitude_low_meters;
      double constraint_speed_high_mps;
      double constraint_speed_low_mps;
   };

   void StoreWaypointData(const unsigned int &iteration, const aaesim::open_source::SimulationTime &simulation_time,
                          const std::string &aircraft_id,
                          const std::vector<aaesim::Wgs84PrecalcWaypoint> &precalc_waypoints,
                          const WaypointData::WaypointDataSource &waypoint_data_source,
                          std::map<std::string, std::vector<WaypointData> > &waypoints_by_aircraft_id);

   void StoreWaypointData(const unsigned int &iteration, const aaesim::open_source::SimulationTime &simulation_time,
                          const std::string &aircraft_id, const std::vector<PrecalcWaypoint> &precalc_waypoints,
                          const WaypointData::WaypointDataSource &waypoint_data_source,
                          std::map<std::string, std::vector<WaypointData> > &waypoints_by_aircraft_id);

   // Vectors that store the predicted trajectory data from the various input sources.
   std::map<std::string, std::vector<WaypointData> > fms_waypoint_data;
   std::map<std::string, std::vector<WaypointData> > algorithm_waypoint_ownship;
   std::map<std::string, std::vector<WaypointData> > algorithm_waypoint_target;

   static log4cplus::Logger logger;
};
}  // namespace interval_management
