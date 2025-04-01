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
#include <fstream>
#include <scalar/Speed.h>
#include <nlohmann/json.hpp>
#include "public/LoggingLoadable.h"
#include "public/Logging.h"
#include "public/Waypoint.h"
#include "public/TangentPlaneSequence.h"
#include "utility/UtilityConstants.h"

class AircraftIntent : public LoggingLoadable {
  public:
   enum WaypointPhaseOfFlight { ASCENT, CRUISE, DESCENT };

   enum Arinc424LegType { UNSET = -1, IF, VI, TF, RF, CF, VA, CA };

   struct RouteData {
      std::vector<std::string> m_name;
      std::vector<WaypointPhaseOfFlight> m_waypoint_phase_of_flight;
      std::vector<Units::MetersLength> m_x;
      std::vector<Units::MetersLength> m_y;
      std::vector<Units::MetersLength> m_z;
      std::vector<Units::MetersLength> m_nominal_altitude;
      std::vector<Units::RadiansAngle> m_latitude;
      std::vector<Units::RadiansAngle> m_longitude;
      std::vector<Units::FeetPerSecondSpeed> m_nominal_ias;
      std::vector<Units::MetersLength> m_high_altitude_constraint;
      std::vector<Units::MetersLength> m_low_altitude_constraint;
      std::vector<Units::MetersPerSecondSpeed> m_high_speed_constraint;
      std::vector<Units::MetersPerSecondSpeed> m_low_speed_constraint;
      std::vector<Arinc424LegType> m_leg_type;
      std::vector<Units::MetersLength> m_x_rf_center;
      std::vector<Units::MetersLength> m_y_rf_center;
      std::vector<Units::MetersLength> m_rf_radius;
      std::vector<Units::SignedRadiansAngle> m_rf_latitude;
      std::vector<Units::SignedRadiansAngle> m_rf_longitude;
   };

   AircraftIntent();

   virtual ~AircraftIntent() = default;

   AircraftIntent(const AircraftIntent &in);

   AircraftIntent &operator=(const AircraftIntent &in);

   bool operator==(const AircraftIntent &obj) const;

   void Copy(const AircraftIntent &in);

   void Initialize();

   bool load(DecodedStream *input);

   virtual void LoadWaypointsFromList(const std::list<Waypoint> &ascent_waypoints,
                                      const std::list<Waypoint> &cruise_waypoints,
                                      const std::list<Waypoint> &descent_waypoints);

   void UpdateXYZFromLatLonWgs84();

   std::list<Waypoint> GetWaypointList() const;

   void GetLatLonFromXYZ(const Units::Length &xMeters, const Units::Length &yMeters, const Units::Length &zMeters,
                         Units::Angle &lat, Units::Angle &lon) const;

   void SetNumberOfWaypoints(unsigned int n);

   const Waypoint &GetWaypoint(unsigned int i) const;

   const std::string &GetWaypointName(unsigned int i) const;

   Units::MetersLength GetWaypointX(unsigned int i) const;

   Units::MetersLength GetWaypointY(unsigned int i) const;

   Units::MetersLength GetPlannedCruiseAltitude() const;

   void SetPlannedCruiseAltitude(Units::Length altitude);

   const RouteData &GetRouteData() const;

   int GetWaypointIndexByName(const std::string &waypoint_name) const;

   std::pair<int, int> FindCommonWaypoint(const AircraftIntent &intent) const;

   void InsertPairAtIndex(const std::string &wpname, const Units::Length &x, const Units::Length &y, const int index);

   void InsertWaypointAtIndex(const Waypoint &waypoint, const int index);

   virtual void UpdateWaypoint(const Waypoint &waypoint);

   virtual void ClearWaypoints();

   void SetId(int id_in);

   int GetId() const;

   unsigned int GetNumberOfWaypoints() const;

   bool IsLoaded() const;

   void Dump(std::ostream &fileOut) const;

   void DumpParms(std::string) const;

   bool ContainsAscentWaypoints() const;

   const std::vector<Waypoint> &GetAscentWaypoints() const;

   bool ContainsCruiseWaypoints() const;

   const std::vector<Waypoint> &GetCruiseWaypoints() const;

   bool ContainsDescentWaypoints() const;

   const std::vector<Waypoint> &GetDescentWaypoints() const;

   double GetPlannedCruiseMach() const;

   bool ContainsWaypointName(std::string waypoint_name) const;

   /**
    * @brief trim all waypoints after the named waypoint, returning a functional updated object
    *
    * @param aircraft_intent
    * @param waypoint_name
    * @return AircraftIntent
    */
   static AircraftIntent CopyAndTrimAfterNamedWaypoint(const AircraftIntent &aircraft_intent,
                                                       const std::string &waypoint_name);

  protected:
   std::list<Waypoint> AddConnectingLeg(const std::list<Waypoint> &first_waypoint_vector,
                                        const std::list<Waypoint> &second_waypoint_vector) const;

   static std::list<Waypoint> RemoveZeroLengthLegs(const std::list<Waypoint> &waypoints);

   static std::vector<Waypoint> ConvertListToVector(const std::list<Waypoint> &waypoint_list);

   static std::list<Waypoint> ConvertVectorToList(const std::vector<Waypoint> &waypoint_vector);

   void ClearAndResetRouteDataContent(const std::vector<Waypoint> &ascent_waypoints,
                                      const std::vector<Waypoint> &cruise_waypoints,
                                      const std::vector<Waypoint> &descent_waypoints);

   void DoRouteDataLogging() const;

   struct RouteData m_route_data;
   std::shared_ptr<TangentPlaneSequence> m_tangent_plane_sequence;
   double m_planned_cruise_mach;
   std::vector<Waypoint> m_all_waypoints;
   bool m_is_loaded;
   std::vector<Waypoint> m_ascent_waypoints, m_cruise_waypoints, m_descent_waypoints;
   Units::MetersLength m_planned_cruise_altitude;
   std::map<std::string, Arinc424LegType> m_arinc424_dictionary;

  private:
   static const int UNINITIALIZED_AIRCRAFT_ID;
   static log4cplus::Logger m_logger;

   void DeleteRouteDataContent();
   void AddWaypointsToRouteDataVectors(const std::vector<Waypoint> &waypoints, enum WaypointPhaseOfFlight add_as_phase);
   friend std::ostream &operator<<(std::ostream &out, const AircraftIntent &intent);
   int m_id;
};

inline const AircraftIntent::RouteData &AircraftIntent::GetRouteData() const { return m_route_data; }

inline int AircraftIntent::GetId() const { return m_id; }

inline void AircraftIntent::SetId(int id_in) { m_id = id_in; }

inline unsigned int AircraftIntent::GetNumberOfWaypoints() const { return m_route_data.m_name.size(); }

inline Units::MetersLength AircraftIntent::GetPlannedCruiseAltitude() const { return m_planned_cruise_altitude; }

inline bool AircraftIntent::IsLoaded() const { return m_is_loaded; }

inline std::list<Waypoint> AircraftIntent::GetWaypointList() const {
   std::list<Waypoint> output;
   std::copy(m_all_waypoints.begin(), m_all_waypoints.end(), std::back_inserter(output));
   return output;
}

inline std::vector<Waypoint> AircraftIntent::ConvertListToVector(const std::list<Waypoint> &waypoint_list) {
   std::vector<Waypoint> dest;
   std::copy(waypoint_list.begin(), waypoint_list.end(), std::back_inserter(dest));
   return dest;
}

inline std::list<Waypoint> AircraftIntent::ConvertVectorToList(const std::vector<Waypoint> &waypoint_vector) {
   std::list<Waypoint> dest;
   std::copy(waypoint_vector.begin(), waypoint_vector.end(), std::back_inserter(dest));
   return dest;
}

inline void AircraftIntent::AddWaypointsToRouteDataVectors(const std::vector<Waypoint> &waypoints,
                                                           enum WaypointPhaseOfFlight add_as_phase) {
   auto waypoint_itr = waypoints.begin();
   while (waypoint_itr != waypoints.end()) {
      m_all_waypoints.push_back(*waypoint_itr);
      m_route_data.m_name.push_back(waypoint_itr->GetName());
      m_route_data.m_waypoint_phase_of_flight.push_back(add_as_phase);
      m_route_data.m_nominal_altitude.emplace_back(waypoint_itr->GetAltitude());
      m_route_data.m_latitude.emplace_back(waypoint_itr->GetLatitude());
      m_route_data.m_longitude.emplace_back(waypoint_itr->GetLongitude());
      m_route_data.m_nominal_ias.emplace_back(waypoint_itr->GetNominalIas());
      m_route_data.m_leg_type.push_back(m_arinc424_dictionary[waypoint_itr->GetArinc424LegType()]);
      m_route_data.m_high_altitude_constraint.emplace_back(waypoint_itr->GetAltitudeConstraintHigh());
      m_route_data.m_low_altitude_constraint.emplace_back(waypoint_itr->GetAltitudeConstraintLow());
      m_route_data.m_high_speed_constraint.emplace_back(waypoint_itr->GetSpeedConstraintHigh());
      m_route_data.m_low_speed_constraint.emplace_back(waypoint_itr->GetSpeedConstraintLow());
      m_route_data.m_rf_latitude.emplace_back(waypoint_itr->GetRfTurnCenterLatitude());
      m_route_data.m_rf_longitude.emplace_back(waypoint_itr->GetRfTurnCenterLongitude());
      m_route_data.m_rf_radius.emplace_back(waypoint_itr->GetRfTurnArcRadius());

      ++waypoint_itr;
   }
}

inline std::list<Waypoint> AircraftIntent::AddConnectingLeg(const std::list<Waypoint> &first_waypoint_vector,
                                                            const std::list<Waypoint> &second_waypoint_vector) const {
   if (first_waypoint_vector.empty())
      // nothing to do
      return second_waypoint_vector;
   if (!second_waypoint_vector.empty() &&
       (first_waypoint_vector.back().GetName() == second_waypoint_vector.front().GetName())) {
      // nothing to do
      return second_waypoint_vector;
   }

   std::list<Waypoint> updated_waypoints;
   Waypoint to_copy(first_waypoint_vector.back());
   Waypoint new_tf_leg(to_copy.GetName() + "_copy_as_IF", to_copy.GetLatitude(), to_copy.GetLongitude(),
                       to_copy.GetAltitudeConstraintHigh(), to_copy.GetAltitudeConstraintLow(),
                       to_copy.GetSpeedConstraintHigh(), to_copy.GetAltitude(), to_copy.GetNominalIas(), "IF");
   updated_waypoints.push_back(new_tf_leg);
   std::copy(second_waypoint_vector.begin(), second_waypoint_vector.end(), std::back_inserter(updated_waypoints));
   return updated_waypoints;
}

std::ostream &operator<<(std::ostream &out, const AircraftIntent &intent);

inline bool AircraftIntent::ContainsAscentWaypoints() const { return !m_ascent_waypoints.empty(); }

inline bool AircraftIntent::ContainsCruiseWaypoints() const { return !m_cruise_waypoints.empty(); }

inline bool AircraftIntent::ContainsDescentWaypoints() const { return !m_descent_waypoints.empty(); }

inline double AircraftIntent::GetPlannedCruiseMach() const { return m_planned_cruise_mach; }

inline void AircraftIntent::DoRouteDataLogging() const {
   using json = nlohmann::json;
   if (m_logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
      for (auto idx = 0; idx < m_route_data.m_name.size(); ++idx) {
         json j;
         j["segment_index"] = idx;
         j["name"] = m_route_data.m_name[idx];
         j["phase_of_flight_int"] = m_route_data.m_waypoint_phase_of_flight[idx];
         j["x_position_m"] = Units::MetersLength(m_route_data.m_x[idx]).value();
         j["y_position_m"] = Units::MetersLength(m_route_data.m_y[idx]).value();
         j["z_position_m"] = Units::MetersLength(m_route_data.m_z[idx]).value();
         j["nominal_alitude_ft"] = Units::FeetLength(m_route_data.m_nominal_altitude[idx]).value();
         j["latitude_deg"] = Units::DegreesAngle(m_route_data.m_latitude[idx]).value();
         j["longitude_deg"] = Units::DegreesAngle(m_route_data.m_longitude[idx]).value();
         j["nominal_ias_kts"] = Units::KnotsSpeed(m_route_data.m_nominal_ias[idx]).value();
         j["alt_high_ft"] = Units::FeetLength(m_route_data.m_high_altitude_constraint[idx]).value();
         j["alt_low_ft"] = Units::FeetLength(m_route_data.m_low_altitude_constraint[idx]).value();
         j["speed_high_knots"] = Units::KnotsSpeed(m_route_data.m_high_speed_constraint[idx]).value();
         j["speed_low_knots"] = Units::KnotsSpeed(m_route_data.m_low_speed_constraint[idx]).value();
         j["leg_type_int"] = m_route_data.m_leg_type[idx];
         j["rf_turn_x_position_m"] = Units::MetersLength(m_route_data.m_x_rf_center[idx]).value();
         j["rf_turn_y_position_m"] = Units::MetersLength(m_route_data.m_y_rf_center[idx]).value();
         j["rf_turn_radius_nm"] = Units::NauticalMilesLength(m_route_data.m_rf_radius[idx]).value();
         j["rf_turn_lat_deg"] = Units::SignedDegreesAngle(m_route_data.m_rf_latitude[idx]).value();
         j["rf_turn_lon_deg"] = Units::SignedDegreesAngle(m_route_data.m_rf_longitude[idx]).value();
         LOG4CPLUS_TRACE(m_logger, j.dump());
      }
   }
}

inline bool AircraftIntent::ContainsWaypointName(std::string waypoint_name) const {
   auto name_comparator = [waypoint_name](const Waypoint &waypoint_to_test) {
      return waypoint_to_test.GetName().compare(waypoint_name) == 0;
   };
   return std::any_of(m_all_waypoints.rbegin(), m_all_waypoints.rend(), name_comparator);
}

inline const std::vector<Waypoint> &AircraftIntent::GetAscentWaypoints() const { return m_ascent_waypoints; }

inline const std::vector<Waypoint> &AircraftIntent::GetDescentWaypoints() const { return m_descent_waypoints; }

inline const std::vector<Waypoint> &AircraftIntent::GetCruiseWaypoints() const { return m_cruise_waypoints; }
