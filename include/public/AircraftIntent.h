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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

class AircraftIntent;   // avoid dependency loop

#include <string>
#include "public/LoggingLoadable.h"
#include "utility/Logging.h"
#include "public/Waypoint.h"
#include "public/TangentPlaneSequence.h"
#include "utility/constants.h"
#include <fstream>
#include <Speed.h>

using namespace aaesim::constants;

class AircraftIntent : public LoggingLoadable
{
public:

   struct RouteData
   {
      std::string m_name[MAX_NUM_WAYPOINTS];
      Units::MetersLength m_x[MAX_NUM_WAYPOINTS];
      Units::MetersLength m_y[MAX_NUM_WAYPOINTS];
      Units::MetersLength m_z[MAX_NUM_WAYPOINTS];
      Units::MetersLength m_altitude[MAX_NUM_WAYPOINTS];
      Units::RadiansAngle m_latitude[MAX_NUM_WAYPOINTS];
      Units::RadiansAngle m_longitude[MAX_NUM_WAYPOINTS];

      Units::FeetPerSecondSpeed m_nominal_ias[MAX_NUM_WAYPOINTS];
      Units::MetersLength m_high_altitude_constraint[MAX_NUM_WAYPOINTS];
      Units::MetersLength m_low_altitude_constraint[MAX_NUM_WAYPOINTS];
      Units::MetersPerSecondSpeed m_high_speed_constraint[MAX_NUM_WAYPOINTS];
      Units::MetersPerSecondSpeed m_low_speed_constraint[MAX_NUM_WAYPOINTS];
      double m_mach[MAX_NUM_WAYPOINTS];

      // Added for RF Leg
      Units::MetersLength m_x_rf_center[MAX_NUM_WAYPOINTS];
      Units::MetersLength m_y_rf_center[MAX_NUM_WAYPOINTS];
      Units::MetersLength m_rf_radius[MAX_NUM_WAYPOINTS];
      Units::RadiansAngle m_rf_latitude[MAX_NUM_WAYPOINTS];
      Units::RadiansAngle m_rf_longitude[MAX_NUM_WAYPOINTS];
   };

   AircraftIntent();

   virtual ~AircraftIntent();

   AircraftIntent(const AircraftIntent &in);

   AircraftIntent &operator=(const AircraftIntent &in);

   void Copy(const AircraftIntent &in);

   void Initialize();

   bool load(DecodedStream *input);

   virtual void LoadWaypointsFromList(std::list<Waypoint> &waypoint_list);

   void UpdateXYZFromLatLonWgs84();

   void GetLatLonFromXYZ(const Units::Length &xMeters,
                         const Units::Length &yMeters,
                         const Units::Length &zMeters,
                         Units::Angle &lat,
                         Units::Angle &lon);

   void SetNumberOfWaypoints(unsigned int n);

   void SetWaypoints(std::list<Waypoint>& waypoint_list);

   const Waypoint &GetWaypoint(unsigned int i) const;

   const std::string &GetWaypointName(int i) const;

   Units::MetersLength GetWaypointX(int i) const;

   Units::MetersLength GetWaypointY(int i) const;

   Units::MetersLength GetPlannedCruiseAltitude() const;

   void SetPlannedCruiseAltitude(Units::Length altitude);

   const struct RouteData &GetFms() const;

   void SetMachTransCas(const Units::Speed cas);

   const Units::KnotsSpeed GetMachTransCas() const;

   int GetWaypointIndexByName(const std::string &waypoint_name) const;


   std::pair<int, int> FindCommonWaypoint(const AircraftIntent &intent) const;

   void InsertPairAtIndex(const std::string &wpname,
                          const Units::Length &x,
                          const Units::Length &y,
                          const int index);

   void InsertWaypointAtIndex(const Waypoint &waypoint, const int index);

   virtual void UpdateWaypoint(const Waypoint &waypoint);

   virtual void ClearWaypoints();

   const std::shared_ptr<TangentPlaneSequence> &GetTangentPlaneSequence() const;

   double GetPlannedCruiseMach() const;

   void SetId(int id_in);

   int GetId() const;

   unsigned int GetNumberOfWaypoints() const;

   bool IsLoaded();

   void Dump(std::ostream &fileOut) const;

   void DumpParms(std::string) const;

protected:
   struct RouteData m_fms;
   std::shared_ptr<TangentPlaneSequence> m_tangent_plane_sequence;

   std::string m_waypoint_name[MAX_NUM_WAYPOINTS];
   Units::MetersLength m_waypoint_y[MAX_NUM_WAYPOINTS];
   Units::MetersLength m_waypoint_x[MAX_NUM_WAYPOINTS];
   Units::MetersLength m_waypoint_altitude[MAX_NUM_WAYPOINTS];
   Units::DegreesAngle m_waypoint_descent_angle[MAX_NUM_WAYPOINTS];
   Units::FeetPerSecondSpeed m_waypoint_nominal_ias[MAX_NUM_WAYPOINTS];
   Units::KnotsPerSecondAcceleration m_waypoint_descent_rate[MAX_NUM_WAYPOINTS];
   double m_waypoint_mach[MAX_NUM_WAYPOINTS];
   Units::Speed m_mach_transition_cas;
   Units::MetersLength m_planned_cruise_altitude;
   double m_planned_cruise_mach;
   std::list<Waypoint> m_waypoints;
   unsigned int m_number_of_waypoints;

private:
   friend std::ostream &operator<<(std::ostream &out,
                                   const AircraftIntent &intent);

   int m_id;
   bool m_is_loaded;

   static const int UNINITIALIZED_AIRCRAFT_ID;

   static log4cplus::Logger logger;

   void ResetWaypoints(std::list<Waypoint> &waypoint_list);

};

inline const std::shared_ptr<TangentPlaneSequence> &AircraftIntent::GetTangentPlaneSequence() const {
   return m_tangent_plane_sequence;
}

inline const struct AircraftIntent::RouteData &AircraftIntent::GetFms() const {
   return m_fms;
}

inline int AircraftIntent::GetId() const {
   return m_id;
}

inline void AircraftIntent::SetId(int id_in) {
   m_id = id_in;
}

inline void AircraftIntent::SetWaypoints(std::list<Waypoint>& waypoint_list) {
   m_waypoints = waypoint_list;
}

inline unsigned int AircraftIntent::GetNumberOfWaypoints() const {
   return m_number_of_waypoints;
}

inline Units::MetersLength AircraftIntent::GetPlannedCruiseAltitude() const {
   return m_planned_cruise_altitude;
}

inline bool AircraftIntent::IsLoaded() {
   return m_is_loaded;
}

inline void AircraftIntent::SetMachTransCas(const Units::Speed cas) {
   m_mach_transition_cas = cas;
}

inline const Units::KnotsSpeed AircraftIntent::GetMachTransCas() const {
   return m_mach_transition_cas;
}

inline double AircraftIntent::GetPlannedCruiseMach() const {
   return m_planned_cruise_mach;
}

std::ostream &operator<<(std::ostream &out,
                         const AircraftIntent &intent);
