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
      std::string Name[MAX_NUM_WAYPOINTS];
      Units::MetersLength xWp[MAX_NUM_WAYPOINTS]; // meters
      Units::MetersLength yWp[MAX_NUM_WAYPOINTS]; // meters
      Units::MetersLength zWp[MAX_NUM_WAYPOINTS]; // meters
      Units::MetersLength AltWp[MAX_NUM_WAYPOINTS]; // meters
      Units::RadiansAngle LatWp[MAX_NUM_WAYPOINTS]; // radians
      Units::RadiansAngle LonWp[MAX_NUM_WAYPOINTS]; // radians

      // int number_of_waypoints;
      Units::FeetPerSecondSpeed nominal_IAS_at_waypoint[MAX_NUM_WAYPOINTS]; // feet/second
      Units::MetersLength altHi[MAX_NUM_WAYPOINTS]; // meters
      Units::MetersLength altLow[MAX_NUM_WAYPOINTS]; // meters
      Units::MetersPerSecondSpeed speedHi[MAX_NUM_WAYPOINTS]; // meters/second
      Units::MetersPerSecondSpeed speedLow[MAX_NUM_WAYPOINTS]; // meters/second
      double MACH_at_waypoint[MAX_NUM_WAYPOINTS];

      // Added for RF Leg
      Units::MetersLength xCp[MAX_NUM_WAYPOINTS]; // meters
      Units::MetersLength yCp[MAX_NUM_WAYPOINTS]; // meters
      Units::MetersLength radiusCp[MAX_NUM_WAYPOINTS]; // meters
      Units::RadiansAngle LatCp[MAX_NUM_WAYPOINTS]; // radians
      Units::RadiansAngle LonCp[MAX_NUM_WAYPOINTS]; // radians
   };

   AircraftIntent();

   virtual ~AircraftIntent();

   AircraftIntent(const AircraftIntent &in);

   AircraftIntent &operator=(const AircraftIntent &in);

   void Copy(const AircraftIntent &in);

   void Initialize();

   // load method inherited from Loadable
   bool load(DecodedStream *input);

   bool IsLoaded();

   // ------------------------------------------------------
   // stuff for loading
   void LoadWaypointsFromList(std::list<Waypoint> &waypoint_list); // call this after the laod to finish things up
   void UpdateXYZFromLatLonWgs84();

   double CalcGreatCircleDist(double Lat1,
                              double Lon1,
                              double Lat2,
                              double Lon2);

   double CalcGreatCircleCrs(double Lat1,
                             double Lon1,
                             double Lat2,
                             double Lon2,
                             double d);

   const std::shared_ptr<TangentPlaneSequence> &GetTangentPlaneSequence() const;

   // used in unittest
   void GetLatLonFromXYZ(const Units::Length &xMeters,
                             const Units::Length &yMeters,
                             const Units::Length &zMeters,
                             Units::Angle &lat,
                             Units::Angle &lon);

   // setters and getters
   void SetId(int id_in);

   int GetId() const;

   unsigned int GetNumberOfWaypoints() const;

   void SetNumberOfWaypoints(unsigned int n);

   const std::string &GetWaypointName(unsigned int i) const;

   Units::MetersLength GetWaypointX(unsigned int i) const;

   Units::MetersLength GetWaypointY(unsigned int i) const;

   Units::MetersLength GetPlannedCruiseAltitude() const;

   const struct RouteData &GetFms() const;

   void SetMachTransCas(const Units::Speed cas);

   const Units::KnotsSpeed GetMachTransCas() const;

   // misc
   int FindWaypointIx(std::string waypoint) const;

   void Dump(std::ofstream &fileOut) const;

   void DumpParms(std::string) const;

   std::pair<int, int> FindCommonWaypoint(const AircraftIntent &intent) const;

   void InsertPairAtIndex(const std::string &wpname,
                          const Units::Length &x,
                          const Units::Length &y,
                          const int index);

   double GetPlannedCruiseMach() const;

protected:
   std::shared_ptr<TangentPlaneSequence> m_tangent_plane_sequence;

   unsigned int m_number_of_waypoints;
   struct RouteData m_fms;

private:
   friend std::ostream &operator<<(std::ostream &out,
                                   const AircraftIntent &intent);

   std::list<Waypoint> m_waypoints;
   static log4cplus::Logger logger;

   std::string m_waypoint_name[MAX_NUM_WAYPOINTS];
   Units::MetersLength m_waypoint_y[MAX_NUM_WAYPOINTS]; // meters
   Units::MetersLength m_waypoint_x[MAX_NUM_WAYPOINTS]; // meters
   Units::MetersLength m_waypoint_altitude[MAX_NUM_WAYPOINTS]; // meters
   Units::DegreesAngle m_waypoint_descent_angle[MAX_NUM_WAYPOINTS]; //descent_angle (degrees)
   Units::FeetPerSecondSpeed m_waypoint_nominal_ias[MAX_NUM_WAYPOINTS]; // feet/second
   Units::KnotsPerSecondAcceleration m_waypoint_descent_rate[MAX_NUM_WAYPOINTS];
   double m_waypoint_mach[MAX_NUM_WAYPOINTS];

   Units::Speed m_mach_transition_cas;
   Units::MetersLength m_planned_cruise_altitude;
   double m_planned_cruise_mach;

   int m_id;
   bool m_is_loaded;
   static const int UNINITIALIZED_AIRCRAFT_ID;

   /**
    * Private because these implementations are no longer used. But, we still want the code around for historical purposes.
    */
   //void update_xy_from_latlon_stereographic();
   //void update_xy_from_latlon_greatcircle();
   //---end old implementations--------------------------------------

   /**
    * Private because these fields are only used locally and could
    * probably become local method variables if they are removed
    * from dump() and copy().
    */
//    double		waypoint_Alt[MAX_NUM_WAYPOINTS]; // meters
//    double		waypoint_Descent_angle_degree[MAX_NUM_WAYPOINTS]; //descent_angle (degrees)
//    double		nominal_IAS_at_waypoint[MAX_NUM_WAYPOINTS]; // feet/second
//    double		MACH_at_waypoint[MAX_NUM_WAYPOINTS];
//    double		waypoint_Descent_rate_knot_per_second[MAX_NUM_WAYPOINTS];
   //---end old fields-------------------------------------------------
};

inline bool AircraftIntent::IsLoaded() {
   return m_is_loaded;
}

inline void AircraftIntent::SetMachTransCas(const Units::Speed cas) {
   m_mach_transition_cas = cas;
}

inline const Units::KnotsSpeed AircraftIntent::GetMachTransCas() const {
   return m_mach_transition_cas;
}

std::ostream &operator<<(std::ostream &out,
                         const AircraftIntent &intent);
