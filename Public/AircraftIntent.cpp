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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/AircraftIntent.h"
#include "public/AircraftCalculations.h"
#include "public/SingleTangentPlaneSequence.h"
#include <stdexcept>

using namespace std;

log4cplus::Logger AircraftIntent::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AircraftIntent"));

const int AircraftIntent::UNINITIALIZED_AIRCRAFT_ID = -1;

AircraftIntent::AircraftIntent()
      : m_number_of_waypoints(0),
        m_id(UNINITIALIZED_AIRCRAFT_ID),
        m_is_loaded(false) {
   m_waypoint_x[20] = {};
   m_waypoint_y[20] = {};
   m_waypoint_altitude[20] = {};
   m_waypoint_descent_angle[20] = {};
   m_waypoint_nominal_ias[20] = {};
   m_waypoint_mach[20] = {};
   m_waypoint_descent_rate[20] = {};

   Initialize();
}

// copy constructor for AircraftIntent
AircraftIntent::AircraftIntent(const AircraftIntent &in)
      : m_number_of_waypoints(0),
        m_id(UNINITIALIZED_AIRCRAFT_ID),
        m_is_loaded(false) {
   m_waypoint_x[20] = {};
   m_waypoint_y[20] = {};
   m_waypoint_altitude[20] = {};
   m_waypoint_descent_angle[20] = {};
   m_waypoint_nominal_ias[20] = {};
   m_waypoint_mach[20] = {};
   m_waypoint_descent_rate[20] = {};

   Initialize();

   Copy(in);
}


AircraftIntent::~AircraftIntent() {
}

void AircraftIntent::Initialize() {
   m_tangent_plane_sequence.reset();

   for (unsigned int c = 0; c < MAX_NUM_WAYPOINTS; ++c) {
      m_waypoint_name[c] = "";
      m_waypoint_altitude[c] = Units::ZERO_LENGTH;
      m_waypoint_descent_angle[c] = Units::ZERO_ANGLE;
      m_waypoint_nominal_ias[c] = Units::ZERO_SPEED;
      m_waypoint_mach[c] = 0;
      m_waypoint_descent_rate[c] = Units::Acceleration(0);

      m_fms.Name[c] = "";
      m_fms.AltWp[c] = Units::ZERO_LENGTH;
      m_fms.LatWp[c] = Units::ZERO_ANGLE;
      m_fms.LonWp[c] = Units::ZERO_ANGLE;
      m_fms.nominal_IAS_at_waypoint[c] = Units::ZERO_SPEED;
      m_fms.MACH_at_waypoint[c] = 0;

      // the new constraint values
      m_fms.altHi[c] = Units::ZERO_LENGTH;
      m_fms.altLow[c] = Units::ZERO_LENGTH;
      m_fms.speedHi[c] = Units::ZERO_SPEED;
      m_fms.speedLow[c] = Units::ZERO_SPEED;

      // RF Leg values
      m_fms.LatCp[c] = Units::ZERO_ANGLE;
      m_fms.LonCp[c] = Units::ZERO_ANGLE;
      m_fms.radiusCp[c] = Units::ZERO_LENGTH;
      m_fms.LatCp[c] = Units::ZERO_ANGLE;
      m_fms.LonCp[c] = Units::ZERO_ANGLE;
   }

   m_planned_cruise_altitude = Units::ZERO_LENGTH;
}

// assignment operator overload
AircraftIntent &AircraftIntent::operator=(const AircraftIntent &in) {
   Copy(in);

   return *this;
}

// call this after the load to finish things up 

void AircraftIntent::LoadWaypointsFromList(list<Waypoint> &waypoint_list) {
   int c = 0;
   list<Waypoint>::iterator i = waypoint_list.begin();
   list<Waypoint>::iterator e = waypoint_list.end();

   while (i != e) {
      assert((*i).GetName().size() < 16);

      m_waypoint_name[c] = (*i).GetName();

      // waypoint_x and waypoint_y are set after loading.
      m_waypoint_altitude[c] = (*i).GetAltitude();
      m_waypoint_descent_angle[c] = (*i).GetDescentAngle();
      m_waypoint_nominal_ias[c] = (*i).GetNominalIas();
      m_waypoint_mach[c] = (*i).GetMach();
      m_waypoint_descent_rate[c] = (*i).GetDescentRate();

      m_fms.Name[c] = (*i).GetName();
      m_fms.AltWp[c] = (*i).GetAltitude();
      m_fms.LatWp[c] = (*i).GetLatitude();
      m_fms.LonWp[c] = (*i).GetLongitude();
      m_fms.nominal_IAS_at_waypoint[c] = (*i).GetNominalIas();
      m_fms.MACH_at_waypoint[c] = (*i).GetMach();

      // the new constraint values
      m_fms.altHi[c] = (*i).GetAltitudeConstraintHigh();
      m_fms.altLow[c] = (*i).GetAltitudeConstraintLow();
      m_fms.speedHi[c] = (*i).GetSpeedConstraintHigh();
      m_fms.speedLow[c] = (*i).GetSpeedConstraintLow();

      // RF Leg values
      m_fms.LatCp[c] = (*i).GetRfTurnCenterLatitude();
      m_fms.LonCp[c] = (*i).GetRfTurnCenterLongitude();
      m_fms.radiusCp[c] = (*i).GetRfTurnArcRadius();

      //------
      //gwang 2009-09: handle NASA ASTAR route (waypoint format)
//        todo: commented this out, because I'm not sure it's really doing anything helpful. ALso doesn't check if it's
//       todo: off the end. handled up above.
//      if (fms.AltWp[c] == 0)
//      {
//         fms.AltWp[c] = fms.AltWp[c-1];
//      }
      //end gwang 2009-09

      //gwang 2009-09: handle the NASA ASTAR route (waypoint format)
      //If Mach is zero, it means there is no restriction.
      //It should be equal to the value of the Mach at the previous waypoint.
      //The logic is as follows:
      double mach = m_waypoint_mach[c];
      Units::FeetPerSecondSpeed ias = m_waypoint_nominal_ias[c];

      if (mach != 0) //This is for most cases. IAS should be the IAS at mach-IAS transition.
      {
         m_fms.nominal_IAS_at_waypoint[c] = m_mach_transition_cas;
         m_fms.MACH_at_waypoint[c] = m_waypoint_mach[c];
      } else if (c >= 1 && m_fms.MACH_at_waypoint[c - 1] != 0
                 && ias.value() == 0) {
         m_fms.nominal_IAS_at_waypoint[c] = m_mach_transition_cas;
         m_fms.MACH_at_waypoint[c] = m_fms.MACH_at_waypoint[c - 1];
      } else if (c >= 1 && ias.value() != 0) {
         m_fms.nominal_IAS_at_waypoint[c] = ias;
         m_fms.MACH_at_waypoint[c] = 0;
      } else if (c >= 1 && m_fms.nominal_IAS_at_waypoint[c - 1].value() != 0
                 && ias.value() == 0) {
         m_fms.nominal_IAS_at_waypoint[c] = m_fms.nominal_IAS_at_waypoint[c - 1];
         m_fms.MACH_at_waypoint[c] = 0;
      } else {
         m_fms.nominal_IAS_at_waypoint[c] = ias;
         m_fms.MACH_at_waypoint[c] = mach;
      }

      //------
      ++i;
      ++c;
   }


   m_number_of_waypoints = waypoint_list.size();
   m_waypoints = waypoint_list;

   // the waypoints are loaded. Initialize the earth model with them
   m_tangent_plane_sequence = shared_ptr<TangentPlaneSequence>(new SingleTangentPlaneSequence(waypoint_list));
}

void AircraftIntent::UpdateXYZFromLatLonWgs84() {
   // Call the earth model
   vector<EarthModel::LocalPositionEnu> localPositions;
   localPositions = m_tangent_plane_sequence->getLocalPositionsFromInitialization();
   EarthModel::GeodeticPosition geoPosition;
   EarthModel::LocalPositionEnu xyPosition;

   // Process the waypoints
   for (int var = m_number_of_waypoints - 1; var >= 0; --var) {
      // Waypoints
      geoPosition.altitude = Units::ZERO_LENGTH;
      geoPosition.latitude = Units::RadiansAngle(m_fms.LatWp[var]);
      geoPosition.longitude = Units::RadiansAngle(m_fms.LonWp[var]);
      m_tangent_plane_sequence->convertGeodeticToLocal(geoPosition, xyPosition);
      m_waypoint_x[var] = m_fms.xWp[var] = xyPosition.x; // yup, waypoint_x always holds the same value as fms.xWp
      m_waypoint_y[var] = m_fms.yWp[var] = xyPosition.y; // yup, waypoint_y always holds the same value as fms.yWp
      m_fms.zWp[var] = xyPosition.z;

      // Center points
      if (m_fms.radiusCp[var].value() == 0) {
         m_fms.xCp[var] = Units::ZERO_LENGTH;
         m_fms.yCp[var] = Units::ZERO_LENGTH;
      } else {
         geoPosition.altitude = Units::FeetLength(0);
         geoPosition.latitude = Units::RadiansAngle(m_fms.LatCp[var]);
         geoPosition.longitude = Units::RadiansAngle(m_fms.LonCp[var]);
         m_tangent_plane_sequence->convertGeodeticToLocal(geoPosition, xyPosition);
         m_fms.xCp[var] = xyPosition.x;
         m_fms.yCp[var] = xyPosition.y;
      }
   }
}

double AircraftIntent::CalcGreatCircleDist(double lat1,
                                           double lon1,
                                           double lat2,
                                           double lon2) {
   double d, dNM, dist;

   d = 2 * asin(sqrt(pow(sin((lat1 - lat2) / 2), 2.0) + cos(lat1) * cos(lat2) * pow(sin((lon1 - lon2) / 2), 2.0)));

   dNM = ((180.0 * 60.0) / PI) * d;

   dist = dNM * NAUTICAL_MILES_TO_METERS;

   return dist;
}

double AircraftIntent::CalcGreatCircleCrs(double lat1,
                                          double lon1,
                                          double lat2,
                                          double lon2,
                                          double dist) {
   double dNM, d, crs;

   dNM = dist / NAUTICAL_MILES_TO_METERS;

   d = dNM / ((180.0 * 60.0) / PI);

   if (sin(lon2 - lon1) < 0) {
      crs = acos((sin(lat2) - sin(lat1) * cos(d)) / (sin(d) * cos(lat1)));
   } else {
      double temp;
      temp = (sin(lat2) - sin(lat1) * cos(d)) / (sin(d) * cos(lat1));

      if (temp > 1.0) {
         temp = 1.0;
      }

      if (temp < -1.0) {
         temp = -1.0;
      }

      crs = 2 * PI - acos(temp);
   }

   // Convert course from great-circle convention to mathematical convention
   crs = crs - 3.0 * PI / 2.0;

   return crs;

}

int AircraftIntent::FindWaypointIx(std::string inwaypoint) const {

   // Finds index into waypoint list for input waypoint.
   //
   // inwaypoint:input waypoint to search for.
   // returns index into AircraftIntent waypoint list for
   //         input waypoint.  If waypoint not found,
   //         -1 returned.

   int ix = -1;

   for (int i = 0; ((i < m_number_of_waypoints) && (ix == -1)); ++i) {
      if (inwaypoint == m_waypoint_name[i]) {
         ix = i;
      }
   }

   return ix;

}

void AircraftIntent::Dump(ofstream &fileOut) const {
   fileOut << "------------" << endl;
   fileOut << "Intent of aircraft  " << m_id << ":" << endl;
   fileOut << "mach_transition_cas " << Units::KnotsSpeed(m_mach_transition_cas).value() << endl;


   for (unsigned int i = 0; i < m_number_of_waypoints; i++) {
      fileOut << "-----" << endl;
      fileOut << "Waypoint  " << i << ":" << endl;
      fileOut << "waypoint_name[i] " << m_waypoint_name[i] << endl;
      fileOut << "nominal_IAS_at_waypoint[i] " << m_waypoint_nominal_ias[i] << endl;
      fileOut << "MACH_at_waypoint[i] " << m_waypoint_mach[i] << endl;
      fileOut << "waypoint_Alt[i] " << m_waypoint_altitude[i].value() << endl;
      fileOut << "waypoint_Descent_angle_degree[i] " << m_waypoint_descent_angle[i] << endl;
      fileOut << "waypoint_Descent_rate_knot_per_second[i] " << m_waypoint_descent_rate[i] << endl;
      fileOut << "waypoint_x[i] " << m_waypoint_x[i].value() << endl;
      fileOut << "waypoint_y[i] " << m_waypoint_y[i].value() << endl;
//        fileOut << 	"Fms.Name[i] "   << fms.Name[i] << endl;
//        fileOut << 	"Fms.xWp[i] "   << fms.xWp[i]/FT_M << endl;
//        fileOut << 	"Fms.yWp[i] "   << fms.yWp[i]/FT_M << endl;
//        fileOut << 	"Fms.AltWp[i] "   << fms.AltWp[i]/FT_M << endl;
//        fileOut << 	"Fms.MACH_at_waypoint[i] "   << fms.MACH_at_waypoint[i] << endl;
//        fileOut << 	"Fms.nominal_IAS_at_waypoint[i] "   << fms.nominal_IAS_at_waypoint[i] << endl;
   }
}

int AircraftIntent::GetId() const {
   return m_id;
}

void AircraftIntent::SetId(int id_in) {
   m_id = id_in;
}

unsigned int AircraftIntent::GetNumberOfWaypoints() const {
   return m_number_of_waypoints;
}

void AircraftIntent::SetNumberOfWaypoints(unsigned int n) {
   m_number_of_waypoints = n;
}

const string &AircraftIntent::GetWaypointName(unsigned int i) const {
   return m_waypoint_name[i];
}

Units::MetersLength AircraftIntent::GetWaypointX(unsigned int i) const {
   return Units::MetersLength(m_waypoint_x[i]);
}

Units::MetersLength AircraftIntent::GetWaypointY(unsigned int i) const {
   return Units::MetersLength(m_waypoint_y[i]);
}

Units::MetersLength AircraftIntent::GetPlannedCruiseAltitude() const {
   return Units::MetersLength(m_planned_cruise_altitude);
}

// helper method for copy constructor and assignment operator
void AircraftIntent::Copy(const AircraftIntent &in) {
   m_id = in.m_id;
   m_planned_cruise_altitude = in.m_planned_cruise_altitude;
   m_number_of_waypoints = in.m_number_of_waypoints;
   m_is_loaded = in.m_is_loaded;

   // for loop to copy all waypoint information
   for (int loop = 0; loop < MAX_NUM_WAYPOINTS; loop++) {
      m_waypoint_name[loop] = in.m_waypoint_name[loop];
      m_waypoint_y[loop] = in.m_waypoint_y[loop];
      m_waypoint_x[loop] = in.m_waypoint_x[loop];
      m_waypoint_altitude[loop] = in.m_waypoint_altitude[loop];
      m_waypoint_descent_angle[loop] = in.m_waypoint_descent_angle[loop];
      m_waypoint_nominal_ias[loop] = in.m_waypoint_nominal_ias[loop];
      m_waypoint_mach[loop] = in.m_waypoint_mach[loop];
      m_waypoint_descent_rate[loop] = in.m_waypoint_descent_rate[loop];
   }
   m_mach_transition_cas = in.m_mach_transition_cas;

   // loop to copy FMS values
   for (int loop2 = 0; loop2 < MAX_NUM_WAYPOINTS; loop2++) {
      m_fms.Name[loop2] = in.m_fms.Name[loop2];
      m_fms.xWp[loop2] = in.m_fms.xWp[loop2];
      m_fms.yWp[loop2] = in.m_fms.yWp[loop2];
      m_fms.LatWp[loop2] = in.m_fms.LatWp[loop2];
      m_fms.LonWp[loop2] = in.m_fms.LonWp[loop2];
      m_fms.AltWp[loop2] = in.m_fms.AltWp[loop2];
      m_fms.nominal_IAS_at_waypoint[loop2] = in.m_fms.nominal_IAS_at_waypoint[loop2];
      m_fms.MACH_at_waypoint[loop2] = in.m_fms.MACH_at_waypoint[loop2];
      m_fms.altHi[loop2] = in.m_fms.altHi[loop2];
      m_fms.altLow[loop2] = in.m_fms.altLow[loop2];
      m_fms.speedHi[loop2] = in.m_fms.speedHi[loop2];
      m_fms.speedLow[loop2] = in.m_fms.speedLow[loop2];
      m_fms.LatCp[loop2] = in.m_fms.LatCp[loop2];
      m_fms.LonCp[loop2] = in.m_fms.LonCp[loop2];
      m_fms.xCp[loop2] = in.m_fms.xCp[loop2];
      m_fms.yCp[loop2] = in.m_fms.yCp[loop2];
      m_fms.radiusCp[loop2] = in.m_fms.radiusCp[loop2];
   }

   //Fms.number_of_waypoints = in.Fms.number_of_waypoints;
   m_tangent_plane_sequence = in.m_tangent_plane_sequence;
   m_waypoints = in.m_waypoints;
}

bool AircraftIntent::load(DecodedStream *input) {

   list<Waypoint> waypoint_list;

   set_stream(input);

   double tmp1;
   Units::FeetLength tmp2(0);

   Units::KnotsSpeed machtransitioncasload;

   // register all the variables used by the Aircraft Intent
   register_var("number_of_waypoints", &m_number_of_waypoints, true);
   register_var("MachTransitionCas", &machtransitioncasload, true); // used for kinematic vertical path prediction
   register_var("plannedCruiseMach", &tmp1, true); // used for kinematic vertical path prediction
   register_var("plannedCruiseAltitude", &tmp2, true); // used for kinematic vertical path prediction
   register_named_list("waypoints", &waypoint_list, true);

   //do the actual reading:
   m_is_loaded = complete();

   m_mach_transition_cas = machtransitioncasload;
   m_planned_cruise_altitude = tmp2;
   m_planned_cruise_mach = tmp1;

   // loads all the waypoint information from the list
   LoadWaypointsFromList(waypoint_list);

   return m_is_loaded;
}

double AircraftIntent::GetPlannedCruiseMach() const {
   return m_planned_cruise_mach;
}

void AircraftIntent::DumpParms(std::string str) const {

   // Dumps AircraftIntent objects.
   // To use this, logger properties level must be set to DEBUG.
   //
   // str:Header string for output.


   LOG4CPLUS_DEBUG(AircraftIntent::logger, endl << str.c_str() << endl);


   LOG4CPLUS_DEBUG(AircraftIntent::logger, "mach_transition_cas "
         << Units::KnotsSpeed(m_mach_transition_cas).value() << endl);

   LOG4CPLUS_DEBUG(AircraftIntent::logger,
                   "No waypoints " << m_number_of_waypoints << "  FMS no waypoints " << m_number_of_waypoints << endl
                                   << endl);

   for (unsigned int i = 0; i < m_number_of_waypoints; i++) {

      LOG4CPLUS_DEBUG(AircraftIntent::logger, "Waypoint " << i << m_waypoint_name[i].c_str() << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "y   x   " << m_waypoint_y[i] << "  " << m_waypoint_x[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "Alt     " << m_waypoint_altitude[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "des ang " << m_waypoint_descent_angle[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "nom IAS " << m_waypoint_nominal_ias[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "wpt mch " << m_waypoint_mach[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "des rte " << m_waypoint_descent_rate[i] << endl);

      LOG4CPLUS_DEBUG(AircraftIntent::logger, endl);

      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS name    " << m_fms.Name[i].c_str() << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS Lat Lon " << m_fms.LatWp[i] << "  " << m_fms.LonWp[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS y   x   " << m_fms.yWp[i] << "  " << m_fms.xWp[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS alt z   " << m_fms.AltWp[i] << "  " << m_fms.zWp[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger,
                      "FMS ias mch " << m_fms.nominal_IAS_at_waypoint[i] << "  " << m_fms.MACH_at_waypoint[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS alt h l " << m_fms.altHi[i] << "  " << m_fms.altLow[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS spd h l " << m_fms.speedHi[i] << "  " << m_fms.speedLow[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, endl << endl);

      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS CP Lat Lon " << m_fms.LatCp[i] << "  " << m_fms.LonCp[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS CP y   x   " << m_fms.yCp[i] << "  " << m_fms.xCp[i] << endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS CP radius   " << m_fms.radiusCp[i] << "  " << endl);
   }

}

void AircraftIntent::GetLatLonFromXYZ(const Units::Length &xMeters,
                                          const Units::Length &yMeters,
                                          const Units::Length &zMeters,
                                          Units::Angle &lat,
                                          Units::Angle &lon) {
   /*
    * Fms struct must have been initialized first
    */
   if (m_number_of_waypoints < 1) {
      throw logic_error("No waypoints in AircraftIntent");
   }

   // use the ellipsoidal model
   EarthModel::LocalPositionEnu localPos;
   localPos.x = xMeters;
   localPos.y = yMeters;
   localPos.z = zMeters;

   EarthModel::GeodeticPosition geo;
   m_tangent_plane_sequence->convertLocalToGeodetic(localPos, geo);
   lat = geo.latitude;
   lon = geo.longitude;

   return;
}

const shared_ptr<TangentPlaneSequence> &AircraftIntent::GetTangentPlaneSequence() const {
   return m_tangent_plane_sequence;
}

const struct AircraftIntent::RouteData &AircraftIntent::GetFms() const {
   return m_fms;
}

pair<int, int> AircraftIntent::FindCommonWaypoint(const AircraftIntent &intent) const {
   /*
    * Find the earliest common waypoint (closest to the start of own intent).
    *
    * Returns -1 if not found.
    */

   int ix = GetNumberOfWaypoints() - 1;
   int tx = intent.GetNumberOfWaypoints() - 1;
   int thisIndex = -1, thatIndex = -1;  // default. Will return this if no common waypoint found

   while ((ix >= 0) && (tx >= 0)) {
      if (GetWaypointName(ix).compare(intent.GetWaypointName(tx)) == 0) {
         // Still have a match from end.
         thisIndex = ix;
         thatIndex = tx;
         ix--;
         tx--;
      } else {
         break;
      } // Exit out of loop
   }

   return pair<int, int>(thisIndex, thatIndex);
}

void AircraftIntent::InsertPairAtIndex(const std::string &wpname,
                                       const Units::Length &x,
                                       const Units::Length &y,
                                       const int index) {
   /*
    * The incoming point must have been validated by the caller.
    *
    * The point will be converted to a waypoint and inserted into the waypoint list that this object
    * was initialized with. Then the object will be re-initialized.
    */
   Units::Angle lat, lon;
   GetLatLonFromXYZ(x, y, Units::ZERO_LENGTH, lat, lon);

   Waypoint wp;
   wp.SetRfTurnArcRadius(Units::ZERO_LENGTH);
   wp.SetWaypointLatLon(lat, lon);
   wp.SetName(wpname);

   // Insert new waypoint
   std::list<Waypoint>::iterator itr = std::next(m_waypoints.begin(), index); // get an iterator pointing to index
   m_waypoints.insert(itr, wp);  // insert BEFORE iterator

   // Re-initialize this object with the new waypoints
   LoadWaypointsFromList(m_waypoints);
   UpdateXYZFromLatLonWgs84();
}

std::ostream &operator<<(std::ostream &out,
                         const AircraftIntent &intent) {
   out << "aircraft_intent {" << std::endl;
   out << "number_of_waypoints " << intent.GetNumberOfWaypoints() << std::endl;
   out << "MachTransitionCas " << intent.GetMachTransCas().value() << std::endl;
   out << "plannedCruiseMach 0" << std::endl;   // not used or saved but required
   out << "plannedCruiseAltitude " << Units::FeetLength(intent.GetPlannedCruiseAltitude()).value() << std::endl;
   out << "waypoints {" << std::endl;
   out << intent.m_waypoints;
   out << "    }" << std::endl;
   out << "}" << std::endl;
   return out;
}
