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

#include "public/AircraftIntent.h"

#include <stdexcept>

#include "math/InvalidIndexException.h"
#include "public/AircraftCalculations.h"
#include "public/SingleTangentPlaneSequence.h"

log4cplus::Logger AircraftIntent::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AircraftIntent"));

const int AircraftIntent::UNINITIALIZED_AIRCRAFT_ID = -1;

AircraftIntent::AircraftIntent()
      : m_fms(),
        m_tangent_plane_sequence(nullptr),
        m_waypoints(),
        m_number_of_waypoints(0),
        m_id(UNINITIALIZED_AIRCRAFT_ID),
        m_is_loaded(false) {
   Initialize();
}

AircraftIntent::AircraftIntent(const AircraftIntent &in)
      : m_fms(),
        m_tangent_plane_sequence(nullptr),
        m_waypoints(),
        m_number_of_waypoints(0),
        m_id(UNINITIALIZED_AIRCRAFT_ID),
        m_is_loaded(false) {
   Initialize();

   Copy(in);
}


AircraftIntent::~AircraftIntent() = default;

void AircraftIntent::Initialize() {
   m_waypoint_x[MAX_NUM_WAYPOINTS-1] = {};
   m_waypoint_y[MAX_NUM_WAYPOINTS-1] = {};
   m_waypoint_altitude[MAX_NUM_WAYPOINTS-1] = {};
   m_waypoint_descent_angle[MAX_NUM_WAYPOINTS-1] = {};
   m_waypoint_nominal_ias[MAX_NUM_WAYPOINTS-1] = {};
   m_waypoint_mach[MAX_NUM_WAYPOINTS-1] = {};
   m_waypoint_descent_rate[MAX_NUM_WAYPOINTS-1] = {};

   m_tangent_plane_sequence.reset();

   for (unsigned int c = 0; c < MAX_NUM_WAYPOINTS; ++c) {
      m_waypoint_name[c] = "";
      m_waypoint_altitude[c] = Units::ZERO_LENGTH;
      m_waypoint_descent_angle[c] = Units::ZERO_ANGLE;
      m_waypoint_nominal_ias[c] = Units::ZERO_SPEED;
      m_waypoint_mach[c] = 0;
      m_waypoint_descent_rate[c] = Units::Acceleration(0);

      m_fms.m_name[c] = "";
      m_fms.m_altitude[c] = Units::ZERO_LENGTH;
      m_fms.m_latitude[c] = Units::ZERO_ANGLE;
      m_fms.m_longitude[c] = Units::ZERO_ANGLE;
      m_fms.m_nominal_ias[c] = Units::ZERO_SPEED;
      m_fms.m_mach[c] = 0;

      // the new constraint values
      m_fms.m_high_altitude_constraint[c] = Units::ZERO_LENGTH;
      m_fms.m_low_altitude_constraint[c] = Units::ZERO_LENGTH;
      m_fms.m_high_speed_constraint[c] = Units::ZERO_SPEED;
      m_fms.m_low_speed_constraint[c] = Units::ZERO_SPEED;

      // RF Leg values
      m_fms.m_rf_latitude[c] = Units::ZERO_ANGLE;
      m_fms.m_rf_longitude[c] = Units::ZERO_ANGLE;
      m_fms.m_rf_radius[c] = Units::ZERO_LENGTH;
      m_fms.m_rf_latitude[c] = Units::ZERO_ANGLE;
      m_fms.m_rf_longitude[c] = Units::ZERO_ANGLE;
   }

   m_planned_cruise_altitude = Units::ZERO_LENGTH;
}

AircraftIntent &AircraftIntent::operator=(const AircraftIntent &in) {
   Copy(in);

   return *this;
}

void AircraftIntent::LoadWaypointsFromList(std::list<Waypoint> &waypoint_list) {
   int c = 0;
   auto i = waypoint_list.begin();
   auto e = waypoint_list.end();

   while (i != e) {
      assert((*i).GetName().size() < 16);

      m_waypoint_name[c] = (*i).GetName();

      // waypoint_x and waypoint_y are set after loading.
      m_waypoint_altitude[c] = (*i).GetAltitude();
      m_waypoint_descent_angle[c] = (*i).GetDescentAngle();
      m_waypoint_nominal_ias[c] = (*i).GetNominalIas();
      m_waypoint_mach[c] = (*i).GetMach();
      m_waypoint_descent_rate[c] = (*i).GetDescentRate();

      m_fms.m_name[c] = (*i).GetName();
      m_fms.m_altitude[c] = (*i).GetAltitude();
      m_fms.m_latitude[c] = (*i).GetLatitude();
      m_fms.m_longitude[c] = (*i).GetLongitude();
      m_fms.m_nominal_ias[c] = (*i).GetNominalIas();
      m_fms.m_mach[c] = (*i).GetMach();

      // the new constraint values
      m_fms.m_high_altitude_constraint[c] = (*i).GetAltitudeConstraintHigh();
      m_fms.m_low_altitude_constraint[c] = (*i).GetAltitudeConstraintLow();
      m_fms.m_high_speed_constraint[c] = (*i).GetSpeedConstraintHigh();
      m_fms.m_low_speed_constraint[c] = (*i).GetSpeedConstraintLow();

      // RF Leg values
      m_fms.m_rf_latitude[c] = (*i).GetRfTurnCenterLatitude();
      m_fms.m_rf_longitude[c] = (*i).GetRfTurnCenterLongitude();
      m_fms.m_rf_radius[c] = (*i).GetRfTurnArcRadius();

      double mach = m_waypoint_mach[c];
      Units::FeetPerSecondSpeed ias = m_waypoint_nominal_ias[c];

      if (mach != 0) {
         m_fms.m_nominal_ias[c] = m_mach_transition_cas;
         m_fms.m_mach[c] = m_waypoint_mach[c];
      } else if (c >= 1 && m_fms.m_mach[c - 1] != 0
                 && ias.value() == 0) {
         m_fms.m_nominal_ias[c] = m_mach_transition_cas;
         m_fms.m_mach[c] = m_fms.m_mach[c - 1];
      } else if (c >= 1 && ias.value() != 0) {
         m_fms.m_nominal_ias[c] = ias;
         m_fms.m_mach[c] = 0;
      } else if (c >= 1 && m_fms.m_nominal_ias[c - 1].value() != 0
                 && ias.value() == 0) {
         m_fms.m_nominal_ias[c] = m_fms.m_nominal_ias[c - 1];
         m_fms.m_mach[c] = 0;
      } else {
         m_fms.m_nominal_ias[c] = ias;
         m_fms.m_mach[c] = mach;
      }

      ++i;
      ++c;
   }

   m_number_of_waypoints = static_cast<unsigned int>(waypoint_list.size());
   m_waypoints = waypoint_list;

   m_tangent_plane_sequence = std::shared_ptr<TangentPlaneSequence>(new SingleTangentPlaneSequence(waypoint_list));
}

void AircraftIntent::UpdateXYZFromLatLonWgs84() {
   EarthModel::GeodeticPosition geoPosition;
   EarthModel::LocalPositionEnu xyPosition;

   for (int var = m_number_of_waypoints - 1; var >= 0; --var) {
      geoPosition.altitude = Units::ZERO_LENGTH;
      geoPosition.latitude = Units::RadiansAngle(m_fms.m_latitude[var]);
      geoPosition.longitude = Units::RadiansAngle(m_fms.m_longitude[var]);
      m_tangent_plane_sequence->convertGeodeticToLocal(geoPosition, xyPosition);
      m_waypoint_x[var] = m_fms.m_x[var] = xyPosition.x; // yup, waypoint_x always holds the same value as fms.xWp
      m_waypoint_y[var] = m_fms.m_y[var] = xyPosition.y; // yup, waypoint_y always holds the same value as fms.yWp
      m_fms.m_z[var] = xyPosition.z;

      if (m_fms.m_rf_radius[var].value() == 0) {
         m_fms.m_x_rf_center[var] = Units::ZERO_LENGTH;
         m_fms.m_y_rf_center[var] = Units::ZERO_LENGTH;
      } else {
         geoPosition.altitude = Units::FeetLength(0);
         geoPosition.latitude = Units::RadiansAngle(m_fms.m_rf_latitude[var]);
         geoPosition.longitude = Units::RadiansAngle(m_fms.m_rf_longitude[var]);
         m_tangent_plane_sequence->convertGeodeticToLocal(geoPosition, xyPosition);
         m_fms.m_x_rf_center[var] = xyPosition.x;
         m_fms.m_y_rf_center[var] = xyPosition.y;
      }
   }
}

int AircraftIntent::GetWaypointIndexByName(const std::string &waypoint_name) const {
   // returns index into AircraftIntent waypoint list for
   //         input waypoint.  If waypoint not found, -1 returned.

   int ix = -1;

   for (int i = 0; ((i < m_number_of_waypoints) && (ix == -1)); ++i) {
      if (waypoint_name == m_waypoint_name[i]) {
         ix = i;
      }
   }

   return ix;
}

void AircraftIntent::Dump(std::ofstream &fileOut) const {
   fileOut << "------------" << std::endl;
   fileOut << "Intent of aircraft  " << m_id << ":" << std::endl;
   fileOut << "mach_transition_cas " << Units::KnotsSpeed(m_mach_transition_cas).value() << std::endl;


   for (unsigned int i = 0; i < m_number_of_waypoints; i++) {
      fileOut << "-----" << std::endl;
      fileOut << "Waypoint  " << i << ":" << std::endl;
      fileOut << "waypoint_name[i] " << m_waypoint_name[i] << std::endl;
      fileOut << "nominal_IAS_at_waypoint[i] " << m_waypoint_nominal_ias[i] << std::endl;
      fileOut << "MACH_at_waypoint[i] " << m_waypoint_mach[i] << std::endl;
      fileOut << "waypoint_Alt[i] " << m_waypoint_altitude[i].value() << std::endl;
      fileOut << "waypoint_Descent_angle_degree[i] " << m_waypoint_descent_angle[i] << std::endl;
      fileOut << "waypoint_Descent_rate_knot_per_second[i] " << m_waypoint_descent_rate[i] << std::endl;
      fileOut << "waypoint_x[i] " << m_waypoint_x[i].value() << std::endl;
      fileOut << "waypoint_y[i] " << m_waypoint_y[i].value() << std::endl;
   }
}

void AircraftIntent::Copy(const AircraftIntent &in) {
   m_id = in.m_id;
   m_planned_cruise_altitude = in.m_planned_cruise_altitude;
   m_planned_cruise_mach = in.m_planned_cruise_mach;
   m_number_of_waypoints = in.m_number_of_waypoints;
   m_is_loaded = in.m_is_loaded;

   for (auto loop = 0; loop < MAX_NUM_WAYPOINTS; loop++) {
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

   for (auto loop2 = 0; loop2 < MAX_NUM_WAYPOINTS; loop2++) {
      m_fms.m_name[loop2] = in.m_fms.m_name[loop2];
      m_fms.m_x[loop2] = in.m_fms.m_x[loop2];
      m_fms.m_y[loop2] = in.m_fms.m_y[loop2];
      m_fms.m_latitude[loop2] = in.m_fms.m_latitude[loop2];
      m_fms.m_longitude[loop2] = in.m_fms.m_longitude[loop2];
      m_fms.m_altitude[loop2] = in.m_fms.m_altitude[loop2];
      m_fms.m_nominal_ias[loop2] = in.m_fms.m_nominal_ias[loop2];
      m_fms.m_mach[loop2] = in.m_fms.m_mach[loop2];
      m_fms.m_high_altitude_constraint[loop2] = in.m_fms.m_high_altitude_constraint[loop2];
      m_fms.m_low_altitude_constraint[loop2] = in.m_fms.m_low_altitude_constraint[loop2];
      m_fms.m_high_speed_constraint[loop2] = in.m_fms.m_high_speed_constraint[loop2];
      m_fms.m_low_speed_constraint[loop2] = in.m_fms.m_low_speed_constraint[loop2];
      m_fms.m_rf_latitude[loop2] = in.m_fms.m_rf_latitude[loop2];
      m_fms.m_rf_longitude[loop2] = in.m_fms.m_rf_longitude[loop2];
      m_fms.m_x_rf_center[loop2] = in.m_fms.m_x_rf_center[loop2];
      m_fms.m_y_rf_center[loop2] = in.m_fms.m_y_rf_center[loop2];
      m_fms.m_rf_radius[loop2] = in.m_fms.m_rf_radius[loop2];
   }

   m_tangent_plane_sequence = in.m_tangent_plane_sequence;
   m_waypoints = in.m_waypoints;
}

bool AircraftIntent::load(DecodedStream *input) {

   std::list<Waypoint> waypoint_list;

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

   LoadWaypointsFromList(waypoint_list);

   return m_is_loaded;
}

void AircraftIntent::DumpParms(std::string str) const {
   LOG4CPLUS_DEBUG(AircraftIntent::logger, std::endl << str.c_str() << std::endl);


   LOG4CPLUS_DEBUG(AircraftIntent::logger, "mach_transition_cas "
         << Units::KnotsSpeed(m_mach_transition_cas).value() << std::endl);

   LOG4CPLUS_DEBUG(AircraftIntent::logger,
                   "No waypoints " << m_number_of_waypoints << "  FMS no waypoints " << m_number_of_waypoints << std::endl
                                   << std::endl);

   for (unsigned int i = 0; i < m_number_of_waypoints; i++) {

      LOG4CPLUS_DEBUG(AircraftIntent::logger, "Waypoint " << i << m_waypoint_name[i].c_str() << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "y   x   " << m_waypoint_y[i] << "  " << m_waypoint_x[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "Alt     " << m_waypoint_altitude[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "des ang " << m_waypoint_descent_angle[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "nom IAS " << m_waypoint_nominal_ias[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "wpt mch " << m_waypoint_mach[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "des rte " << m_waypoint_descent_rate[i] << std::endl);

      LOG4CPLUS_DEBUG(AircraftIntent::logger, std::endl);

      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS name    " << m_fms.m_name[i].c_str() << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger,
                      "FMS Lat Lon " << m_fms.m_latitude[i] << "  " << m_fms.m_longitude[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS y   x   " << m_fms.m_y[i] << "  " << m_fms.m_x[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS alt z   " << m_fms.m_altitude[i] << "  " << m_fms.m_z[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger,
                      "FMS ias mch " << m_fms.m_nominal_ias[i] << "  " << m_fms.m_mach[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS alt h l " << m_fms.m_high_altitude_constraint[i] << "  "
                                                             << m_fms.m_low_altitude_constraint[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger,
                      "FMS spd h l " << m_fms.m_high_speed_constraint[i] << "  " << m_fms.m_low_speed_constraint[i]
                                     << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, std::endl << std::endl);

      LOG4CPLUS_DEBUG(AircraftIntent::logger,
                      "FMS CP Lat Lon " << m_fms.m_rf_latitude[i] << "  " << m_fms.m_rf_longitude[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger,
                      "FMS CP y   x   " << m_fms.m_y_rf_center[i] << "  " << m_fms.m_x_rf_center[i] << std::endl);
      LOG4CPLUS_DEBUG(AircraftIntent::logger, "FMS CP radius   " << m_fms.m_rf_radius[i] << "  " << std::endl);
   }

}

void AircraftIntent::GetLatLonFromXYZ(const Units::Length &xMeters,
                                      const Units::Length &yMeters,
                                      const Units::Length &zMeters,
                                      Units::Angle &lat,
                                      Units::Angle &lon) {

   // use the ellipsoidal model
   EarthModel::LocalPositionEnu localPos;
   localPos.x = xMeters;
   localPos.y = yMeters;
   localPos.z = zMeters;

   EarthModel::GeodeticPosition geo;
   m_tangent_plane_sequence->convertLocalToGeodetic(localPos, geo);
   lat = geo.latitude;
   lon = geo.longitude;
}

std::pair<int, int> AircraftIntent::FindCommonWaypoint(const AircraftIntent &intent) const {
   /*
    * Find the earliest common waypoint (closest to the start of own intent).
    *
    * Returns -1 if not found.
    */

   int ix = GetNumberOfWaypoints() - 1;
   int tx = intent.GetNumberOfWaypoints() - 1;
   int thisIndex = -1;
   int thatIndex = -1;

   while ((ix >= 0) && (tx >= 0)) {
      if (GetWaypointName(ix) == intent.GetWaypointName(tx)) {

         thisIndex = ix;
         thatIndex = tx;
         ix--;
         tx--;
      } else {
         break;
      }
   }

   return std::make_pair(thisIndex, thatIndex);
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

   InsertWaypointAtIndex(wp, index);
}



void AircraftIntent::InsertWaypointAtIndex(const Waypoint &wp, int index) {

   auto itr = std::next(m_waypoints.begin(), index);
   m_waypoints.insert(itr, wp);

   LoadWaypointsFromList(m_waypoints);
   UpdateXYZFromLatLonWgs84();
}


void AircraftIntent::ClearWaypoints() {
   m_waypoints.clear();

   // Re-initialize this object with no waypoints
   LoadWaypointsFromList(m_waypoints);
   UpdateXYZFromLatLonWgs84();
}

const Waypoint &AircraftIntent::GetWaypoint(unsigned int i) const {
   std::list<Waypoint>::const_iterator itr = std::next(m_waypoints.begin(), i); // get an iterator pointing to index
   return *itr;
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


void AircraftIntent::SetPlannedCruiseAltitude(Units::Length altitude) {
   this->m_planned_cruise_altitude = altitude;
}

const std::string &AircraftIntent::GetWaypointName(int i) const {
   if (i < 0 || i >= m_number_of_waypoints) {
      LOG4CPLUS_FATAL(logger, "Index out of range.");
      throw InvalidIndexException(i, 0, m_number_of_waypoints-1);
   }
   return m_waypoint_name[i];
}

Units::MetersLength AircraftIntent::GetWaypointX(int i) const {
   if (i < 0 || i >= m_number_of_waypoints) {
      LOG4CPLUS_FATAL(logger, "Index out of range.");
      throw InvalidIndexException(i, 0, m_number_of_waypoints-1);
   }
   return m_waypoint_x[i];
}

Units::MetersLength AircraftIntent::GetWaypointY(int i) const {
   if (i < 0 || i >= m_number_of_waypoints) {
      LOG4CPLUS_FATAL(logger, "Index out of range.");
      throw InvalidIndexException(i, 0, m_number_of_waypoints-1);
   }
   return m_waypoint_y[i];
}

