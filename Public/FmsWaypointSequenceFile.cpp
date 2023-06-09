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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/FmsWaypointSequenceFile.h"

using namespace aaesim::open_source;

log4cplus::Logger FmsWaypointSequenceFile::logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("FmsWaypointSequenceFile"));

FmsWaypointSequenceFile::FmsWaypointSequenceFile() : OutputHandler("", "_fms_waypoint_sequence.csv") {
   m_fms_waypoint_data.reserve(10000);
}

void FmsWaypointSequenceFile::Finish() {
   if (!m_fms_waypoint_data.empty()) {

      os.open(filename.c_str());

      if (!os.is_open()) {
         std::string error_msg = "Cannot open " + filename;
         LOG4CPLUS_FATAL(FmsWaypointSequenceFile::logger, error_msg);
         return;
      }

      os.set_delimiter(',', ",");

      // Header
      os << "iteration";
      os << "acid";
      os << "time_sec";
      os << "waypoint_name";
      os << NEWLINE;
      os.flush();

      // Important for outputting double data.
      os.get_ofstream().precision(12);

      for (const auto &ix : m_fms_waypoint_data) {
         os << ix.iteration_number;
         os << ix.acid;
         os << Units::SecondsTime(ix.simulation_time).value();
         os << ix.waypoint_name;
         os << NEWLINE;
         os.flush();
      }

      os.close();
   }

   m_fms_waypoint_data.clear();
   m_finished = true;
}

void FmsWaypointSequenceFile::Gather(const int iteration_number, const SimpleAircraft &aircraft) {
   for (const auto &pair : aircraft.GetFms()->GetSequencedWaypoints()) {
      auto time = pair.first;
      auto wgs84_waypoint = pair.second;

      FmsWaypointData waypoint_data;
      waypoint_data.iteration_number = iteration_number;
      waypoint_data.acid = aircraft.GetAircraftId();
      waypoint_data.simulation_time = time;
      waypoint_data.waypoint_name.assign(wgs84_waypoint.m_name);

      m_fms_waypoint_data.push_back(waypoint_data);
   }
}
