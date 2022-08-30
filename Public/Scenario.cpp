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

#include "public/Scenario.h"

using namespace std;

const int Scenario::AIRCRAFT_ID_NOT_IN_MAP = -1;
log4cplus::Logger Scenario::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("Scenario"));

map<string, int> Scenario::m_aircraft_string_int_map;
RandomGenerator Scenario::m_rand;
const Units::NauticalMilesLength Scenario::DEFAULT_ADS_B_RECEPTION_RANGE_THRESHOLD = Units::NauticalMilesLength(
      90.0); // constant range

Scenario::Scenario() {
   SetScenarioName("");
}

Scenario::~Scenario() {
}

bool Scenario::load(DecodedStream *input) {
   return true;
}

void Scenario::SetScenarioName(const string in) {
   m_scenario_name = in;

   // remove the leading directory structure if present (search for last instance of "/" or "\\")
   unsigned long index;
   index = m_scenario_name.find_last_of("/\\");
   if (index != string::npos) {
      m_scenario_name = m_scenario_name.substr(index + 1); // sets the string to after the last "/" or "\\"
   }

   // check for .txt and remove it if present
   index = m_scenario_name.find(".txt");
   if (index != string::npos) {
      // erases the .txt from the string
      m_scenario_name.erase(index, 4);
   }
}

void Scenario::DuplicateAcidCheck(const size_t aircraft_count) {
   size_t aircraft_id_count(m_aircraft_string_int_map.size());
   if (aircraft_count != aircraft_id_count) {
      string msg = "Scenario has " +
            std::to_string(aircraft_count) + " aircraft but " +
            std::to_string(aircraft_id_count) + " aircraft IDs.";
      LOG4CPLUS_FATAL(m_logger, msg);
      throw runtime_error(msg);
   }
}
