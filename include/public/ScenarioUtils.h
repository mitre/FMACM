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
#include <map>

#include "public/RandomGenerator.h"

namespace aaesim::open_source {
class ScenarioUtils {
  public:
   ~ScenarioUtils() = default;

   static RandomGenerator RANDOM_NUMBER_GENERATOR;
   static const int AIRCRAFT_ID_NOT_IN_MAP;
   static void ClearAircraftIdMap() { m_aircraft_string_int_map.clear(); }
   static std::string GetAircraftIdForUniqueId(const int unique_id) {
      for (const auto &element : m_aircraft_string_int_map) {
         if (element.second == unique_id) return element.first;
      }
      return "";
   }
   static int GetUniqueIdForAircraftId(const std::string aircraft_id) {
      bool is_in_map = m_aircraft_string_int_map.find(aircraft_id) != m_aircraft_string_int_map.end();
      if (is_in_map) {
         return m_aircraft_string_int_map[aircraft_id];
      }
      return AIRCRAFT_ID_NOT_IN_MAP;
   }
   static int GenerateNewUniqueIdForAircraftId(const std::string aircraft_id) {
      int old_id = GetUniqueIdForAircraftId(aircraft_id);
      if (old_id == AIRCRAFT_ID_NOT_IN_MAP) {
         int new_id = m_aircraft_string_int_map.size();
         m_aircraft_string_int_map[aircraft_id] = new_id;
         return new_id;
      } else {
         return old_id;
      }
   }
   static std::string ResolveScenarioRootName(const std::string &full_scenario_filename) {
      std::string local_scenario_name{full_scenario_filename};

      // remove the leading directory structure if present (search for last instance of "/" or "\\")
      auto index = local_scenario_name.find_last_of("/\\");
      if (index != std::string::npos) {
         local_scenario_name = local_scenario_name.substr(index + 1);
      }

      index = local_scenario_name.find(".txt");
      if (index != std::string::npos) {
         local_scenario_name.erase(index, 4);
      }
      return local_scenario_name;
   }

  private:
   static std::map<std::string, int> m_aircraft_string_int_map;
   ScenarioUtils() = default;
};
}  // namespace aaesim::open_source