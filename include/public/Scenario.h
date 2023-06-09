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

#pragma once

#include "public/LoggingLoadable.h"
#include "public/WeatherTruth.h"
#include <string>
#include "math/RandomGenerator.h"
#include "utility/Logging.h"

/**
 * This is a very basic Scenario implementation. Expanded implementations should not edit this class but inherit from
 * it.
 */
class Scenario : public LoggingLoadable {
  public:
   static const Units::NauticalMilesLength DEFAULT_ADS_B_RECEPTION_RANGE_THRESHOLD;

   Scenario();

   virtual ~Scenario();

   virtual void ProcessOneScenario() = 0;

   virtual bool load(DecodedStream *input);

   virtual const std::string &GetScenarioName();

   virtual void SetScenarioName(const std::string &in);

   void DuplicateAcidCheck(const size_t aircraft_count);

   static int GetUniqueIdForAircraftId(const std::string aircraft_id) {
      // Note: implementation intentionally here in the header file because static method
      bool is_in_map = m_aircraft_string_int_map.find(aircraft_id) != m_aircraft_string_int_map.end();
      if (is_in_map) {
         return m_aircraft_string_int_map[aircraft_id];
      }
      return AIRCRAFT_ID_NOT_IN_MAP;  // not in map, return AIRCRAFT_ID_NOT_IN_MAP
   }

   static int GenerateNewUniqueIdForAircraftId(const std::string aircraft_id) {
      // Note: implementation intentionally here in the header file because static method
      int oldId = GetUniqueIdForAircraftId(aircraft_id);
      if (oldId == AIRCRAFT_ID_NOT_IN_MAP) {
         int newid = m_aircraft_string_int_map.size();  // zero-based, so the first aircraft will have uniqueid zero
         m_aircraft_string_int_map[aircraft_id] = newid;
         return newid;
      } else {
         return oldId;
      }
   }

   static void ClearAircraftIdMap() { m_aircraft_string_int_map.clear(); }

   static const int AIRCRAFT_ID_NOT_IN_MAP;

   static RandomGenerator m_rand;

  protected:
   WeatherTruth m_weather;

  private:
   static log4cplus::Logger m_logger;

   std::string m_scenario_name;
   static std::map<std::string, int> m_aircraft_string_int_map;
};

inline const std::string &Scenario::GetScenarioName() { return m_scenario_name; }
