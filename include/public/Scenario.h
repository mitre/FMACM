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

#pragma once

#include "public/LoggingLoadable.h"
#include "public/WeatherTruth.h"
#include <string>
#include "math/RandomGenerator.h"


/**
 * This is a very basic Scenario implementation. Expanded implementations should not edit this class but inherit from it.
 */
class Scenario : public LoggingLoadable
{
public:
   static const Units::NauticalMilesLength DEFAULT_ADS_B_RECEPTION_RANGE_THRESHOLD;

   Scenario();

   ~Scenario();

   virtual void ProcessOneScenario();

   virtual bool load(DecodedStream *input);

   // Scenario name getter/setters. Putting default impl here for simplicity.
   virtual std::string GetScenarioName() {
      return m_scenario_name;
   }

   virtual void SetScenarioName(const std::string in);

   static int GetUniqueIdForAircraftId(const std::string aircraft_id) {
      // Note: implementation intentionally here in the header file because static method
      bool is_in_map = m_aircraft_string_int_map.find(aircraft_id) != m_aircraft_string_int_map.end();
      if (is_in_map) {
         return m_aircraft_string_int_map[aircraft_id];
      }
      return AIRCRAFT_ID_NOT_IN_MAP; // not in map, return AIRCRAFT_ID_NOT_IN_MAP
   }

   static int GenerateNewUniqueIdForAircraftId(const std::string aircraft_id) {
      // Note: implementation intentionally here in the header file because static method
      int oldId = GetUniqueIdForAircraftId(aircraft_id);
      if (oldId == AIRCRAFT_ID_NOT_IN_MAP) {
         int newid = m_aircraft_string_int_map.size(); // zero-based, so the first aircraft will have uniqueid zero
         m_aircraft_string_int_map[aircraft_id] = newid;
         return newid;
      } else {
         return oldId;
      }
   }

   static void ClearAircraftIdMap() {
      m_aircraft_string_int_map.clear();
   }

   static const int AIRCRAFT_ID_NOT_IN_MAP;

   static RandomGenerator m_rand;

protected:
   WeatherTruth m_weather;

private:

   std::string m_scenario_name;
   static std::map<std::string, int> m_aircraft_string_int_map;

};
