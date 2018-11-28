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

   Scenario(void);

   ~Scenario(void);

   virtual void process_one_scenario();

   virtual bool load(DecodedStream *input);

   // Scenario name getter/setters. Putting default impl here for simplicity.
   virtual std::string get_scenario_name() {
      return mScenarioName;
   }

   virtual void set_scenario_name(std::string in);

   static int getUniqueIdForAircraftId(std::string aircraftId) {
      // Note: implementation intentionally here in the header file because static method
      bool isInMap = mAircraftStringIntMap.find(aircraftId) != mAircraftStringIntMap.end();
      if (isInMap) {
         return mAircraftStringIntMap[aircraftId];
      }
      return AIRCRAFT_ID_NOT_IN_MAP; // not in map, return AIRCRAFT_ID_NOT_IN_MAP
   }

   static int generateNewUniqueIdForAircraftId(std::string aircraftId) {
      // Note: implementation intentionally here in the header file because static method
      int oldId = getUniqueIdForAircraftId(aircraftId);
      if (oldId == AIRCRAFT_ID_NOT_IN_MAP) {
         int newid = mAircraftStringIntMap.size(); // zero-based, so the first aircraft will have uniqueid zero
         mAircraftStringIntMap[aircraftId] = newid;
         return newid;
      } else {
         return oldId;
      }
   }

   static void clearAircraftIdMap() {
      mAircraftStringIntMap.clear();
   }

   static const int AIRCRAFT_ID_NOT_IN_MAP;

   static RandomGenerator mRand;

protected:
   WeatherTruth mWeather;

private:

   std::string mScenarioName;
   static std::map<std::string, int> mAircraftStringIntMap;

};
