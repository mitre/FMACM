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

#include "public/Scenario.h"
#include "framework/TestFrameworkAircraft.h"
#include "public/SimulationTime.h"
#include "loader/Loadable.h"
#include <list>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <Angle.h>
#include <Time.h>

class TestFrameworkScenario : public Scenario
{
public:
   TestFrameworkScenario();

   virtual ~TestFrameworkScenario();

   void ProcessOneScenario();

   void InitializeIterationState(int number_of_aircraft);

   void InitializeIterationMetrics(int number_of_aircraft);

   void ProcessOneIteration(int iter);

   bool ProcessOneCycle(SimulationTime &time);

   bool ProcessAllAircraft(SimulationTime &time);

   void LoadOneScenarioFromScenarioClassIntoActorLists();

   bool load(DecodedStream *input);

private:

   void PostLoadAircraft(Units::Time simulation_time_step,
         int predicted_wind_opt,
         bool blend_wind);

   void PostLoad(std::string bada_data_path,
         std::string wind_truth,
         std::string wind_forecast,
         int predicted_wind_opt,
         bool blend_wind,
         Units::Time simulation_time_step);

   void RecordState(const AircraftState &aircraft_state) const;

   // Default declarations for the loadable parameters
   static const Units::SecondsTime mDefaultSimulationTimeStep;

   // scenario initialization
   static const int number_of_iterations = 1;
   static const int number_of_aircraft = 1;
   std::vector<TestFrameworkAircraft> m_aircraft_list;
   double m_seed;
   std::vector<TestFrameworkAircraft> m_master_aircraft_list; // this list is loaded and then copied for each iteration

   //scenario data used for all iterations:
   double m_mean_inter_delivery_time;
   double m_stdev_inter_delivery_time;
   double m_start_time_seed;
   FILE *m_acstates;
};

