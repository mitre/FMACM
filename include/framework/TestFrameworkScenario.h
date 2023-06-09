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

#include "public/Scenario.h"
#include "framework/TestFrameworkAircraft.h"
#include "public/SimulationTime.h"
#include "loader/Loadable.h"
#include <list>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <scalar/Angle.h>
#include <scalar/Time.h>

class TestFrameworkScenario : public Scenario {
  public:
   TestFrameworkScenario();

   virtual ~TestFrameworkScenario();

   void ProcessOneScenario();

   void InitializeIterationState(int number_of_aircraft_in);

   void InitializeIterationMetrics(int number_of_aircraft_in);

   void ProcessOneIteration();

   bool ProcessOneCycle(SimulationTime &time);

   bool ProcessAllAircraft(SimulationTime &time);

   void LoadOneScenarioFromScenarioClassIntoActorLists();

   bool load(DecodedStream *input);

  private:
   void PostLoadAircraft(Units::Time simulation_time_step, int predicted_wind_opt, bool blend_wind);

   void PostLoad(const std::string &bada_data_path, int predicted_wind_opt, bool blend_wind,
                 Units::Time simulation_time_step);

   void RecordState(const aaesim::open_source::AircraftState &aircraft_state,
                    const aaesim::open_source::DynamicsState &dynamics_state) const;

   static const Units::SecondsTime mDefaultSimulationTimeStep;
   static const int number_of_iterations;
   static const int number_of_aircraft;

   std::vector<TestFrameworkAircraft> m_aircraft_list;
   std::vector<TestFrameworkAircraft> m_master_aircraft_list;  // this list is loaded and then copied for each iteration

   double m_mean_inter_delivery_time;
   double m_stdev_inter_delivery_time;
   FILE *m_acstates;
};
