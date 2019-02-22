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

#include "framework/TestFrameworkScenario.h"
#include <stdio.h>
#include <cstring>
#include "math/CustomMath.h"
#include "aaesim/Bada.h"
#include "loader/Loadable.h"
#include "utility/constants.h"
#include "public/StandardAtmosphere.h"
#include "public/WindZero.h"
#include <list>

using namespace std;

// Static member initialization
const Units::SecondsTime TestFrameworkScenario::mDefaultSimulationTimeStep = Units::SecondsTime(
      1.0); // default value in case it is not loaded from the scenario file


TestFrameworkScenario::TestFrameworkScenario() {
   // Initialize scenario values
   m_start_time_seed = 0; // start time seed initialized to 0 (random seed)
   m_seed = 12345;
   m_mean_inter_delivery_time = 0;
   m_stdev_inter_delivery_time = 0;
   m_start_time_seed = 12345;
   m_weather = WeatherTruth(std::shared_ptr<Wind>(),
                           std::shared_ptr<Atmosphere>(new StandardAtmosphere(Units::CelsiusTemperature(0))));
   m_weather.east_west.setBounds(1, 5);
   m_weather.north_south.setBounds(1, 5);
}

TestFrameworkScenario::~TestFrameworkScenario() {
   // destructor stub
}

// inherited Loadable method that defines how a TestFrameworkScenario runfile is loaded
bool TestFrameworkScenario::load(DecodedStream *input) {
   // local vars used just inside the laoder
   string bada_data_path;
   string wind_truth_file = "";
   string wind_forecast_file = "";
   int predictedWindOpt = 0;
   bool blendWind = true;

   // register things with the Loadable base class
   set_stream(input); //-----------------------------------------------------
   register_var("bada_data_path", &bada_data_path, true); // required
   register_named_vector_item("aircraft", &m_master_aircraft_list, true); // required

   // get the base class to load things
   complete();

   // this takes care of all the adaptation of the data structures
   PostLoad(bada_data_path, wind_truth_file, wind_forecast_file, predictedWindOpt, blendWind,
             mDefaultSimulationTimeStep);

   return true;
}

// method that does initialization after the TestFrameworkScenario has been loaded
void TestFrameworkScenario::PostLoad(string bada_data_path,
                                      string wind_truth_file,
                                      string wind_forecast_file,
                                      int predicted_wind_opt,
                                      bool blend_wind,
                                      Units::Time simulation_time_step) {

   strcpy(Bada::input_path, bada_data_path.c_str());

   // Setup observers
   InternalObserver *internalObserver = InternalObserver::getInstance();
   internalObserver->set_scenario_name(GetScenarioName());

   PostLoadAircraft(simulation_time_step, predicted_wind_opt, blend_wind);

   SimulationTime::set_simulation_time_step(simulation_time_step);

}

//-------------------------------

void TestFrameworkScenario::PostLoadAircraft(
      Units::Time simulation_time_step,
      int predicted_wind_opt,
      bool blend_wind) {

   // Post load each aircraft individually.
   for (vector<TestFrameworkAircraft>::iterator i = m_master_aircraft_list.begin(); i != m_master_aircraft_list.end();
        i++) {
      (*i).PostLoad(simulation_time_step, predicted_wind_opt, blend_wind, m_weather);
   }


}


void TestFrameworkScenario::LoadOneScenarioFromScenarioClassIntoActorLists() {

   m_aircraft_list = m_master_aircraft_list;

   int start_time = 0;

   int count = 0;

   for (vector<TestFrameworkAircraft>::iterator i = m_aircraft_list.begin(); i != m_aircraft_list.end(); i++) {
      (*i).m_start_time = start_time;
      //start time for the next aircraft:
      start_time += (int) Scenario::m_rand.TruncatedGaussianSample((double) m_mean_inter_delivery_time,
                                                                  (double) m_stdev_inter_delivery_time, 3.);

      (*i).Initialize(Units::NauticalMilesLength(99), m_weather);
      // TODO what to do with seed?

      count++;
   }
}

// method to process one scenario from the list of scenario files
void TestFrameworkScenario::ProcessOneScenario() {

   //for each iteration:
   for (int i = 0; i < number_of_iterations; i++) {
      //Reload the scenario from the scenario class into actors and actor lists.
      //Reason: To make the internal states of actors clean.
      //Serving as the "reset" function for all actors and actor lists.
//		printf("Iteration %d:\n",i);

      LoadOneScenarioFromScenarioClassIntoActorLists();

      // Initialize the iteration.
      InitializeIterationState(number_of_aircraft);
      //InternalObserver::getInstance()->scenario_iter = i; // sets the current iteration in the internal observer

      // Process the iteration.
      ProcessOneIteration(i);

   }
   //end for each iteration:

   //Process internal observer output:
//	InternalObserver::getInstance()->process();

}


void TestFrameworkScenario::InitializeIterationState(int number_of_aircraft) {

   // Iteration initializer.
   //
   // numberOfAircraft:number of aircraft in scenario.


   //clear the either for a new iteration
   //initial_state.ADS_B_ether.clear();

   //initial_state.aircraft_truth_state_vector_list.clear();

   // Perform any needed initialization for the metrics.
   InitializeIterationMetrics(number_of_aircraft);

}

void TestFrameworkScenario::InitializeIterationMetrics(int number_of_aircraft) {

   // Performs initialization for the metrics for an iteration.
   //
   // numberOfAircraft:number of aircraft in scenario.
   InternalObserver::getInstance()->initializeIteration(number_of_aircraft);
}

// method to run one iteration of the current scenario
void TestFrameworkScenario::ProcessOneIteration(int iter) {
   std::string scenName = GetScenarioName();
   int findResult = scenName.find("-scenario");
   if (findResult >= 0) {
      scenName = scenName.replace(findResult, findResult + 9, "");
   }
   m_acstates = fopen(scenName.append("_AcStates.csv").c_str(), "w");

   fprintf(m_acstates, "Time[sec], DTG[m], V(tas)[m/s], vRate[m/s], x[m], y[m], h[m]\n"); // header text

   SimulationTime time;
   time.init();
   bool iteration_finished = false;

   //cycle loop:
   while (!iteration_finished) {
//		printf("time: %g\n", time.get_current_simulation_time());

      iteration_finished = ProcessOneCycle(time); // run the cycle and generate the new states

      time.increment();
   }
   //end cycle loop
}

// method to process the current cycle
bool TestFrameworkScenario::ProcessOneCycle(SimulationTime &time) {
   bool iteration_finished = true;

   //Process all the aircraft:
   iteration_finished = ProcessAllAircraft(time);

   return iteration_finished;
}

// method to update all of the aircraft for the current cycle
bool TestFrameworkScenario::ProcessAllAircraft(SimulationTime &time) {
   bool iteration_finished = true;
   bool aircraft_iteration_finished = true;

   // loop to process all aircraft:
   vector<TestFrameworkAircraft>::iterator aircraft;

   for (aircraft = m_aircraft_list.begin(); aircraft != m_aircraft_list.end(); ++aircraft) {
      // run the aircraft update method for the current aircraft
      aircraft_iteration_finished =
            (*aircraft).Update(time);

      if (!aircraft_iteration_finished) {
         AircraftState state = (*aircraft).m_truth_state_vector_old;
         RecordState(state);
      }

      iteration_finished = iteration_finished && aircraft_iteration_finished;
   }

   return iteration_finished;
}   //end process all aircraft

void TestFrameworkScenario::RecordState(const AircraftState &aircraft_state) const {
   double v = hypot(aircraft_state.m_xd * FEET_TO_METERS - aircraft_state.m_Vwx,
                    aircraft_state.m_yd * FEET_TO_METERS - aircraft_state.m_Vwy)
              / cos(aircraft_state.m_gamma); // m/s

   fprintf(m_acstates, "%.0f,%.5f,%.3f,%.3f,%.5f,%.5f,%.5f\n",
           aircraft_state.m_time,
           aircraft_state.m_distance_to_go,
           v,
           aircraft_state.m_zd * FEET_TO_METERS,
           aircraft_state.m_x * FEET_TO_METERS,
           aircraft_state.m_y * FEET_TO_METERS,
           aircraft_state.m_z * FEET_TO_METERS
   );
   fflush(m_acstates);
}
