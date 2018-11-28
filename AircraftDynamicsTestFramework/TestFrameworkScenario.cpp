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

/*
 * TestFrameworkScenario.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: sbowman
 */

#include "framework/TestFrameworkScenario.h"
#include <stdio.h>
#include <cstring>
#include "math/CustomMath.h"
#include "aaesim/Bada.h"
#include "loader/Loadable.h"
#include "utility/constants.h"
#include "public/StandardAtmosphere.h"
#include <list>

using namespace std;

// Static member initialization
const Units::SecondsTime TestFrameworkScenario::mDefaultSimulationTimeStep = Units::SecondsTime(
      1.0); // default value in case it is not loaded from the scenario file


TestFrameworkScenario::TestFrameworkScenario() {
   // Initialize scenario values
   start_time_seed = 0; // start time seed initialized to 0 (random seed)
   seed = 12345;
   mean_inter_delivery_time = 0;
   stdev_inter_delivery_time = 0;
   start_time_seed = 12345;
   mWeather = WeatherTruth(std::shared_ptr<Wind>(),
                           std::shared_ptr<Atmosphere>(new StandardAtmosphere(Units::CelsiusTemperature(0))));
   mWeather.east_west.setBounds(1, 5);
   mWeather.north_south.setBounds(1, 5);
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
   register_named_vector_item("aircraft", &master_aircraft_list, true); // required

   // get the base class to load things
   bool r = complete(); //-----------------------------------------------------

   // this takes care of all the adaptation of the data structures
   post_load(bada_data_path, wind_truth_file, wind_forecast_file, predictedWindOpt, blendWind,
             mDefaultSimulationTimeStep);

   return true;
}

// method that does initialization after the TestFrameworkScenario has been loaded
void TestFrameworkScenario::post_load(string bada_data_path,
                                      string wind_truth_file,
                                      string wind_forecast_file,
                                      int predictedWindOpt,
                                      bool blendWind,
                                      Units::Time simulation_time_step) {

   strcpy(Bada::input_path, bada_data_path.c_str());

   // Setup observers
   InternalObserver *internalObserver = InternalObserver::getInstance();
   internalObserver->set_scenario_name(get_scenario_name());

   // Read the wind data in first so that grid available to get predicted winds
   // for each aircraft's individual call.

//  if ((Wind::windForecastFileName != wind_forecast_file) ||
//      (Wind::windTruthFileName != wind_truth_file)) {
//    Wind::windTruthFileName = wind_truth_file;
//    Wind::windForecastFileName = wind_forecast_file;
//    Wind::loadWindFiles();
//  }

   post_load_aircraft(simulation_time_step, predictedWindOpt, blendWind);

   SimulationTime::set_simulation_time_step(simulation_time_step);

}

//-------------------------------

void TestFrameworkScenario::post_load_aircraft(
      Units::Time simulation_time_step,
      int predictedWindOpt,
      bool blendWind) {
   // Post load each aircraft individually.
   for (vector<TestFrameworkAircraft>::iterator i = master_aircraft_list.begin(); i != master_aircraft_list.end();
        i++) {
      (*i).post_load(simulation_time_step, predictedWindOpt, blendWind);
   }


}


void TestFrameworkScenario::load_one_scenario_from_scenario_class_into_actor_lists() {

   aircraft_list = master_aircraft_list;

   int start_time = 0;

   int count = 0;

   for (vector<TestFrameworkAircraft>::iterator i = aircraft_list.begin(); i != aircraft_list.end(); i++) {
      (*i).start_time = start_time;
      //start time for the next aircraft:
      start_time += (int) Scenario::mRand.truncatedGaussianSample((double) mean_inter_delivery_time,
                                                                  (double) stdev_inter_delivery_time, 3.);

      (*i).init(Units::NauticalMilesLength(99), mWeather);
      // TODO what to do with seed?

      count++;
   }
}

// method to process one scenario from the list of scenario files
void TestFrameworkScenario::process_one_scenario() {

   //for each iteration:
   for (int i = 0; i < number_of_iterations; i++) {
      //Reload the scenario from the scenario class into actors and actor lists.
      //Reason: To make the internal states of actors clean.
      //Serving as the "reset" function for all actors and actor lists.
//		printf("Iteration %d:\n",i);

      load_one_scenario_from_scenario_class_into_actor_lists();

      // Initialize the iteration.
      initializeIterationState(number_of_aircraft);
      //InternalObserver::getInstance()->scenario_iter = i; // sets the current iteration in the internal observer

      // Process the iteration.
      process_one_iteration(i);

   }
   //end for each iteration:

   //Process internal observer output:
//	InternalObserver::getInstance()->process();

}


void TestFrameworkScenario::initializeIterationState(int numberOfAircraft) {

   // Iteration initializer.
   //
   // numberOfAircraft:number of aircraft in scenario.


   //clear the either for a new iteration
   //initial_state.ADS_B_ether.clear();

   //this->initial_state.aircraft_truth_state_vector_list.clear();

   // Perform any needed initialization for the metrics.
   initializeIterationMetrics(numberOfAircraft);

}

void TestFrameworkScenario::initializeIterationMetrics(int numberOfAircraft) {

   // Performs initialization for the metrics for an iteration.
   //
   // numberOfAircraft:number of aircraft in scenario.
   InternalObserver::getInstance()->initializeIteration(numberOfAircraft);
}

// method to run one iteration of the current scenario
void TestFrameworkScenario::process_one_iteration(int iter) {
   std::string scenName = this->get_scenario_name();
   int findResult = scenName.find("-scenario");
   if (findResult >= 0) {
      scenName = scenName.replace(findResult, findResult + 9, "");
   }
   acstates = fopen(scenName.append("_AcStates.csv").c_str(), "w");

   fprintf(acstates, "Time[sec], DTG[m], V(tas)[m/s], vRate[m/s], x[m], y[m], h[m]\n"); // header text

   SimulationTime time;
   time.init();
   bool iteration_finished = false;

   //cycle loop:
   while (!iteration_finished) {
//		printf("time: %g\n", time.get_current_simulation_time());

      iteration_finished = process_one_cycle(time); // run the cycle and generate the new states

      time.increment();
   }
   //end cycle loop
}

// method to process the current cycle
bool TestFrameworkScenario::process_one_cycle(SimulationTime &time) {
   bool iteration_finished = true;

   //Process all the aircraft:
   iteration_finished = process_all_aircraft(time);

   return iteration_finished;
}

// method to update all of the aircraft for the current cycle
bool TestFrameworkScenario::process_all_aircraft(SimulationTime &time) {
   bool iteration_finished = true;
   bool aircraft_iteration_finished = true;

   // loop to process all aircraft:
   vector<TestFrameworkAircraft>::iterator aircraft;

   for (aircraft = aircraft_list.begin(); aircraft != aircraft_list.end(); ++aircraft) {
      // run the aircraft update method for the current aircraft
      aircraft_iteration_finished =
            (*aircraft).update(time);

      if (!aircraft_iteration_finished) {
         AircraftState state = (*aircraft).truth_state_vector_old;
         recordState(state);
      }

      iteration_finished = iteration_finished && aircraft_iteration_finished;
   }

   return iteration_finished;
}   //end process all aircraft

void TestFrameworkScenario::recordState(const AircraftState &aircraftState) const {
   double v = hypot(aircraftState.xd * FT_M - aircraftState.Vwx,
                    aircraftState.yd * FT_M - aircraftState.Vwy)
              / cos(aircraftState.gamma); // m/s

   fprintf(acstates, "%.0f,%.5f,%.3f,%.3f,%.5f,%.5f,%.5f\n",
           aircraftState.time,
           aircraftState.m_distance_to_go,
           v,
           aircraftState.zd * FT_M,
           aircraftState.x * FT_M,
           aircraftState.y * FT_M,
           aircraftState.z * FT_M
   );
   fflush(acstates);
}
