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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <string>
#include "Time.h"
#include "utility/Logging.h"

class SimulationTime
{
public:
   //Data:


   //Methods:
   SimulationTime(void);

   ~SimulationTime(void);

   SimulationTime(const SimulationTime &in);

   SimulationTime &operator=(const SimulationTime &in);

   static const SimulationTime make(const Units::SecondsTime time);

   void init(void);

   void increment(void);

   Units::SecondsTime get_current_simulation_time() const;

   std::string getCurrentSimulationTimeAsString() const;

   static Units::SecondsTime get_simulation_time_step() {
      return simulation_time_step;
   }

   static void set_simulation_time_step(Units::Time in) {
      simulation_time_step = in;;
   }

   int get_sim_cycle() const;

   void set_cycle(int cycle_in);


private:
   void copy(SimulationTime const &in);

   //Data:
   int cycle;
   Units::SecondsTime current_time;
   static Units::SecondsTime simulation_time_step;

   static log4cplus::Logger logger;
};
