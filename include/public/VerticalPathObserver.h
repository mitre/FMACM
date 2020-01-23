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

#include "public/VerticalPath.h"
#include <fstream>
#include <string>

// Used to write trajectory data for all the flights over all the iterations for a type of trajectory. Examples of types
// of trajectories are the precalculated trajectories created at the beginning of the run, kinematic trajectories for
// own aircraft, or kinematic trajectories for target aircraft.

class VerticalPathObserver
{

public:
   VerticalPathObserver();

   VerticalPathObserver(std::string scenario_name,
                        std::string file_name,
                        bool is_target_aircraft_data);

   virtual ~VerticalPathObserver();

   void AddTrajectory(int id,
                      const VerticalPath &vertical_path);

   void SetIterationNumber(int iteration_number);

   int GetIterationNumber();

   void WriteData();

protected:
   void Initialize();

   std::string m_scenario_name;
   std::string m_file_name;
   std::string m_column_header;

   std::ofstream out_stream;

private:
   std::string CreateFullFileName(const std::string &scenario_name,
                                  const std::string &file_name);

   std::string GetHeader();

   int m_iteration;

   bool m_is_target_aircraft_data;
};

inline void VerticalPathObserver::SetIterationNumber(int iteration_number) {
   m_iteration = iteration_number;
}

inline int VerticalPathObserver::GetIterationNumber() {
   return m_iteration;
}