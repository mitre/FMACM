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

#include <fstream>
#include <string>
#include <log4cplus/logger.h>
#include "public/VerticalPath.h"

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
   static log4cplus::Logger m_logger;

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