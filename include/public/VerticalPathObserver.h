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

#include "public/VerticalPath.h"
#include <fstream>
#include <string>

// Trajectory output class.  Used to write full trajectory flight data
// into a .csv file.  Output is the trajectory data for all the flights
// over all the iterations for a type of trajectory.  Examples of types
// of trajectories are the precalculated trajectories created at the
// beginning of the run, kinematic trajectories for own aircraft, or
// kinematic trajectories for target aircraft.


class VerticalPathObserver
{

public:
   // Basic constructor.  Do not used; included for completeness only.
   VerticalPathObserver(void);

   // Setup constructor.
   VerticalPathObserver(std::string inScenName,
                        std::string inFileName,
                        bool inIsTargetData);

   virtual ~VerticalPathObserver();

   // Adds a new trajectory to file.
   void addTrajectory(int id,
                      const VerticalPath &fullTraj);

   // Sets iteration.
   void setIter(int iterIn);

   // Gets iteration.
   int getIter(void);

   void writeData();


protected:

   // Initializes class.
   void initialize(void);

   // Makes full file name from scenario name and file name.
   std::string createFullFileName(std::string scenName,
                                  std::string fileName);

   std::string scenName; // Name of scenario being run.
   std::string fileName; // Individual file name.
   std::string header; // File line header.

   std::ofstream strm; // Output stream of file.


private:


   // Gets header.
   virtual std::string getHeader(void);

   int iter; // Current iteration.

   bool isTargetData; // Determines whether data is for target or own ship.

};
