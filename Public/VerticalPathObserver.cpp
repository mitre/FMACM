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

#include "public/VerticalPathObserver.h"
#include "utility/constants.h"
#include <iomanip>

using namespace std;
using namespace aaesim::constants;

VerticalPathObserver::VerticalPathObserver(void)
      : scenName(""), fileName(""), header("") {

   // Default constructor.  Included for completeness.
   // Class not setup to use this and write trajectories.


   iter = -1;
   isTargetData = false;

}


VerticalPathObserver::VerticalPathObserver(string inScenName,
                                           string inFileName,
                                           bool inIsTargetData)
      :
      scenName(inScenName), fileName(inFileName) {

   // Constructor to initialize class.  Opens file using
   // full file name made from scenario name and file name
   // and writes header.
   //
   // scenName:Input scenario name.
   // fileName:Input file name.
   // isTargetData:Determines whether file is for target aircraft or not.
   //              true-file is for target aircraft.
   //              false-file is not for target aircraft.

   iter = -1;
   isTargetData = inIsTargetData;

   header = getHeader();

   initialize();

}

VerticalPathObserver::~VerticalPathObserver() {
}

void VerticalPathObserver::addTrajectory(int id,
                                         const VerticalPath &fullTraj) {

   // Adds (writes) input trajectory to the output file.
   //
   // id:aircraft id.
   // fullTraj:full trajectory.

   if (strm.is_open()) {
      for (unsigned int i = 0; i < fullTraj.x.size(); i++) {
         strm << iter << ",";
         strm << id << ",";
         strm << fullTraj.time[i] << ",";
         strm << fullTraj.x[i] / FEET_TO_METERS << ",";
         strm << fullTraj.h[i] / FEET_TO_METERS << ",";
         strm << fullTraj.v[i] / KNOTS_TO_METERS_PER_SECOND << ",";
         strm << fullTraj.h_dot[i] << ",";
         strm << fullTraj.v_dot[i] << ",";
         strm << fullTraj.theta[i] << ",";
         strm << fullTraj.gs[i] / KNOTS_TO_METERS_PER_SECOND << ",";
         strm << fullTraj.mass[i] << endl;
      }
   }

}


void VerticalPathObserver::setIter(int iterIn) {

   // Sets iteration.
   //
   // iterIn:input iteration.

   iter = iterIn;

}


int VerticalPathObserver::getIter(void) {

   // Gets iteration.
   //
   // returns iteration.

   return iter;

}


string VerticalPathObserver::createFullFileName(string scenName,
                                                string fileName) {

   // Builds full file name from scenario name and file name.
   //
   // scenName:scenario name.
   // fileName:file name.
   //
   // returns full file name.

   string fullName = scenName + "_" + fileName + ".csv";

   return fullName;

}


string VerticalPathObserver::getHeader(void) {

   // Gets header based on whether file is for target aircraft
   // or not.
   //
   // returns file header.

   string hdr = "Iteration,";

   if (isTargetData) {
      hdr += "target AC_ID,";
   } else {
      hdr += "AC_ID,";
   }

   hdr += "Time,Distance(feet),Altitude(Feet),IAS_Speed(Knots),Altitude_Change,Velocity_Change,Theta,GroundSpeed(Knots),Mass";

   return hdr;

}


void VerticalPathObserver::initialize(void) {
   // Initialization method.


   // Open file

   string fullFileName = createFullFileName(scenName, fileName);

   strm.open(fullFileName.c_str());


   if (strm.is_open()) {

      // Write file header

      strm << header << endl;

   } else {

      // Error message

      cout << "CANNOT OPEN TRAJECTORY FILE " << fullFileName << endl
           << "TRAJECTORY DATA WILL NOT BE WRITTEN" << endl << endl;

   }

}

void VerticalPathObserver::writeData() {
   // Destructor.  Closes file.
   if (strm.is_open()) {
      strm.close();
   }

}
