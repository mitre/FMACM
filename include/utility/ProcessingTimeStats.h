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
 * ProcessingTimeStats
 *
 * Created on: 18 May 16
 *     Author: greanias
 */


#pragma once

#include <string>
#include <ctime>


// Class to gather and dump processor time stats over poritions
// of code.
//
// Sample use of code:
//
// ProcessStuff::processLoopStats defined in header file.
//
// ProcessStuff::ProcessStuff(void) {
// ...
//     processingLoopStats.setHdr("Stats for processing loop");
// ...
//
// ProcessStuff::execute(void) {
//
// ...
//    processingLoopStats.start();
//
//    for (auto ix=0;ix<widgets.getNum();ix++) {
//      <code in for loop>
//    }
// 
//    processingLoopStats.stop();
//...
//
// Processor time data will be collected for the loop.
// When the procssingLoopStats destructor is invoked,
// all the collected stats will be dumped.


class ProcessingTimeStats
{
public:

   ProcessingTimeStats(void);

   ProcessingTimeStats(std::string str);

   ~ProcessingTimeStats(void);

   // Sets header for dump.
   void setHdr(std::string str);

   // Gets time at start point in code.
   void start(void);

   // Collects processing time in ms between start point and this point.
   void stop(void);

private:
   // Gathers stats between start and stop point.
   void gather(void);

   // Dumps stats.
   void dump(void);

   clock_t t0;
   clock_t t1;

   double ms;
   int entries;

   std::string hdr;
};
