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
 * AircraftIntentFromFile.h
 *
 *  Created on: Feb 24, 2015
 *      Author: SBOWMAN
 */

#ifndef AIRCRAFTINTENTFROMFILE_H_
#define AIRCRAFTINTENTFROMFILE_H_

#include "public/AircraftIntent.h"

class AircraftIntentFromFile : public AircraftIntent
{
public:
   AircraftIntentFromFile();

   virtual ~AircraftIntentFromFile();

   // override method inherited from parent as we don't want any of that in this implementation
   bool load(DecodedStream *input);

   // override method inherited from parent as we don't want nay of the base class implementation
   void update_xy_from_latlon();

private:

   /**
    * Implementation that takes the data from CSV and populates the parent's waypoint data sets.
    */
   void populateWaypointsFromCsv(std::string csvFile);

   double localstod(std::string s);

   int localstoi(std::string s);
};

#endif /* AIRCRAFTINTENTFROMFILE_H_ */
