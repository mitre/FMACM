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

#include <string>
#include <scalar/Time.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Angle.h>
#include <scalar/Temperature.h>
#include "public/minicsv.h"

/**
 * This class serves as a parent for the two most recent file writer classes,
 * TrajectoryFile and PredictionFile. As more of the file output capability in AAESim
 * is migrated away from the Internal and External observers, this class should expand to hold
 * the common data members and data gathering capability for these, and any future classes. May
 * be expanded to include input.
 */

class OutputHandler
{

public:

   /*
    * The constructor takes the scenario name, and the desired file extension to describe the
    * type of output contained in this file.
    */
   OutputHandler(const std::string &scenario_name,
                 const std::string &file_extension);

   // Does nothing at this level. All file writing is done in the destructor in a derived class.
   virtual ~OutputHandler();

protected:

   // Structure to contain data that is generally common to output files.
   // Derived classes should also derive from this structure, and add the
   // additional necessary members to match the desired output.
   struct CommonSimData
   {
      CommonSimData() {
         iteration_number = -1;
         simulation_time = Units::SecondsTime(-1.0);
         acid.assign("");
      };

      // The current iteration number
      int iteration_number;

      // The time when this data was generated
      Units::Time simulation_time;

      // The Aircraft ID
      std::string acid;

   };

   // Name of file to be written
   std::string filename;

   // Output stream that handles writing when object is destroyed
   mini::csv::ofstream os;
};