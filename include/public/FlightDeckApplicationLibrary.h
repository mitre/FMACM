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
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/FlightDeckApplicationLoader.h"
#include "public/OutputHandler.h"

// FIXME should I replace OutputHandler with special FlightDeckWriter? Probably.
// See also AAES-1329
namespace aaesim {
namespace open_source {

struct aaesim::open_source::FlightDeckApplicationLibrary {
   /*
   Allows the library to provide a mapping of keyword:factory for each flightdeck application it can construct.

   The key is a unique string that will be used internally to identify which application has been loaded at run-time.
   */
   std::map<std::string, aaesim::open_source::FlightDeckApplicationLoader> RegisterFlightDeckApplications() = 0;

   /*
   Allows the library to declare output handlers that can be used with their flightdeck applications.

   The map key is a string name that will appear in the scenario file. The simulation will use this
   string to enable/disable the use of a file writer.
   */
   std::map<std::string, aaesim::open_source::OutputHandler> RegisterApplicationDataWriters() = 0;

   /*
   The map key must be a string match for each application declared in RegisterFlightDeckApplications();

   FIXME Consider if this Register should be combined with the return of RegisterFlightDeckApplications() like this:

   */
   std::map<std::string, std::vector<aaesim::open_source::OutputHandler>>
         RegisterDataWritersPerFlightDeckApplication() = 0;
};
}  // namespace open_source
}  // namespace aaesim
