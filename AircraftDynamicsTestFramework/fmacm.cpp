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

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include "cppmanifest/version.h"
#include "cppmanifest/cppmanifest.h"
#include "public/RunFile.h"
#include "public/Scenario.h"
#include "framework/TestFrameworkScenario.h"
#include "loader/RunFileArchiveDirector.h"
#include "loader/Loadable.h"
#include "utility/Logging.h"

#ifndef _LINUX_
#include <direct.h>
#endif

using namespace std;

#define _MAX_PATH 260

void read_runfile(RunFile &run_file, std::string str);

void process_scenarios(RunFile &run_file);

static log4cplus::Logger logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("main"));
const std::string VERSION_FLAG("--version");

int main(int argc, char *argv[]) {
   InitializeLogging();
   LOG4CPLUS_INFO(logger, "running " << aaesim::cppmanifest::getVersion());
   HTMLDump::SetSoftwareVersion(aaesim::cppmanifest::getVersion());

   // handle command line flag --version, --buildinfo
   if (argc == 2) {
      std::string arg1(argv[1]);
      if (arg1 == VERSION_FLAG) {
         cout << "aaesim version " << aaesim::cppmanifest::getVersion() << endl;
         return 0;  // nothing else to do
      } else if (arg1 == aaesim::cppmanifest::BUILDINFO_CLI_FLAG) {
         cout << "fmacm build info:" << endl;
         aaesim::cppmanifest::printMetaData();
         return 0;  // nothing else to do
      }
   }

   RunFile run_file;
   std::string arg2("");
   if (argc == 2) {
      arg2 = argv[1];
   }

   read_runfile(run_file, arg2);

   process_scenarios(run_file);
}

void read_runfile(RunFile &run_file, std::string arg) {
   // Gets list of run scenario files from configuration file.
   //
   // run_file:output list of scenario files.
   // arg:argument from run command.  If set, contains
   //     configuration file name for Linux runs, (can
   //     include directory).

   // Set configuration filename to be opened.

   std::string configFile("");

#ifdef _LINUX_

   // Setup for linux run.

   if (!arg.empty()) {

      // Argument set

      if (arg.find('/') != string::npos) {

         // Has directory-take the argument as the whole
         // configuration file name.

         configFile = arg;

      } else {

         // Set configuration file name to default
         // directory plus argument.

         configFile = "../Run_Files/";
         configFile += arg.c_str();
      }

   } else {

      // Argument not set-use default directory and name.

      configFile = "../Run_Files/config-loader.txt";
   }

#else

   // Setup for windows run-a straight default.

   configFile = "C:/WinSS/config-loader.txt";

#endif

   FILE *fp;

   fp = fopen(configFile.c_str(), "r");

   if (fp == NULL) {
      printf("Configuration file %s not found.", configFile.c_str());
   } else {
      // Get scenario files for the run.

      int numScenarios;
      char scenarioFileName[300];

      fscanf(fp, "%d", &numScenarios);
      for (int i = 0; i < numScenarios; i++) {
         fscanf(fp, "%s", scenarioFileName);
         pair<string, std::shared_ptr<Scenario> > newPair;
         newPair.first = scenarioFileName;
         newPair.second = std::shared_ptr<TestFrameworkScenario>(new TestFrameworkScenario);

         run_file.scenarios.push_back(newPair);
      }
      fclose(fp);
   }
}

void process_scenarios(RunFile &run_file) {
   bool r;

   list<string>::iterator i;

   // iterating through all the scenarios:
   vector<pair<string, std::shared_ptr<Scenario> > >::const_iterator citr;
   for (citr = run_file.scenarios.begin(); citr != run_file.scenarios.end(); ++citr) {
      std::string sFileName = citr->first;

      DecodedStream stream;

      r = stream.open_file(sFileName);
      if (!r) {
         string msg = string("Cannot open file ") + sFileName;
         LOG4CPLUS_FATAL(logger, msg);
         throw runtime_error(msg);
      }

      stream.set_echo(false);  // default set to false, must turn it on in input file

      char CurrentPath[_MAX_PATH];

      getcwd(CurrentPath, _MAX_PATH);

      string cwd = CurrentPath;

      stream.set_Local_Path(cwd);

      std::shared_ptr<RunFileArchiveDirector> rfad = std::make_shared<RunFileArchiveDirector>();

      stream.set_Archive_Director(rfad);

      // Set scenario name here, before load, for time to go output.
      citr->second->SetScenarioName(sFileName);  // sets the scenario name

      // So user can tell which scenario is being processed during regression testing
      LOG4CPLUS_INFO(logger, "Processing Scenario File: " << sFileName << std::endl);

      // Load the parameters for this scenario
      citr->second->load(&stream);

      // Process the loaded scenario
      citr->second->ProcessOneScenario();

      // Clear out the aircraft id map
      citr->second->ClearAircraftIdMap();
   }
}
