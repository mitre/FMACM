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

#include "public/version.h"
#include "public/cppmanifest.h"
#include "public/RunFile.h"
#include "public/InternalObserver.h"
#include "public/Scenario.h"
#include "framework/TestFrameworkScenario.h"
#include "loader/RunFileArchiveDirector.h"
#include "loader/Loadable.h"
#include "utility/Logging.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#ifndef _LINUX_
#include <direct.h>
#endif

using namespace std;


#define _MAX_PATH 260


void read_runfile(RunFile &run_file,
                  std::string str);

void process_scenarios(RunFile &run_file);

void process_overall_output(InternalObserver &internal_observer);

static log4cplus::Logger logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("main"));
const std::string versionFlag("--version");
const std::string buildinfoFlag("--buildinfo");

int main(int argc,
         char *argv[]) {
   InitializeLogging();
   string version = "aaesim version " + aaesim::getVersion();
   LOG4CPLUS_INFO(logger, "running " << version);
   HTMLDump::SetSoftwareVersion(version);

   // handle command line flag --version, --buildinfo
   if (argc == 2) {
      std::string arg1(argv[1]);
      if (arg1 == versionFlag) {
         cout << "aaesim version " << aaesim::getVersion() << endl;
         return 0; // nothing else to do
      } else if (arg1 == buildinfoFlag) {
         cout << "aaesim build info:" << endl;
         cout << "Build version: " << aaesim::getVersion() << endl;
         cout << "Created by: " << cppmanifest::getUserName() << endl;
         cout << "Created date-time: " << cppmanifest::getBuildTimeStamp() << endl;
         cout << "Built with GCC version: " << cppmanifest::getBuildCompilerVersion() << endl;
         cout << "Built on system name: " << cppmanifest::getBuildSystemName() << endl;
         cout << "Built on system processor: " << cppmanifest::getBuildSystemProcessor() << endl;
         cout << "Built with system ver: " << cppmanifest::getBuildSystemVersion() << endl;
         cout << "Built on system host name: " << cppmanifest::getBuildHostName() << endl;
         cout << "Built from git branch: " << cppmanifest::getGitBranch() << endl;
         if (cppmanifest::getGitIsClean()) {
            cout << "Built from git hash: " << cppmanifest::getGitHash() << endl;
         } else {
            cout << "Built from git hash: " << cppmanifest::getGitHash() << "-DIRTY" << endl;
         }

         return 0; // nothing else to do
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

void read_runfile(RunFile &run_file,
                  std::string arg) {
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

         run_file.scenariosToRun.push_back(newPair);
      }
      fclose(fp);
   }
}


void process_scenarios(RunFile &run_file) {
   bool r;

   list<string>::iterator i;

   //iterating through all the scenarios:
   vector<pair<string, std::shared_ptr<Scenario> > >::const_iterator citr;
   for (citr = run_file.scenariosToRun.begin(); citr != run_file.scenariosToRun.end(); ++citr) {
      std::string sFileName = citr->first;

      DecodedStream stream;

      r = stream.open_file(sFileName);
      if (!r) {
         string msg = string("Cannot open file ") + sFileName;
         LOG4CPLUS_FATAL(logger, msg);
         throw runtime_error(msg);
      }

      stream.set_echo(false); // default set to false, must turn it on in input file

      char CurrentPath[_MAX_PATH];

      getcwd(CurrentPath, _MAX_PATH);

      string cwd = CurrentPath;

      stream.set_Local_Path(cwd);

      std::shared_ptr<RunFileArchiveDirector> rfad = std::make_shared<RunFileArchiveDirector>();

      stream.set_Archive_Director(rfad);

      // Set scenario name here, before load, for time to go output.
      citr->second->SetScenarioName(sFileName); // sets the scenario name

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


void process_overall_output(InternalObserver &internal_observer) {

}
