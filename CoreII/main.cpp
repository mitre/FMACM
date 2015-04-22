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
// Copyright 2015 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "RunFile.h"
#include "InternalObserver.h"
#include "Scenario.h"
#include "TestFrameworkScenario.h"
#include "RunFileArchiveDirector.h"
#include "Loadable.h"
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


void read_runfile(RunFile &run_file, std::string str);
void process_scenarios(RunFile &run_file);
void process_overall_output(InternalObserver &internal_observer);


int main(int argc, char* argv[])
{
   RunFile run_file;
   std::string arg2("");
   if(argc == 2)
      arg2 = argv[1];
   read_runfile(run_file, arg2);

   process_scenarios(run_file);
}

void read_runfile(RunFile &run_file, std::string arg)
{
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

  if (fp == NULL)
    printf("Configuration file %s not found.", configFile.c_str());

  else {
    // Get scenario files for the run.

    int numScenarios;
    char scenarioFileName[300];

    fscanf(fp, "%d", &numScenarios);
    for(int i=0; i<numScenarios; i++) {
      fscanf(fp, "%s", scenarioFileName);
      run_file.list_of_scenario_input_files_full_name.push_back(scenarioFileName);
    }
    fclose(fp);
  }
}

void process_scenarios(RunFile &run_file)
{
   bool r ;

   list<string>::iterator i;

   //iterating through all the scenarios:
   int scen_num = 0;
   for (i = run_file.list_of_scenario_input_files_full_name.begin(); i != run_file.list_of_scenario_input_files_full_name.end(); ++i)
   {
      string scenario_file_name = (*i).data();

      //scenario.load_one_scenario_from_scenario_file_to_scenario_class(scenario_file_name);
      DecodedStream stream;
      r = stream.open_file(scenario_file_name);
   
      if(!r)
      {
         stream.report_error("cant open file\n");
         exit(-60);
      }

      stream.set_echo(false); // default set to false, must turn it on in input file
      //stream.open_dump_file("echo_file.html"); // dump the output to echo_file.txt as you are reading it 

         
      char CurrentPath[_MAX_PATH];
      
      getcwd(CurrentPath, _MAX_PATH);
     
      string cwd = CurrentPath;

      stream.set_Local_Path(cwd);

      RunFileArchiveDirector *rfad = new RunFileArchiveDirector;
      
      stream.set_Archive_Director(rfad);

      /*
       * For now, allow hard-coding the specific scenario to run. In the future,
       * it would be nice to select based on some run-time input. But, no obvious path
       * to allowing this currently. Punting on this design issue.
       */
      TestFrameworkScenario testAcDynamicsScenario;
      Scenario *scenario = &testAcDynamicsScenario;

      // Set scenario name here, before load, for time to go output.
      scenario->set_scenario_name(scenario_file_name); // sets the scenario name

      // Load the parameters for this scenario
      scenario->load(&stream);

      // Process the loaded scenario
      scenario->process_one_scenario();
      
      scen_num++;

      delete rfad; // frees the RunFileArchiveDirector
   }
}

void process_overall_output(InternalObserver &internal_observer)
{

}
