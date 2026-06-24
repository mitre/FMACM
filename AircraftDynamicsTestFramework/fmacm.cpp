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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <loader/Loadable.h>
#include <loader/RunFileArchiveDirector.h>
#include <log4cplus/initializer.h>
#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cppmanifest/cppmanifest.h"
#include "framework/TestFrameworkScenario.h"
#include "public/Log4cplusSetup.h"
#include "public/ScenarioUtils.h"

#define _MAX_PATH 260

const std::vector<std::pair<std::string, std::shared_ptr<TestFrameworkScenario>>> LoadConfigurationFile(
      const std::string &filename);

void ProcessScenarioDescriptions(
      const std::vector<std::pair<std::string, std::shared_ptr<TestFrameworkScenario>>> &scenarios);

static log4cplus::Logger logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("main"));
const std::string VERSION_FLAG("--version");

int main(int argc, char *argv[]) {
   log4cplus::Initializer initializer;
   LoadLoggerProperties();
   LOG4CPLUS_INFO(logger, "running " << aaesim::cppmanifest::GetVersion());

   if (argc == 2) {
      std::string arg1(argv[1]);
      if (arg1 == VERSION_FLAG) {
         std::cout << "fmacm version " << aaesim::cppmanifest::GetVersion() << std::endl;
         return EXIT_SUCCESS;
      } else if (arg1 == aaesim::cppmanifest::BUILDINFO_CLI_FLAG) {
         std::cout << "fmacm build info:" << std::endl;
         aaesim::cppmanifest::PrintMetaData();
         return EXIT_SUCCESS;
      }
   } else {
      std::string error_msg = "Invalid number of command line arguments; only one is allowed.";
      LOG4CPLUS_FATAL(logger, error_msg);
      return EXIT_FAILURE;
   }

   std::string configuration_filename = argv[1];
   if (configuration_filename.empty()) {
      std::string error_msg = "No configuration file provided. Must provide a configuration file.";
      LOG4CPLUS_FATAL(logger, error_msg);
      return EXIT_FAILURE;
   }

   try {
      auto scenario_descriptions = LoadConfigurationFile(configuration_filename);
      ProcessScenarioDescriptions(scenario_descriptions);
      scenario_descriptions.clear();
   } catch (const std::exception &e) {
      std::string error_msg = "Error occurred while processing configuration file: " + std::string(e.what());
      LOG4CPLUS_FATAL(logger, error_msg);
      return EXIT_FAILURE;
   }

   return EXIT_SUCCESS;
}

const std::vector<std::pair<std::string, std::shared_ptr<TestFrameworkScenario>>> LoadConfigurationFile(
      const std::string &filename) {
   std::string configuration_filename("");
   if (filename.find('/') != std::string::npos) {
      configuration_filename = filename;
   } else {
      configuration_filename = "../Run_Files/";
      configuration_filename += filename.c_str();
   }

   FILE *fp;
   fp = fopen(configuration_filename.c_str(), "r");
   std::vector<std::pair<std::string, std::shared_ptr<TestFrameworkScenario>>> scenarios;
   if (fp == NULL) {
      std::string error_msg = "Configuration file " + configuration_filename + " not found.";
      LOG4CPLUS_FATAL(logger, error_msg);
      throw std::runtime_error(error_msg);
   } else {
      // Get scenario files for the run.
      int scenario_count;
      char scenario_filename[300];

      fscanf(fp, "%d", &scenario_count);
      for (int i = 0; i < scenario_count; ++i) {
         fscanf(fp, "%299s", scenario_filename);
         auto new_pair = std::make_pair(scenario_filename, std::make_shared<TestFrameworkScenario>());
         scenarios.push_back(new_pair);
      }
      fclose(fp);
   }
   return scenarios;
}

void ProcessScenarioDescriptions(
      const std::vector<std::pair<std::string, std::shared_ptr<TestFrameworkScenario>>> &scenarios) {
   char current_path[_MAX_PATH];
   getcwd(current_path, _MAX_PATH);
   const std::string cwd = current_path;

   auto scenario_runner =
         [cwd](const std::pair<std::string, std::shared_ptr<TestFrameworkScenario>> &scenario_description) {
            auto scenario_file_name = scenario_description.first;
            auto scenario = scenario_description.second;
            LOG4CPLUS_INFO(logger, "Processing Scenario File: " << scenario_file_name << std::endl);

            DecodedStream stream;
            bool r = stream.open_file(scenario_file_name);
            if (!r) {
               std::string msg = std::string("Cannot open file ") + scenario_file_name;
               LOG4CPLUS_FATAL(logger, msg);
               throw std::runtime_error(msg);
            }
            stream.set_echo(false);
            stream.set_Local_Path(FilePath(cwd));
            stream.set_Archive_Director(std::make_shared<RunFileArchiveDirector>());
            auto scenario_root_name = aaesim::open_source::ScenarioUtils::ResolveScenarioRootName(scenario_file_name);
            scenario->SetScenarioName(scenario_root_name);
            scenario->load(&stream);
            scenario->SimulateAllIterations();
         };
   std::for_each(scenarios.begin(), scenarios.end(), scenario_runner);
}
