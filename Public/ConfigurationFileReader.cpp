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

#include "public/ConfigurationFileReader.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <unistd.h>

log4cplus::Logger aaesim::open_source::ConfigurationFileReader::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("ConfigurationFileReader"));

const std::vector<std::string> aaesim::open_source::ConfigurationFileReader::LoadConfigurationFile(
      std::string suggested_filename) {
   std::string configuration_filename("");
   if (suggested_filename.find('/') != std::string::npos) {
      configuration_filename = suggested_filename;
   } else {
      configuration_filename = "../Run_Files/";
      configuration_filename += suggested_filename.c_str();
   }

   FILE *fp;
   fp = fopen(configuration_filename.c_str(), "r");
   std::vector<std::string> scenario_filenames;
   if (fp == NULL) {
      std::string error_msg = "Configuration file " + configuration_filename + " not found.";
      LOG4CPLUS_FATAL(m_logger, error_msg);
      throw std::runtime_error(error_msg);
   } else {
      int scenario_count;
      char scenario_filename[300];
      fscanf(fp, "%d", &scenario_count);
      for (int i = 0; i < scenario_count; ++i) {
         fscanf(fp, "%299s", scenario_filename);
         scenario_filenames.push_back(scenario_filename);
      }
      fclose(fp);
   }
   return scenario_filenames;
}
