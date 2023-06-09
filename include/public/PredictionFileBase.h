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

#include <map>
#include <vector>

#include "utility/Logging.h"
#include "public/AircraftState.h"
#include "public/OutputHandler.h"
#include "public/VerticalPath.h"

#include <scalar/Time.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Angle.h>
#include <scalar/Temperature.h>

/**
 * Abstract class containing data objects and methods to create
 * forms of PredictedData.csv file.
 */

struct PredictionFileBase : public OutputHandler {
  public:
   PredictionFileBase(const std::string &file_suffix);

   struct PredictionData {
      enum DataSource {
         INVALID_SOURCE = -1,
         FMS_DESCENT = 0,
         IM_ALGO_OWNSHIP,
         IM_ALGO_TARGET,
         FMS_ASCENT,
      };

      PredictionData()
         : iteration_number(-1),
           simulation_time(Units::SecondsTime(-1.0)),
           acid(),
           source(PredictionData::INVALID_SOURCE),
           altitude(Units::MetersLength(-1.0)),
           IAS(Units::MetersPerSecondSpeed(-1.0)),
           GS(Units::MetersPerSecondSpeed(-1.0)),
           TAS(Units::MetersPerSecondSpeed(-1.0)),
           time_to_go(Units::SecondsTime(-1.0)),
           distance_to_go(Units::MetersLength(-1.0)),
           VwePred(Units::MetersPerSecondSpeed(-1.0)),
           VwnPred(Units::MetersPerSecondSpeed(-1.0)),
           algorithm(VerticalPath::PredictionAlgorithmType::UNDETERMINED),
           flap_setting(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED){};

      int iteration_number;
      Units::Time simulation_time;
      std::string acid;

      DataSource source;
      Units::Length altitude;
      Units::Speed IAS;
      Units::Speed GS;
      Units::Speed TAS;
      Units::Time time_to_go;
      Units::Length distance_to_go;

      // Predicted wind components
      Units::MetersPerSecondSpeed VwePred;
      Units::MetersPerSecondSpeed VwnPred;

      VerticalPath::PredictionAlgorithmType algorithm;

      aaesim::open_source::bada_utils::FlapConfiguration flap_setting;
   };

   std::vector<PredictionData> ExtractPredictionDataFromVerticalPath(const unsigned int &iteration,
                                                                     const Units::Time simulation_time,
                                                                     const std::string &acid,
                                                                     const VerticalPath &vertical_path,
                                                                     const PredictionData::DataSource &source);

   static log4cplus::Logger logger;
};
