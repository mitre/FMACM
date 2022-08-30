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

#include <utility>

#include <utility>

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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/VerticalPathObserver.h"
#include "utility/constants.h"
#include <iomanip>
#include <log4cplus/loggingmacros.h>

using std::string;
using std::cout;
using std::endl;

using namespace aaesim::constants;
log4cplus::Logger VerticalPathObserver::m_logger = log4cplus::Logger::getInstance("VerticalPathObserver");

VerticalPathObserver::VerticalPathObserver()
      : m_scenario_name(),
        m_file_name(),
        m_column_header() {
   m_iteration = -1;
   m_is_target_aircraft_data = false;
}


VerticalPathObserver::VerticalPathObserver(string scenario_name,
                                           string file_name,
                                           bool is_target_aircraft_data)
      : m_scenario_name(std::move(scenario_name)),
        m_file_name(std::move(file_name)) {


   m_iteration = -1;
   m_is_target_aircraft_data = is_target_aircraft_data;

   m_column_header = GetHeader();

   Initialize();
}

VerticalPathObserver::~VerticalPathObserver() = default;

void VerticalPathObserver::Initialize() {
   string fullFileName = CreateFullFileName(m_scenario_name, m_file_name);

   out_stream.open(fullFileName.c_str());

   if (out_stream.is_open()) {
      out_stream << m_column_header << endl;
   } else {
      LOG4CPLUS_ERROR(m_logger, "Cannot open file for output: " + fullFileName);
   }
}

void VerticalPathObserver::AddTrajectory(int id,
                                         const VerticalPath& vertical_path) {
   if (out_stream.is_open()) {
      for (unsigned int i = 0; i < vertical_path.along_path_distance_m.size(); i++) {
         out_stream << m_iteration << ",";
         out_stream << id << ",";
         out_stream << vertical_path.time_to_go_sec[i] << ",";
         out_stream << vertical_path.along_path_distance_m[i] / FEET_TO_METERS << ",";
         out_stream << vertical_path.altitude_m[i] / FEET_TO_METERS << ",";
         out_stream << vertical_path.cas_mps[i] / KNOTS_TO_METERS_PER_SECOND << ",";
         out_stream << vertical_path.altitude_rate_mps[i] << ",";
         out_stream << vertical_path.tas_rate_mps[i] << ",";
         out_stream << vertical_path.theta_radians[i] << ",";
         out_stream << Units::KnotsSpeed(Units::MetersPerSecondSpeed(vertical_path.gs_mps[i])).value() << ",";
         out_stream << vertical_path.mass_kg[i] << ",";
         out_stream << vertical_path.algorithm_type[i] << endl;
      }
   }
}

string VerticalPathObserver::CreateFullFileName(const string& scenario_name,
                                                const string& file_name) {
   return scenario_name + "_" + file_name + ".csv";
}

string VerticalPathObserver::GetHeader() {
   string hdr = "Iteration,";

   if (m_is_target_aircraft_data) {
      hdr += "target AC_ID,";
   } else {
      hdr += "AC_ID,";
   }

   hdr += "Time,Distance(feet),Altitude(Feet),IAS_Speed(Knots),Altitude_Change,Velocity_Change,Theta,GroundSpeed(Knots),"
          "Mass,Algorithm";

   return hdr;
}

void VerticalPathObserver::WriteData() {
   if (out_stream.is_open()) {
      out_stream.close();
   }
}
