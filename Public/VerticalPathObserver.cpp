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

using namespace std;
using namespace aaesim::constants;

VerticalPathObserver::VerticalPathObserver()
      : m_scenario_name(), m_file_name(), m_column_header() {
   m_iteration = -1;
   m_is_target_aircraft_data = false;
}


VerticalPathObserver::VerticalPathObserver(const string &scenario_name,
                                           const string &file_name,
                                           bool is_target_aircraft_data)
      :m_scenario_name(scenario_name),
       m_file_name(file_name) {


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
      cout << "CANNOT OPEN TRAJECTORY FILE " << fullFileName << endl
           << "TRAJECTORY DATA WILL NOT BE WRITTEN" << endl << endl;
   }
}

void VerticalPathObserver::AddTrajectory(int id,
                                         const VerticalPath &vertical_path) {
   if (out_stream.is_open()) {
      for (unsigned int i = 0; i < vertical_path.x.size(); i++) {
         out_stream << m_iteration << ",";
         out_stream << id << ",";
         out_stream << vertical_path.time[i] << ",";
         out_stream << vertical_path.x[i] / FEET_TO_METERS << ",";
         out_stream << vertical_path.h[i] / FEET_TO_METERS << ",";
         out_stream << vertical_path.v[i] / KNOTS_TO_METERS_PER_SECOND << ",";
         out_stream << vertical_path.h_dot[i] << ",";
         out_stream << vertical_path.v_dot[i] << ",";
         out_stream << vertical_path.theta[i] << ",";
         out_stream << Units::KnotsSpeed(Units::MetersPerSecondSpeed(vertical_path.gs[i])).value() << ",";
         out_stream << vertical_path.mass[i] << endl;
      }
   }
}

string VerticalPathObserver::CreateFullFileName(const string &scenario_name,
                                                const string &file_name) {
   return scenario_name + "_" + file_name + ".csv";
}

string VerticalPathObserver::GetHeader() {
   string hdr = "Iteration,";

   if (m_is_target_aircraft_data) {
      hdr += "target AC_ID,";
   } else {
      hdr += "AC_ID,";
   }

   hdr += "Time,Distance(feet),Altitude(Feet),IAS_Speed(Knots),Altitude_Change,Velocity_Change,Theta,GroundSpeed(Knots),Mass";

   return hdr;
}

void VerticalPathObserver::WriteData() {
   if (out_stream.is_open()) {
      out_stream.close();
   }
}
