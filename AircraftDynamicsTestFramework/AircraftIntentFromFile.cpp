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

#include "framework/AircraftIntentFromFile.h"
#include "framework/HfpReader2020.h"

using std::string;

AircraftIntentFromFile::AircraftIntentFromFile() = default;

AircraftIntentFromFile::~AircraftIntentFromFile() = default;

bool AircraftIntentFromFile::load(DecodedStream *input) {
   set_stream(input);

   std::string csvfile;
   register_var("hfp_csv_file", &csvfile, true);

   bool loaded = complete();

   if (loaded) {
      PopulateWaypointsFromCsv(csvfile);
   }

   return loaded;
}

void AircraftIntentFromFile::PopulateWaypointsFromCsv(const std::string &csv_file) {
   testvector::HfpReader2020 hfp_reader(csv_file, 1);

   Units::MetersLength x_waypoint_location[128];
   Units::MetersLength y_waypoint_location[128];
   Units::MetersLength distance_to_go[128];
   Units::MetersLength x_turn_center[128];
   Units::MetersLength y_turn_center[128];
   Units::MetersLength turn_radius[128];

   Units::RadiansAngle course[128];
   Units::RadiansAngle start_of_turn[128];
   Units::RadiansAngle end_of_turn[128];

   Units::DegreesAngle waypoint_latitude[128];
   Units::DegreesAngle waypoint_longitude[128];
   Units::DegreesAngle turn_center_latitude[128];
   Units::DegreesAngle turn_center_longitude[128];

   int irow = -1;
   while (hfp_reader.Advance()) {
      irow++;

      // the value extracted from the CSV file is 1-based, we need 0-based
      int row_index = static_cast<int>(hfp_reader.GetDouble(0));

      x_waypoint_location[row_index] = hfp_reader.GetX();
      y_waypoint_location[row_index] = hfp_reader.GetY();

      distance_to_go[row_index] = hfp_reader.GetDTG();
      course[row_index] = hfp_reader.GetCourse();

      x_turn_center[row_index] = hfp_reader.GetTurnCenterX();
      y_turn_center[row_index] = hfp_reader.GetTurnCenterY();

      start_of_turn[row_index] = hfp_reader.GetAngleStartOfTurn();
      end_of_turn[row_index] = hfp_reader.GetAngleEndOfTurn();
      turn_radius[row_index] = hfp_reader.GetTurnRadius();

      waypoint_latitude[row_index] = hfp_reader.GetLatitude();
      waypoint_longitude[row_index] = hfp_reader.GetLongitude();

      turn_center_latitude[row_index] = hfp_reader.GetTurnCenterLatitude();
      turn_center_longitude[row_index] = hfp_reader.GetTurnCenterLongitude();
   }

   // Now store all information into the public Fms struct in the appropriate order
   int number_of_data_rows = irow + 1;
   int forward_row_index = 0;

   SetId(0);  // hardcoding in this test framework, must match the id in TestFrameworkAircraft.cpp
   SetNumberOfWaypoints(number_of_data_rows);
   for (int reverse_row_index = number_of_data_rows - 1; reverse_row_index >= 0; --reverse_row_index) {
      m_route_data.m_name.push_back(string("F") + string(std::to_string(forward_row_index)));
      m_route_data.m_latitude.push_back(waypoint_latitude[reverse_row_index]);
      m_route_data.m_longitude.push_back(waypoint_longitude[reverse_row_index]);
      m_route_data.m_x.push_back(x_waypoint_location[reverse_row_index]);
      m_route_data.m_y.push_back(y_waypoint_location[reverse_row_index]);
      forward_row_index++;
   }
}

double AircraftIntentFromFile::LocalStringToDouble(const string &s) {
   std::istringstream iss(s);
   double val = 0;
   iss >> val;
   return val;
}

int AircraftIntentFromFile::LocalStringToInt(const string &s) {
   std::istringstream iss(s);
   int val = 0;
   iss >> val;
   return val;
}

void AircraftIntentFromFile::update_xy_from_latlon() {
   // intentionally empty to prevent parent method from running
}
