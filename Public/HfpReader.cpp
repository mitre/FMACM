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

/*
 * HfpReader.cpp
 *
 *  Created on: Sep 5, 2019
 *      Author: klewis
 */

#include "public/HfpReader.h"
#include <stdexcept>

using namespace std;

namespace testvector {

HfpReader::HfpReader(std::string file_name, int header_lines) :
         DataReader(file_name, 0, 0) {
   SetColumnIndexesFromHeader(header_lines);
}

HfpReader::HfpReader(std::shared_ptr<std::istream> input_stream, int header_lines) :
         DataReader(input_stream, 0, 0) {
   SetColumnIndexesFromHeader(header_lines);
}

HfpReader::~HfpReader() {
}

void HfpReader::SetColumnIndexesFromHeader(const int header_lines) {
   if (header_lines < 1) {
      throw logic_error("Headers are required for HfpReader.");
   }
   int line_number(0);
   Advance();  // first header line
   // Meaningful HPF headers are at least 4 characters; first column is normally 5.
   while (GetString(0).length() < 3 && line_number < header_lines) {
      line_number++;
      Advance();
   }
   if (line_number == header_lines) {
      throw runtime_error("Suitable header line not found.");
   }

   BuildColumnIndex();

   // Record column indexes.
   m_x_column = GetColumnNumber("x[m]");
   m_y_column = GetColumnNumber("y[m]");
   m_dtg_column = GetColumnNumber("DTG[m]");
   m_segment_type_column = GetColumnNumber("Segment Type");
   m_course_column = GetColumnNumber("Course[rad]");
   m_turn_center_x_column = GetColumnNumber("Turn Center x[m]");
   m_turn_center_y_column = GetColumnNumber("Turn Center y[m]");
   m_angle_start_of_turn_column = GetColumnNumber("Angle Start of Turn[rad]");
   m_angle_end_of_turn_column = GetColumnNumber("Angle End of Turn[rad]");
   m_turn_radius_column = GetColumnNumber("R[m]");
   m_ground_speed_column = GetColumnNumber("groundspeed_mps");
   m_bank_angle_column = GetColumnNumber("bank_angle_deg");
   m_latitude_column = GetColumnNumber("Lat[deg]");
   m_longitude_column = GetColumnNumber("Lon[deg]");
   m_turn_center_latitude_column = GetColumnNumber("Turn Center Lat[deg]");
   m_turn_center_longitude_column = GetColumnNumber("Turn Center Lon[deg]");

   SetExpectedColumnCount(GetColumnCount());

   // Assume the next line is numeric data,
   // even if we didn't read the expected number of header lines.
}

Units::Length HfpReader::GetX() {
   return Units::MetersLength(GetDouble(m_x_column));
}

Units::Length HfpReader::GetY() {
   return Units::MetersLength(GetDouble(m_y_column));
}

Units::Length HfpReader::GetDTG() {
   return Units::MetersLength(GetDouble(m_dtg_column));
}

std::string HfpReader::GetSegmentType() {
   return GetString(m_segment_type_column);
}

Units::Angle HfpReader::GetCourse() {
   return Units::RadiansAngle(GetDouble(m_course_column));
}

Units::Length HfpReader::GetTurnCenterX() {
   return Units::MetersLength(GetDouble(m_turn_center_x_column));
}

Units::Length HfpReader::GetTurnCenterY() {
   return Units::MetersLength(GetDouble(m_turn_center_y_column));
}

Units::Angle HfpReader::GetAngleStartOfTurn() {
   return Units::RadiansAngle(GetDouble(m_angle_start_of_turn_column));
}

Units::Angle HfpReader::GetAngleEndOfTurn() {
   return Units::RadiansAngle(GetDouble(m_angle_end_of_turn_column));
}

Units::Length HfpReader::GetTurnRadius() {
   return Units::MetersLength(GetDouble(m_turn_radius_column));
}

Units::Speed HfpReader::GetGroundSpeed() {
   return Units::MetersPerSecondSpeed(GetDouble(m_ground_speed_column));
}

Units::Angle HfpReader::GetBankAngle() {
   return Units::DegreesAngle(GetDouble(m_bank_angle_column));
}

Units::Angle HfpReader::GetLatitude() {
   return Units::DegreesAngle(GetDouble(m_latitude_column));
}

Units::Angle HfpReader::GetLongitude() {
   return Units::DegreesAngle(GetDouble(m_longitude_column));
}

Units::Angle HfpReader::GetTurnCenterLatitude() {
   return Units::DegreesAngle(GetDouble(m_turn_center_latitude_column));
}

Units::Angle HfpReader::GetTurnCenterLongitude() {
   return Units::DegreesAngle(GetDouble(m_turn_center_longitude_column));
}

} // namespace testvector
