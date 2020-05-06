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
 * HfpReaderPre2020.cpp
 *
 *  Created on: Sep 5, 2019
 *      Author: klewis
 */

#include "public/HfpReaderPre2020.h"
#include <stdexcept>

using namespace std;

namespace testvector {

HfpReaderPre2020::HfpReaderPre2020(std::string file_name, int header_lines) :
         DataReader(file_name, 0, 0) {
   SetColumnIndexesFromHeader(header_lines);
}

HfpReaderPre2020::HfpReaderPre2020(std::shared_ptr<std::istream> input_stream, int header_lines) :
         DataReader(input_stream, 0, 0) {
   SetColumnIndexesFromHeader(header_lines);
}

HfpReaderPre2020::~HfpReaderPre2020() {
}

void HfpReaderPre2020::SetColumnIndexesFromHeader(const int header_lines) {
   if (header_lines < 1) {
      throw logic_error("Headers are required for HfpReaderPre2020.");
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

Units::Length HfpReaderPre2020::GetX() {
   return Units::MetersLength(GetDouble(m_x_column));
}

Units::Length HfpReaderPre2020::GetY() {
   return Units::MetersLength(GetDouble(m_y_column));
}

Units::Length HfpReaderPre2020::GetDTG() {
   return Units::MetersLength(GetDouble(m_dtg_column));
}

std::string HfpReaderPre2020::GetSegmentType() {
   return GetString(m_segment_type_column);
}

Units::Angle HfpReaderPre2020::GetCourse() {
   return Units::RadiansAngle(GetDouble(m_course_column));
}

Units::Length HfpReaderPre2020::GetTurnCenterX() {
   return Units::MetersLength(GetDouble(m_turn_center_x_column));
}

Units::Length HfpReaderPre2020::GetTurnCenterY() {
   return Units::MetersLength(GetDouble(m_turn_center_y_column));
}

Units::Angle HfpReaderPre2020::GetAngleStartOfTurn() {
   return Units::RadiansAngle(GetDouble(m_angle_start_of_turn_column));
}

Units::Angle HfpReaderPre2020::GetAngleEndOfTurn() {
   return Units::RadiansAngle(GetDouble(m_angle_end_of_turn_column));
}

Units::Length HfpReaderPre2020::GetTurnRadius() {
   return Units::MetersLength(GetDouble(m_turn_radius_column));
}

Units::Speed HfpReaderPre2020::GetGroundSpeed() {
   return Units::MetersPerSecondSpeed(GetDouble(m_ground_speed_column));
}

Units::Angle HfpReaderPre2020::GetBankAngle() {
   return Units::DegreesAngle(GetDouble(m_bank_angle_column));
}

Units::Angle HfpReaderPre2020::GetLatitude() {
   return Units::DegreesAngle(GetDouble(m_latitude_column));
}

Units::Angle HfpReaderPre2020::GetLongitude() {
   return Units::DegreesAngle(GetDouble(m_longitude_column));
}

Units::Angle HfpReaderPre2020::GetTurnCenterLatitude() {
   return Units::DegreesAngle(GetDouble(m_turn_center_latitude_column));
}

Units::Angle HfpReaderPre2020::GetTurnCenterLongitude() {
   return Units::DegreesAngle(GetDouble(m_turn_center_longitude_column));
}

} // namespace testvector
