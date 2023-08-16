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

#include "framework/HfpReader2020.h"

#include <stdexcept>
#include <utility>
#include <algorithm>

#include <public/HorizontalPath.h>

using namespace testvector;

HfpReader2020::HfpReader2020(std::string file_name, int header_lines)
   : DataReader(std::move(file_name), 0, 0),
     m_x_column(),
     m_y_column(),
     m_dtg_column(),
     m_segment_type_column(),
     m_course_column(),
     m_turn_center_x_column(),
     m_turn_center_y_column(),
     m_angle_start_of_turn_column(),
     m_angle_end_of_turn_column(),
     m_turn_radius_column(),
     m_ground_speed_column(),
     m_bank_angle_column(),
     m_latitude_column(),
     m_longitude_column(),
     m_turn_center_latitude_column(),
     m_turn_center_longitude_column() {
   SetColumnIndexesFromHeader(header_lines);
}

HfpReader2020::HfpReader2020(std::shared_ptr<std::istream> input_stream, int header_lines)
   : DataReader(std::move(input_stream), 0, 0) {
   SetColumnIndexesFromHeader(header_lines);
}

HfpReader2020::~HfpReader2020() = default;

void HfpReader2020::SetColumnIndexesFromHeader(const int header_lines) {
   if (header_lines < 1) {
      throw std::logic_error("Headers are required for HfpReader2020.");
   }

   int line_number = 0;
   Advance();  // first header line

   // Meaningful HPF headers are at least 4 characters; first column is normally 5.
   while (GetString(0).length() < 3 && line_number < header_lines) {
      line_number++;
      Advance();
   }

   if (line_number == header_lines) {
      throw std::runtime_error("Suitable header line not found.");
   }

   BuildColumnIndex();

   // Record column indexes.
   m_x_column = GetColumnNumber("x[m]");
   m_y_column = GetColumnNumber("y[m]");
   m_dtg_column = GetColumnNumber("DTG[m]");
   m_segment_type_column = GetColumnNumber("Segment_Type");
   m_course_column = GetColumnNumber("Course[rad]");
   m_turn_center_x_column = GetColumnNumber("Turn_Center_x[m]");
   m_turn_center_y_column = GetColumnNumber("Turn_Center_y[m]");
   m_angle_start_of_turn_column = GetColumnNumber("Angle_Start_of_Turn[rad]");
   m_angle_end_of_turn_column = GetColumnNumber("Angle_End_of_Turn[rad]");
   m_turn_radius_column = GetColumnNumber("R[m]");
   m_ground_speed_column = GetColumnNumber("GS[m/s]");
   m_bank_angle_column = GetColumnNumber("Bank_Angle[deg]");
   m_latitude_column = GetColumnNumber("Lat[deg]");
   m_longitude_column = GetColumnNumber("Lon[deg]");
   m_turn_center_latitude_column = GetColumnNumber("Turn_Center_Lat[deg]");
   m_turn_center_longitude_column = GetColumnNumber("Turn_Center_Lon[deg]");

   SetExpectedColumnCount(GetColumnCount());
}

Units::Length HfpReader2020::GetX() { return Units::MetersLength(GetDouble(m_x_column)); }

Units::Length HfpReader2020::GetY() { return Units::MetersLength(GetDouble(m_y_column)); }

Units::Length HfpReader2020::GetDTG() { return Units::MetersLength(GetDouble(m_dtg_column)); }

HorizontalPath::SegmentType HfpReader2020::GetSegmentType() {
   std::string segment_type = GetString(m_segment_type_column);
   std::transform(segment_type.cbegin(), segment_type.cend(), segment_type.begin(),
                  [](unsigned char c) { return std::tolower(c); });
   if (segment_type == "straight") {
      return HorizontalPath::STRAIGHT;
   } else if (segment_type == "turn") {
      return HorizontalPath::TURN;
   } else {
      return HorizontalPath::UNSET;
   }
}

Units::UnsignedAngle HfpReader2020::GetCourse() { return Units::RadiansAngle(GetDouble(m_course_column)); }

Units::Length HfpReader2020::GetTurnCenterX() { return Units::MetersLength(GetDouble(m_turn_center_x_column)); }

Units::Length HfpReader2020::GetTurnCenterY() { return Units::MetersLength(GetDouble(m_turn_center_y_column)); }

Units::UnsignedAngle HfpReader2020::GetAngleStartOfTurn() {
   return Units::RadiansAngle(GetDouble(m_angle_start_of_turn_column));
}

Units::UnsignedAngle HfpReader2020::GetAngleEndOfTurn() {
   return Units::RadiansAngle(GetDouble(m_angle_end_of_turn_column));
}

Units::Length HfpReader2020::GetTurnRadius() { return Units::MetersLength(GetDouble(m_turn_radius_column)); }

Units::Speed HfpReader2020::GetGroundSpeed() { return Units::MetersPerSecondSpeed(GetDouble(m_ground_speed_column)); }

Units::Angle HfpReader2020::GetBankAngle() { return Units::DegreesAngle(GetDouble(m_bank_angle_column)); }

Units::Angle HfpReader2020::GetLatitude() { return Units::DegreesAngle(GetDouble(m_latitude_column)); }

Units::Angle HfpReader2020::GetLongitude() { return Units::DegreesAngle(GetDouble(m_longitude_column)); }

Units::Angle HfpReader2020::GetTurnCenterLatitude() {
   return Units::DegreesAngle(GetDouble(m_turn_center_latitude_column));
}

Units::Angle HfpReader2020::GetTurnCenterLongitude() {
   return Units::DegreesAngle(GetDouble(m_turn_center_longitude_column));
}
