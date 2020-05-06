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
 * HfpReaderPre2020.h
 *
 * Reads a TV.csv file containing a sequence of aircraft states.
 *
 *  Created on: Sep 5, 2019
 *      Author: klewis
 */

#pragma once

#include "public/DataReader.h"
#include <Length.h>
#include <Angle.h>
#include <Speed.h>

namespace testvector {

class HfpReaderPre2020 : public DataReader {
public:
   static const size_t EXPECTED_TV_COLUMN_COUNT;
   HfpReaderPre2020(std::string file_name, int header_lines);
   HfpReaderPre2020(std::shared_ptr<std::istream> input_stream, int header_lines);
   HfpReaderPre2020();
   virtual ~HfpReaderPre2020();
   Units::Length GetX();
   Units::Length GetY();
   Units::Length GetDTG();
   std::string GetSegmentType();
   Units::Angle GetCourse();
   Units::Length GetTurnCenterX();
   Units::Length GetTurnCenterY();
   Units::Angle GetAngleStartOfTurn();
   Units::Angle GetAngleEndOfTurn();
   Units::Length GetTurnRadius();
   Units::Speed GetGroundSpeed();
   Units::Angle GetBankAngle();
   Units::Angle GetLatitude();
   Units::Angle GetLongitude();
   Units::Angle GetTurnCenterLatitude();
   Units::Angle GetTurnCenterLongitude();

private:
   void SetColumnIndexesFromHeader(const int header_lines);
   int m_x_column, m_y_column;
   int m_dtg_column;
   int m_segment_type_column;
   int m_course_column;
   int m_turn_center_x_column, m_turn_center_y_column;
   int m_angle_start_of_turn_column, m_angle_end_of_turn_column;
   int m_turn_radius_column;
   int m_ground_speed_column;
   int m_bank_angle_column;
   int m_latitude_column, m_longitude_column;
   int m_turn_center_latitude_column, m_turn_center_longitude_column;
};

} // namespace testvector

