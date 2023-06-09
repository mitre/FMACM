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
#include <scalar/Length.h>
#include <scalar/Angle.h>
#include <scalar/Speed.h>

namespace testvector {

class HfpReaderPre2020 : public aaesim::open_source::DataReader {
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

}  // namespace testvector
