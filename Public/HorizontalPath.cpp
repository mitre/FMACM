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

#include "public/HorizontalPath.h"


HorizontalPath::HorizontalPath()
{
   m_segment_type = HorizontalPath::SegmentType::UNSET;
   m_x_position_meters = 0;
   m_y_position_meters = 0;
   m_path_length_cumulative_meters = 0;
   m_path_course = 0;
}

bool HorizontalPath::operator==(const HorizontalPath &that) const {
   return ((this->m_x_position_meters == that.m_x_position_meters) &&
           (this->m_y_position_meters == that.m_y_position_meters) &&
           (this->m_segment_type == that.m_segment_type) &&
           (this->m_path_length_cumulative_meters == that.m_path_length_cumulative_meters) &&
           (this->m_path_course == that.m_path_course) &&
           ((this->m_segment_type == HorizontalPath::SegmentType::STRAIGHT) ||
            (this->m_turn_info == that.m_turn_info)));
}


HorizontalPath::~HorizontalPath() = default;
