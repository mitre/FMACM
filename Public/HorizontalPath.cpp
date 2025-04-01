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

#include "public/HorizontalPath.h"

double aaesim::open_source::HorizontalPath::GetXPositionMeters() const { return m_x_position_meters; }

double aaesim::open_source::HorizontalPath::GetYPositionMeters() const { return m_y_position_meters; }

void aaesim::open_source::HorizontalPath::SetXYPositionMeters(double x_position_meters, double y_position_meters) {
   m_x_position_meters = x_position_meters;
   m_y_position_meters = y_position_meters;
}

bool aaesim::open_source::HorizontalPath::operator==(const HorizontalPath &that) const {
   return (this->m_x_position_meters == that.m_x_position_meters) &&
          (this->m_y_position_meters == that.m_y_position_meters) && (this->m_segment_type == that.m_segment_type);
}