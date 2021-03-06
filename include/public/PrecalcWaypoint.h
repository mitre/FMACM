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

#pragma once

#include "public/LoggingLoadable.h"
#include "public/PrecalcConstraint.h"
#include "UnsignedAngle.h"

class PrecalcWaypoint : public LoggingLoadable
{
public:
   PrecalcWaypoint();

   virtual ~PrecalcWaypoint();

   bool operator==(const PrecalcWaypoint &obj) const;

   bool load(DecodedStream *input);

   std::string m_name;

   Units::Length m_leg_length;
   Units::UnsignedRadiansAngle m_course_angle;

   Units::MetersLength m_x_pos_meters;
   Units::MetersLength m_y_pos_meters;

   Units::MetersLength m_rf_leg_center_x;
   Units::MetersLength m_rf_leg_center_y;
   Units::MetersLength m_radius_rf_leg;

   Units::RadiansAngle m_bank_angle;
   Units::MetersPerSecondSpeed m_ground_speed;

   PrecalcConstraint m_precalc_constraints;

private:
   bool m_loaded;
};

