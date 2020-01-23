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

#include <Speed.h>
#include <UnsignedAngle.h>
#include <Length.h>

class HorizontalTurnPath
{
public:
   HorizontalTurnPath(void);

   ~HorizontalTurnPath(void);

   bool operator==(const HorizontalTurnPath &that) const;

   double x_position_meters;
   double y_position_meters;
   Units::UnsignedRadiansAngle q_start;
   Units::UnsignedRadiansAngle q_end;
   Units::MetersLength radius;
   Units::UnsignedRadiansAngle bankAngle;
   Units::MetersPerSecondSpeed groundspeed;
};

