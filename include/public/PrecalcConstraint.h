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

#pragma once

// enum for active_flag  values from original code
enum class ActiveFlagType
{
   UNSET = 0,
   BELOW_ALT_ON_SPEED = 1,
   AT_ALT_ON_SPEED,
   BELOW_ALT_SLOW,
   SEG_END_LOW_ALT,
   SEG_END_MID_ALT,
   AT_ALT_SLOW,
   SEG_END_AT_ALT = 2
};


bool operator<=(ActiveFlagType l,
                ActiveFlagType r);


class PrecalcConstraint
{

public:

   PrecalcConstraint(void);

   ~PrecalcConstraint(void);

   PrecalcConstraint &operator=(const PrecalcConstraint &obj);

   bool operator<(const PrecalcConstraint &obj) const;

   bool operator!=(const PrecalcConstraint &obj) const;

   bool operator==(const PrecalcConstraint &obj) const;

   double constraint_dist; // distance constraints-meters.
   double constraint_altHi; // altitude max constraints-meters.
   double constraint_altLow; // altitude min constraints-meters.
   double constraint_speedHi; // speed max contraint-meters per second.
   double constraint_speedLow; // speed min contraint-meters per second.
   double index;
   ActiveFlagType active_flag;
   bool violation_flag;

};

