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

#pragma once
#include <fstream>
#include <scalar/Length.h>
#include <scalar/Speed.h>

enum class ActiveFlagType {
   UNSET = 0,
   BELOW_ALT_ON_SPEED = 1,
   AT_ALT_ON_SPEED,
   BELOW_ALT_SLOW,
   SEG_END_LOW_ALT,
   SEG_END_MID_ALT,
   AT_ALT_SLOW,
   SEG_END_AT_ALT,
   AT_250_BELOW_10K
};

bool operator<=(ActiveFlagType l, ActiveFlagType r);

class PrecalcConstraint {

  public:
   PrecalcConstraint();

   virtual ~PrecalcConstraint();

   PrecalcConstraint &operator=(const PrecalcConstraint &obj);

   bool operator<(const PrecalcConstraint &obj) const;

   bool operator!=(const PrecalcConstraint &obj) const;

   bool operator==(const PrecalcConstraint &obj) const;

   Units::Length constraint_along_path_distance;  // cumulative along-path distance of this constraint
   Units::Length constraint_altHi;                // altitude max constraints
   Units::Length constraint_altLow;               // altitude min constraints
   Units::Speed constraint_speedHi;               // speed max contraint
   Units::Speed constraint_speedLow;              // speed min contraint
   int index;
   ActiveFlagType active_flag;
   bool violation_flag;
};

std::ostream &operator<<(std::ostream &out, const PrecalcConstraint &constraint);
