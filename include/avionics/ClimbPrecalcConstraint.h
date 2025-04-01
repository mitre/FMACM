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

enum class ClimbActiveFlagType {
   UNSET = 0,
   BEFORE_CONSTRAINT_AT_ALT_ON_SPEED = 1,
   BEFORE_CONSTRAINT_AT_ALT_SLOW,
   BEFORE_CONSTRAINT_LOW_ON_SPEED,
   BEFORE_CONSTRAINT_LOW_SLOW,
   BEFORE_CONSTRAINT_HIGH_ON_SPEED,
   BEFORE_CONSTRAINT_HIGH_SLOW,
   PAST_CONSTRAINT_AT_ALT_ON_SPEED,
   PAST_CONSTRAINT_AT_ALT_SLOW,
   PAST_CONSTRAINT_LOW_ON_SPEED,
   PAST_CONSTRAINT_LOW_SLOW,
   PAST_CONSTRAINT_HIGH_ON_SPEED,
   PAST_CONSTRAINT_HIGH_SLOW,
   AIRSPACE_VIOLATION
};

bool operator<=(ClimbActiveFlagType l, ClimbActiveFlagType r);

class ClimbPrecalcConstraint {

  public:
   ClimbPrecalcConstraint();

   virtual ~ClimbPrecalcConstraint();

   ClimbPrecalcConstraint &operator=(const ClimbPrecalcConstraint &obj);

   bool operator<(const ClimbPrecalcConstraint &obj) const;

   bool operator!=(const ClimbPrecalcConstraint &obj) const;

   bool operator==(const ClimbPrecalcConstraint &obj) const;

   Units::Length constraint_along_path_distance;  // cumulative along-path distance of this constraint
   Units::Length constraint_altHi;                // altitude max constraints
   Units::Length constraint_altLow;               // altitude min constraints
   Units::Speed constraint_speedHi;               // speed max contraint
   Units::Speed constraint_speedLow;              // speed min contraint
   int index;
   ClimbActiveFlagType active_flag;
};

std::ostream &operator<<(std::ostream &out, const ClimbPrecalcConstraint &constraint);
