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

#include "public/PrecalcConstraint.h"

bool operator<=(ActiveFlagType l,
                ActiveFlagType r) {
   return ((static_cast<int> (l)) <= (static_cast<int> (r)));
}

PrecalcConstraint::PrecalcConstraint() {

   active_flag = ActiveFlagType::UNSET;
   constraint_dist = 0.0; // distance constraints-meters.
   constraint_altHi = 0.0; // altitude max constraints-meters.
   constraint_altLow = 0.0; // altitude min constraints-meters.
   constraint_speedHi = 0.0; // speed max constraint-meters per second.
   constraint_speedLow = 0.0; // speed min constraint-meters per second.
   index = -1;
   violation_flag = false;

}

PrecalcConstraint::~PrecalcConstraint() = default;

PrecalcConstraint &PrecalcConstraint::operator=(const PrecalcConstraint &obj) {
   if (this != &obj) {

      constraint_dist = obj.constraint_dist;
      constraint_altHi = obj.constraint_altHi;
      constraint_altLow = obj.constraint_altLow;
      constraint_speedHi = obj.constraint_speedHi;
      constraint_speedLow = obj.constraint_speedLow;
      index = obj.index;
      active_flag = obj.active_flag;
      violation_flag = obj.violation_flag;

   }
   return *this;
}

bool PrecalcConstraint::operator<(const PrecalcConstraint &obj) const {
   return constraint_dist < obj.constraint_dist;
}

bool PrecalcConstraint::operator==(const PrecalcConstraint &obj) const {
   bool match = (constraint_dist == obj.constraint_dist);

   match = match && (constraint_altHi == obj.constraint_altHi);
   match = match && (constraint_altLow == obj.constraint_altLow);

   match = match && (constraint_speedHi == obj.constraint_speedHi);
   match = match && (constraint_speedLow == obj.constraint_speedLow);

   match = match && (index == obj.index);

   match = match && (active_flag == obj.active_flag);
   match = match && (violation_flag == obj.violation_flag);

   return match;

}

bool PrecalcConstraint::operator!=(const PrecalcConstraint &obj) const {
   return !operator==(obj);
}


std::ostream& operator<<(std::ostream &out, const PrecalcConstraint &constraint) {
   out << "Constraint " << constraint.index <<
         " at " << constraint.constraint_dist <<
         ", active=" << static_cast<int>(constraint.active_flag) <<
         " alt between " << constraint.constraint_altLow <<
         " and " << constraint.constraint_altHi << " meters, " <<
         " speed between " << constraint.constraint_speedLow <<
         " and " << constraint.constraint_speedHi << " m/s, " <<
         " violation=" << constraint.violation_flag;
   return out;
}
