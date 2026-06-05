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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/PrecalcConstraint.h"

using namespace aaesim::open_source;

bool operator<=(ActiveFlagType l, ActiveFlagType r) { return ((static_cast<int>(l)) <= (static_cast<int>(r))); }

PrecalcConstraint &PrecalcConstraint::operator=(const PrecalcConstraint &obj) {
   if (this != &obj) {
      constraint_along_path_distance = obj.constraint_along_path_distance;
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
   return constraint_along_path_distance < obj.constraint_along_path_distance;
}

bool PrecalcConstraint::operator==(const PrecalcConstraint &obj) const {
   bool match = (constraint_along_path_distance == obj.constraint_along_path_distance);

   match = match && (constraint_altHi == obj.constraint_altHi);
   match = match && (constraint_altLow == obj.constraint_altLow);

   match = match && (constraint_speedHi == obj.constraint_speedHi);
   match = match && (constraint_speedLow == obj.constraint_speedLow);

   match = match && (index == obj.index);

   match = match && (active_flag == obj.active_flag);
   match = match && (violation_flag == obj.violation_flag);

   return match;
}

bool PrecalcConstraint::operator!=(const PrecalcConstraint &obj) const { return !operator==(obj); }
