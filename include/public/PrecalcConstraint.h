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

#pragma once
#include <scalar/Length.h>
#include <scalar/Speed.h>

#include <fstream>
#include <string>

namespace aaesim::open_source {

enum ActiveFlagType {
   UNSET = 0,
   BELOW_ALT_ON_SPEED = 1,
   AT_ALT_ON_SPEED,
   BELOW_ALT_SLOW,
   SEG_END_LOW_ALT,
   SEG_END_MID_ALT,
   AT_ALT_SLOW,
   SEG_END_AT_ALT,
   AT_250_BELOW_10K,
   AT_ALT_FAST
};

static std::string ActiveFlagAsString(const ActiveFlagType &flag) {
   switch (flag) {
      case UNSET:
         return "UNSET";
      case BELOW_ALT_ON_SPEED:
         return "BELOW_ALT_ON_SPEED";
      case AT_ALT_ON_SPEED:
         return "AT_ALT_ON_SPEED";
      case BELOW_ALT_SLOW:
         return "BELOW_ALT_SLOW";
      case SEG_END_LOW_ALT:
         return "SEG_END_LOW_ALT";
      case SEG_END_MID_ALT:
         return "SEG_END_MID_ALT";
      case AT_ALT_SLOW:
         return "AT_ALT_SLOW";
      case SEG_END_AT_ALT:
         return "SEG_END_AT_ALT";
      case AT_250_BELOW_10K:
         return "AT_250_BELOW_10K";
      case AT_ALT_FAST:
         return "AT_ALT_FAST";
      default:
         throw std::runtime_error("Developer Error: This should be impossible");
   }
};

struct PrecalcConstraint final {
   PrecalcConstraint() = default;
   ~PrecalcConstraint() = default;

   PrecalcConstraint &operator=(const PrecalcConstraint &obj);
   bool operator<(const PrecalcConstraint &obj) const;
   bool operator!=(const PrecalcConstraint &obj) const;
   bool operator==(const PrecalcConstraint &obj) const;

   Units::Length constraint_along_path_distance{Units::zero()};
   Units::Length constraint_altHi{Units::zero()};
   Units::Length constraint_altLow{Units::zero()};
   Units::Speed constraint_speedHi{Units::zero()};
   Units::Speed constraint_speedLow{Units::zero()};
   int index{-1};
   ActiveFlagType active_flag{ActiveFlagType::UNSET};
   bool violation_flag{false};
   bool is_last_constraint{false};
};
}  // namespace aaesim::open_source
