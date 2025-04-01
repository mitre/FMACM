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

#include <scalar/Speed.h>
#include <scalar/UnsignedAngle.h>
#include <scalar/SignedAngle.h>
#include <scalar/Length.h>
#include "public/GeolibUtils.h"
#include "public/HorizontalTurnPath.h"

namespace aaesim {
class Wgs84HorizontalTurnPath final {
  public:
   Wgs84HorizontalTurnPath() = default;
   Wgs84HorizontalTurnPath(Units::UnsignedAngle bank, Units::Speed gs,
                           aaesim::open_source::HorizontalTurnPath::TURN_TYPE turn_type)
      : m_bank_angle(Units::UnsignedRadiansAngle(bank)),
        m_groundspeed(Units::MetersPerSecondSpeed(gs)),
        m_turn_type(turn_type){};

   virtual ~Wgs84HorizontalTurnPath(void) = default;

   bool operator==(const Wgs84HorizontalTurnPath &rhs) const;
   bool operator!=(const Wgs84HorizontalTurnPath &rhs) const;
   const Units::SignedAngle &GetBankAngle() const;
   const Units::Speed &GetGroundspeed() const;
   aaesim::open_source::HorizontalTurnPath::TURN_TYPE GetTurnType() const;

  private:
   Units::SignedAngle m_bank_angle{Units::zero()};
   Units::Speed m_groundspeed{Units::zero()};
   aaesim::open_source::HorizontalTurnPath::TURN_TYPE m_turn_type{aaesim::open_source::HorizontalTurnPath::UNKNOWN};
};

inline const Units::SignedAngle &Wgs84HorizontalTurnPath::GetBankAngle() const { return m_bank_angle; }

inline const Units::Speed &Wgs84HorizontalTurnPath::GetGroundspeed() const { return m_groundspeed; }

inline aaesim::open_source::HorizontalTurnPath::TURN_TYPE Wgs84HorizontalTurnPath::GetTurnType() const {
   return m_turn_type;
}

}  // namespace aaesim
