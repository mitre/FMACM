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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <vector>
#include "public/AircraftState.h"
#include "public/ADSBSVReport.h"
#include "public/ADSBReceiver.h"

namespace aaesim {
namespace open_source {
struct ASSAP {  // Airborne Surveillance & Separation Assurance Processing
   virtual ~ASSAP() = default;

   virtual aaesim::open_source::AircraftState Update(const aaesim::open_source::AircraftState &state_to_sync_with,
                                                     const Sensor::ADSB::ADSBSVReport &most_recent_ads_b) const = 0;

   virtual std::shared_ptr<const aaesim::open_source::ADSBReceiver> GetAdsbReceiver() const = 0;

   virtual void Initialize(std::shared_ptr<const aaesim::open_source::ADSBReceiver> adsb_receiver) = 0;

   virtual const Units::SecondsTime GetMaxCoastTime() const = 0;
};
}  // namespace open_source
}  // namespace aaesim
