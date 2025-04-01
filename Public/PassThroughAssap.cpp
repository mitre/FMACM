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

#include "public/PassThroughAssap.h"

#include "public/NullADSBReceiver.h"

aaesim::open_source::PassThroughAssap::PassThroughAssap()
   : m_adsb_receiver(std::make_shared<aaesim::open_source::NullADSBReceiver>()) {}

aaesim::open_source::AircraftState aaesim::open_source::PassThroughAssap::Update(
      const aaesim::open_source::AircraftState &state_to_sync_with,
      const aaesim::open_source::ADSBSVReport &most_recent_ads_b) const {
   return aaesim::open_source::AircraftState::FromAdsbReport(most_recent_ads_b);
}

void aaesim::open_source::PassThroughAssap::Initialize(
      std::shared_ptr<const aaesim::open_source::ADSBReceiver> adsb_receiver) {
   m_adsb_receiver = adsb_receiver;
}