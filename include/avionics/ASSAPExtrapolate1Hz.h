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

#include "public/ASSAP.h"

class ASSAPExtrapolate1Hz final : public aaesim::open_source::ASSAP {
  public:
   ASSAPExtrapolate1Hz(Units::Time maximum_coast_time);

   virtual aaesim::open_source::AircraftState Update(
         const aaesim::open_source::AircraftState &state_to_sync_with,
         const aaesim::open_source::ADSBSVReport &most_recent_ads_b) const override;

   void Initialize(std::shared_ptr<const aaesim::open_source::ADSBReceiver> adsb_receiver) override;

   virtual std::shared_ptr<const aaesim::open_source::ADSBReceiver> GetAdsbReceiver() const { return m_adsb_receiver; }

   virtual const Units::SecondsTime GetMaxCoastTime() const { return m_coast_time; }

  private:
   std::shared_ptr<const aaesim::open_source::ADSBReceiver> m_adsb_receiver;
   Units::Time m_coast_time;
   aaesim::open_source::AircraftState ExtrapolateForward1Hz(
         const aaesim::open_source::ADSBSVReport &received_adsb_report, Units::Time extrapolate_to_time) const;
};
