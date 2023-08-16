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

#include "public/FlightDeckApplication.h"
#include "loader/Loadable.h"
#include "public/Guidance.h"
#include <scalar/Time.h>
#include <scalar/Speed.h>

class SpeedCommandsFromStaticData final : public aaesim::open_source::FlightDeckApplication {

  public:
   struct SpeedRecord {
      SpeedRecord() : simtime(Units::infinity()), speed_command(Units::infinity()) {}
      Units::SecondsTime simtime;
      Units::MetersPerSecondSpeed speed_command;
   };
   SpeedCommandsFromStaticData();
   SpeedCommandsFromStaticData(const std::vector<SpeedCommandsFromStaticData::SpeedRecord> &speed_data,
                               Units::Time pilot_delay_duration);
   ~SpeedCommandsFromStaticData() = default;

   void Initialize(aaesim::open_source::FlightDeckApplicationInitializer &initializer_visitor) override;

   aaesim::open_source::Guidance Update(const aaesim::open_source::SimulationTime &simtime,
                                        const aaesim::open_source::Guidance &current_guidance,
                                        const aaesim::open_source::DynamicsState &dynamics_state,
                                        const aaesim::open_source::AircraftState &own_state) override;

   bool IsActive() const override;

  private:
   static const int m_hist_len = 20;

   aaesim::open_source::Guidance Update(Units::Time time);

   std::vector<SpeedRecord> m_speed_data;
   double m_ias_hist[m_hist_len];
   Units::SecondsTime m_pilot_delay_seconds;
};
