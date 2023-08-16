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

#include "framework/SpeedCommandsFromStaticData.h"

SpeedCommandsFromStaticData::SpeedCommandsFromStaticData() : m_speed_data(), m_ias_hist(), m_pilot_delay_seconds(0) {
   for (double &i : m_ias_hist) {
      i = 0.0;
   }
}

SpeedCommandsFromStaticData::SpeedCommandsFromStaticData(
      const std::vector<SpeedCommandsFromStaticData::SpeedRecord> &speed_data, Units::Time pilot_delay_duration) {
   m_speed_data = speed_data;
   m_pilot_delay_seconds = pilot_delay_duration;
}

aaesim::open_source::Guidance SpeedCommandsFromStaticData::Update(Units::Time time) {
   aaesim::open_source::Guidance guidance;
   guidance.SetValid(false);

   const Units::Time lookup_time = time - m_pilot_delay_seconds;
   if (m_speed_data[0].simtime > lookup_time) {
      return guidance;
   }

   const Units::Time final_time_available = m_speed_data.back().simtime;
   if (final_time_available < lookup_time) {
      return guidance;
   }

   int ix = 0;
   while ((ix < (m_speed_data.size() - 1)) && (m_speed_data[(ix + 1)].simtime < lookup_time)) {
      ++ix;
   }

   if (m_speed_data[(ix + 1)].simtime == lookup_time) {
      guidance.m_ias_command = Units::FeetPerSecondSpeed(m_speed_data[(ix + 1)].speed_command);
   } else {
      double pct = Units::SecondsTime(time - m_speed_data[ix].simtime).value() /
                   Units::SecondsTime(m_speed_data[(ix + 1)].simtime - m_speed_data[ix].simtime).value();

      Units::Speed interpolatedspeed =
            (1.0 - pct) * m_speed_data[ix].speed_command + pct * m_speed_data[(ix + 1)].speed_command;

      guidance.m_ias_command = Units::FeetPerSecondSpeed(interpolatedspeed);
   }
   guidance.SetValid(true);

   return guidance;
}

void SpeedCommandsFromStaticData::Initialize(
      aaesim::open_source::FlightDeckApplicationInitializer &initializer_visitor) {}

aaesim::open_source::Guidance SpeedCommandsFromStaticData::Update(
      const aaesim::open_source::SimulationTime &simtime, const aaesim::open_source::Guidance &current_guidance,
      const aaesim::open_source::DynamicsState &dynamics_state, const aaesim::open_source::AircraftState &own_state) {
   return Update(simtime.GetCurrentSimulationTime());
}

bool SpeedCommandsFromStaticData::IsActive() const { return true; }