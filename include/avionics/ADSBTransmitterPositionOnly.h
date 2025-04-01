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
// 2024 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/ADSBTransmitter.h"
#include "public/ADSBSVReport.h"
#include "public/AircraftState.h"
#include "public/TangentPlaneSequence.h"
#include "public/SimulationTime.h"
#include "avionics/Track.h"
#include "avionics/DelayBuffer.h"

namespace aaesim::avionics {

class ADSBTransmitterPositionOnly final : public aaesim::open_source::ADSBTransmitter {
  public:
   ADSBTransmitterPositionOnly(int nacp, int nacv, int nicp, int nicv, bool latency_is_known, int latency_factor,
                               bool use_latency_compensation, Units::SecondsTime adsb_update_period,
                               Units::SecondsTime adsb_std_dev_time_of_applicability);
   ~ADSBTransmitterPositionOnly() = default;

   void Initialize(const std::list<Waypoint> &waypoints_along_route) override;
   void Transmit(const aaesim::open_source::SimulationTime &simulation_time,
                 const aaesim::open_source::AircraftState &nav_measurement) override;

   const std::vector<aaesim::open_source::ADSBSVReport> &GetAllTransmissions() const override;

  private:
   inline static log4cplus::Logger m_logger{
         log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("ADSBTransmitterPositionOnly"))};

   bool IsTurning(const aaesim::open_source::AircraftState &state) const;

   Track x_track{2, 0, false};
   Track y_track{2, 0, false};
   Track z_track{2, 0, false};
   DelayBuffer<aaesim::open_source::AircraftState> m_transmit_latency_buffer{};
   std::vector<aaesim::open_source::ADSBSVReport> m_all_transmissions{};
   std::shared_ptr<TangentPlaneSequence> m_position_converter{};

   int m_nacp{0};
   int m_nacv{0};
   int m_nicp{0};
   int m_nicv{0};
   int m_latency_factor{0};
   int m_jitter_factor{0};
   bool m_latency_is_known{false};
   bool m_first_time_through{false};
   bool m_use_latency_compensation{false};

   Units::SecondsTime m_adsb_update_period{Units::SecondsTime(-1.0)};
   Units::SecondsTime m_adsb_std_dev_time_of_applicability{Units::SecondsTime(-1.0)};
};

inline const std::vector<aaesim::open_source::ADSBSVReport> &ADSBTransmitterPositionOnly::GetAllTransmissions() const {
   return m_all_transmissions;
}
}  // namespace aaesim::avionics