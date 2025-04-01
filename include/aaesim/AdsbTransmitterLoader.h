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

#include "public/LoggingLoadable.h"
#include "public/ADSBTransmitter.h"

namespace aaesim::loaders {

enum AdsbSystemType { COMPACT_POSITION_REPORT, FULL_STATE_UPDATE, POSITION_ONLY, TIME_LATENCY };
class AdsbTransmitterLoader final : public LoggingLoadable {

  public:
   AdsbTransmitterLoader();

   bool load(DecodedStream *input);

   std::shared_ptr<aaesim::open_source::ADSBTransmitter> BuildAdsbTransmitter();

   bool IsLoaded() const;

  private:
   static log4cplus::Logger m_logger;

   int m_nacp{0};
   int m_nacv{0};
   int m_nicp{0};
   int m_nicv{0};

   int m_latency_factor{0};
   bool m_latency_is_known{false};
   bool m_first_time_through{false};
   bool m_use_latency_compensation{false};
   bool m_is_loaded{false};

   AdsbSystemType m_adsb_system_type{AdsbSystemType::COMPACT_POSITION_REPORT};
   Units::SecondsTime m_adsb_update_period{Units::SecondsTime(-1.0)};
   Units::SecondsTime m_adsb_std_dev_time_of_applicability{Units::SecondsTime(-1.0)};
};
}  // namespace aaesim::loaders