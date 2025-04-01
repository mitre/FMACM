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

#include "public/ADSBReceiver.h"
#include "public/ADSBTransmitter.h"
#include "public/LoggingLoadable.h"
#include "public/ADSBTransmitter.h"
#include "scalar/Length.h"
#include "avionics/ADSBReceiver.h"
#include "aaesim/AdsbTransmitterLoader.h"

namespace aaesim {
namespace loaders {
class AdsbDeviceLoader final : public LoggingLoadable {
  public:
   AdsbDeviceLoader() = default;
   std::shared_ptr<aaesim::open_source::ADSBReceiver> BuildReceiverModel(
         const Units::Length adsb_reception_range_threshold);
   std::shared_ptr<aaesim::open_source::ADSBTransmitter> BuildTransmitterModel(
         std::list<Waypoint> waypoints_along_route);

   bool load(DecodedStream *input) override;

  private:
   class PrivateDeviceLoader final : public LoggingLoadable {
     public:
      PrivateDeviceLoader() = default;
      bool load(DecodedStream *input) override;
      aaesim::avionics::ADSBReceiver m_loadable_receiver;
      aaesim::loaders::AdsbTransmitterLoader m_adsb_transmitter_loader;
   };
   PrivateDeviceLoader m_device_loader;
   bool m_is_loaded;
};

}  // namespace loaders
}  // namespace aaesim
