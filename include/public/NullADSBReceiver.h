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

namespace aaesim {
namespace open_source {
class NullADSBReceiver : public ADSBReceiver {
  public:
   NullADSBReceiver() = default;
   virtual ~NullADSBReceiver() = default;
   Sensor::ADSB::ADSBSVReport GetCurrentADSBReport(int id) const override {
      return Sensor::ADSB::ADSBSVReport::blank_report;
   }
   Sensor::ADSB::ADSBSVReport GetADSBReportBefore(int id, double time) const override {
      return Sensor::ADSB::ADSBSVReport::blank_report;
   }
   const std::vector<Sensor::ADSB::ADSBSVReport> &GetReportsReceivedByTime(const SimulationTime &time) const override {
      static std::vector<Sensor::ADSB::ADSBSVReport> empty;
      return empty;
   }
   const std::map<int, std::vector<Sensor::ADSB::ADSBSVReport> > &GetAllReportsReceived() const override {
      static std::map<int, std::vector<Sensor::ADSB::ADSBSVReport> > empty;
      return empty;
   }
   std::map<int, Sensor::ADSB::ADSBSVReport> const GetCurrentADSBReport() const override {
      static std::map<int, Sensor::ADSB::ADSBSVReport> empty;
      return empty;
   }
   void Initialize(Units::Length adsb_reception_range_threshold) override {}
   std::map<int, Sensor::ADSB::ADSBSVReport> Receive(const aaesim::open_source::SimulationTime &time,
                                                     const aaesim::open_source::AircraftState &state) override {
      static std::map<int, Sensor::ADSB::ADSBSVReport> empty;
      return empty;
   }
};
}  // namespace open_source

}  // namespace aaesim
