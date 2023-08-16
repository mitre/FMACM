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
#include "public/TangentPlaneSequence.h"

namespace fmacm {
class PreloadedAdsbReceiver final : public aaesim::open_source::ADSBReceiver {
  public:
   PreloadedAdsbReceiver() = default;
   PreloadedAdsbReceiver(std::string ttv_csv_file, std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence);

   Sensor::ADSB::ADSBSVReport GetCurrentADSBReport(int id) const override;
   Sensor::ADSB::ADSBSVReport GetADSBReportBefore(int id, double time) const override;
   const std::vector<Sensor::ADSB::ADSBSVReport> &GetReportsReceivedByTime(
         const aaesim::open_source::SimulationTime &time) const override;
   const std::map<int, std::vector<Sensor::ADSB::ADSBSVReport> > &GetAllReportsReceived() const override;
   std::map<int, Sensor::ADSB::ADSBSVReport> const GetCurrentADSBReport() const override;
   void Initialize(Units::Length adsb_reception_range_threshold) override;
   std::map<int, Sensor::ADSB::ADSBSVReport> Receive(const aaesim::open_source::SimulationTime &time,
                                                     const aaesim::open_source::AircraftState &state) override;

  private:
   std::string m_ttv_filename;
   std::shared_ptr<TangentPlaneSequence> m_tanget_plane_sequence;
   std::map<int, Sensor::ADSB::ADSBSVReport> m_most_recent_adsb_reports;                // ADSB data stored as
                                                                                        // [numericFlightID][Report]
   std::map<int, std::vector<Sensor::ADSB::ADSBSVReport> > m_all_reports_received;      // ADSB data stored as
                                                                                        // [numericFlightID][Report]
   std::map<int, std::vector<Sensor::ADSB::ADSBSVReport> > m_reports_received_by_time;  // ADSB reports received stored
                                                                                        // as [time][report_vector]
};
}  // namespace fmacm
