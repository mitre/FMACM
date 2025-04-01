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

#include "framework/PreloadedAdsbReceiver.h"

#include "public/EarthModel.h"
#include "public/TvReader.h"

fmacm::PreloadedAdsbReceiver::PreloadedAdsbReceiver(std::string ttv_csv_file,
                                                    std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence) {
   m_ttv_filename = ttv_csv_file;
   m_tanget_plane_sequence = tangent_plane_sequence;
}

aaesim::open_source::ADSBSVReport fmacm::PreloadedAdsbReceiver::GetCurrentADSBReport(int id) const {
   if (m_most_recent_adsb_reports.empty()) return aaesim::open_source::ADSBSVReport::EMPTY_REPORT;
   return m_most_recent_adsb_reports.at(id);
}

aaesim::open_source::ADSBSVReport fmacm::PreloadedAdsbReceiver::GetADSBReportBefore(int id, Units::Time time) const {
   throw std::runtime_error("Unimplemented GetADSBReportBefore");
}

const std::vector<aaesim::open_source::ADSBSVReport> &fmacm::PreloadedAdsbReceiver::GetReportsReceivedByTime(
      const aaesim::open_source::SimulationTime &time) const {
   if (m_reports_received_by_time.empty() ||
       m_reports_received_by_time.find(time.GetCycle()) == m_reports_received_by_time.end()) {
      static std::vector<aaesim::open_source::ADSBSVReport> empty;
      return empty;
   }
   return m_reports_received_by_time.at(time.GetCycle());
}

const std::map<int, std::vector<aaesim::open_source::ADSBSVReport> > &
      fmacm::PreloadedAdsbReceiver::GetAllReportsReceived() const {
   return m_all_reports_received;
}

std::map<int, aaesim::open_source::ADSBSVReport> const fmacm::PreloadedAdsbReceiver::GetCurrentADSBReport() const {
   return m_most_recent_adsb_reports;
}

void fmacm::PreloadedAdsbReceiver::Initialize(Units::Length adsb_reception_range_threshold) {
   aaesim::open_source::TvReader data_reader(m_ttv_filename, 1);
   while (data_reader.Advance()) {
      EarthModel::GeodeticPosition geo_position;
      geo_position.latitude = data_reader.GetLat();
      geo_position.longitude = data_reader.GetLon();
      geo_position.altitude = data_reader.GetAlt();
      EarthModel::LocalPositionEnu local_position;
      m_tanget_plane_sequence->ConvertGeodeticToLocal(geo_position, local_position);

      const auto report =
            aaesim::open_source::ADSBSVReport::Builder(data_reader.GetAcid(), data_reader.GetTimeOfReceipt())
                  .Position(local_position.x, local_position.y)
                  ->GeodeticPosition(geo_position.latitude, geo_position.longitude)
                  ->AltitudeMsl(geo_position.altitude)
                  ->GroundSpeed(data_reader.GetEwvel(), data_reader.GetNsvel())
                  ->AltitudeRate(data_reader.GetVertRate())
                  ->NACp(data_reader.GetNacp())
                  ->NACv(data_reader.GetNacv())
                  ->NICp(data_reader.GetNic())
                  ->NICv(data_reader.GetNic())
                  ->Build();

      m_all_reports_received[report.GetId()].push_back(report);
      m_reports_received_by_time[(int)round(data_reader.GetTimeOfReceipt().value())].push_back(report);
   }
}

std::map<int, aaesim::open_source::ADSBSVReport> fmacm::PreloadedAdsbReceiver::Receive(
      const aaesim::open_source::SimulationTime &time, const aaesim::open_source::AircraftState &state) {
   auto surveillance_reports = GetReportsReceivedByTime(time);
   if (surveillance_reports.empty()) return std::map<int, aaesim::open_source::ADSBSVReport>{};
   m_most_recent_adsb_reports.clear();
   m_most_recent_adsb_reports.insert(
         std::make_pair(surveillance_reports.front().GetId(), surveillance_reports.front()));
   return m_most_recent_adsb_reports;
}
