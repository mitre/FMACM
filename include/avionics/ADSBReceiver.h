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

#include <scalar/Length.h>
#include "loader/Loadable.h"
#include "public/SimulationTime.h"
#include "public/AircraftState.h"
#include "public/ADSBSVReport.h"
#include "public/ADSBReceiver.h"
namespace aaesim::avionics {
class ADSBReceiver final : public Loadable, public aaesim::open_source::ADSBReceiver {
  public:
   ADSBReceiver() = default;
   ~ADSBReceiver() = default;

   void Initialize(Units::Length adsb_reception_range_threshold) override;
   bool load(DecodedStream *input) override;
   bool IsLoaded() const { return m_is_loaded; }
   std::map<int, aaesim::open_source::ADSBSVReport> Receive(const aaesim::open_source::SimulationTime &time,
                                                            const aaesim::open_source::AircraftState &state) override;
   std::map<int, aaesim::open_source::ADSBSVReport> const GetCurrentADSBReport() const override {
      return m_most_recent_adsb_reports;
   }
   aaesim::open_source::ADSBSVReport GetCurrentADSBReport(int id) const override;
   aaesim::open_source::ADSBSVReport GetADSBReportBefore(int id, Units::Time time) const override;
   double GetComputedReceptionProbability() const;
   bool GetReceivedHitStatus() const;
   const std::map<int, std::vector<aaesim::open_source::ADSBSVReport> > &GetAllReportsReceived() const override {
      return m_all_reports_received;
   }
   const std::vector<aaesim::open_source::ADSBSVReport> &GetReportsReceivedByTime(
         const aaesim::open_source::SimulationTime &time) const override {
      if (m_reports_received_by_time.empty() ||
          m_reports_received_by_time.find(time.GetCycle()) == m_reports_received_by_time.end()) {
         static std::vector<aaesim::open_source::ADSBSVReport> empty;
         return empty;
      }
      return m_reports_received_by_time.at(time.GetCycle());
   }

  private:
   inline static log4cplus::Logger m_logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("ADSBReceiver"))};

   bool IsReceived(double probability);
   void UpdateAllReportsFromLastReport();
   double CalculateReceptionProbabilityFromRange(const Units::Length range);

   std::map<int, aaesim::open_source::ADSBSVReport> m_most_recent_adsb_reports{};  // ADSB data stored as
                                                                                   // [numericFlightID][Report]
   std::map<int, std::vector<aaesim::open_source::ADSBSVReport> >
         m_all_reports_received{};  // ADSB data stored as
                                    // [numericFlightID][Report]
   std::map<int, std::vector<aaesim::open_source::ADSBSVReport> >
         m_reports_received_by_time{};  // ADSB reports received
                                        // stored as
                                        // [time][report_vector]
   double m_probability_of_reception{0.0};
   bool m_calculate_probability_of_receiving{false};
   Units::Length m_adsb_reception_range_threshold{Units::zero()};
   bool m_first_time_through{false};
   double m_computed_reception_probability{0.0};
   bool m_hit_status{false};
   bool m_is_loaded{false};
};
}  // namespace aaesim::avionics
