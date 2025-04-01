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

#include <map>
#include <vector>

#include "public/ADSBSVReport.h"
#include "public/SimulationTime.h"
#include "public/TangentPlaneSequence.h"

class ADSBEther final {
  public:
   static ADSBEther *GetInstance();

   static void ClearInstance();

   void LoadTtvFile(const std::string &file_name);

   void PublishToEther(const aaesim::open_source::SimulationTime &time, aaesim::open_source::ADSBSVReport data);

   aaesim::open_source::ADSBSVReport GetMostRecentReportForIdAndTime(const int id, const int time_seconds) const;

   std::vector<aaesim::open_source::ADSBSVReport> GetMostRecentAtOrBeforeTime(const int time_seconds) const;

   std::vector<aaesim::open_source::ADSBSVReport> GetAllForID(const int numeric_id) const;

   std::vector<aaesim::open_source::ADSBSVReport> GetAllForPublishedTime(const int time_seconds) const;

   std::map<int, std::vector<aaesim::open_source::ADSBSVReport> > GetAllReports() const;

   ~ADSBEther() = default;

   void Clear();

  private:
   static std::unique_ptr<ADSBEther> m_instance;

   ADSBEther() = default;

   std::map<int, std::vector<aaesim::open_source::ADSBSVReport> > m_surveillance_reports{};

   std::map<int, std::vector<aaesim::open_source::ADSBSVReport> > m_reports_by_simulation_time{};
};
inline std::map<int, std::vector<aaesim::open_source::ADSBSVReport> > ADSBEther::GetAllReports() const {
   return m_surveillance_reports;
}