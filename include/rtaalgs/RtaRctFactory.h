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

#include "public/FlightDeckApplicationLoader.h"
#include "public/SpeedCommandLimiter.h"
#include "rtaalgs/RTAUtils.h"
#include "imalgs/FIMConfiguration.h"
#include "public/StatisticalPilotDelay.h"

namespace required_time_of_arrival {
struct RtaRctClearance : public Loadable {
   RtaRctClearance() = default;
   bool load(DecodedStream *input) override;
   Units::SecondsTime m_required_time_of_arrival{Units::negInfinity()};
   std::string m_rta_waypoint_name{};
};

class RtaRctFactory final : public aaesim::open_source::FlightDeckApplicationLoader {
  public:
   inline static const std::string TOP_LEVEL_BRACKET_NAME{"rta_rct"};
   static std::shared_ptr<aaesim::open_source::FlightDeckApplication> ConstructAlgorithm(
         const interval_management::open_source::FIMConfiguration &configuration, const RtaRctClearance &clearance,
         aaesim::open_source::StatisticalPilotDelay &pilot_delay);
   RtaRctFactory() = default;
   void RegisterLoadableVariables() override;
   std::shared_ptr<aaesim::open_source::FlightDeckApplication> ConstructLoadedAlgorithm(
         aaesim::open_source::StatisticalPilotDelay &pilot_delay) override;
   bool load(DecodedStream *input) override;
   bool VariablesAreLoaded() const override;
   std::string GetTopLevelTag() const override;
   interval_management::open_source::FIMConfiguration GetLoadedConfiguration() const;
   RtaRctClearance GetLoadedClearance() const;

  private:
   interval_management::open_source::FIMConfiguration m_configuration{};
   RtaRctClearance m_clearance{};
   bool m_loaded{false};
};

inline interval_management::open_source::FIMConfiguration RtaRctFactory::GetLoadedConfiguration() const {
   return m_configuration;
};
inline RtaRctClearance RtaRctFactory::GetLoadedClearance() const { return m_clearance; };
inline bool RtaRctFactory::VariablesAreLoaded() const { return m_loaded; };

}  // namespace required_time_of_arrival
