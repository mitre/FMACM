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

#include "aaesim/FmsIntegratedApplicationLoader.h"
#include "public/SpeedCommandLimiter.h"
#include "rtaalgs/RTAUtils.h"

namespace required_time_of_arrival {
struct RtaToacConfiguration : public Loadable {
   RtaToacConfiguration();
   bool load(DecodedStream *input) override;
   std::string m_speed_limiter;
   Units::SecondsTime m_arrival_error_tolerance;
   Units::SecondsTime m_eta_error_tolerance;

   static RtaToacConfiguration CreateRtaToacDefaultConfiguration() {
      RtaToacConfiguration return_this;
      return_this.m_speed_limiter.assign("none");
      return_this.m_arrival_error_tolerance = Units::SecondsTime(1.0);
      return_this.m_eta_error_tolerance = Units::SecondsTime(2.0);
      return return_this;
   }
};

struct RtaToacClearance : public Loadable {
   RtaToacClearance();
   bool load(DecodedStream *input) override;
   Units::SecondsTime m_required_time_of_arrival;
   std::string m_rta_waypoint_name;
};

class RtaToacFactory : public aaesim::avionics::FmsIntegratedApplicationLoader {
  public:
   static const std::string TOP_LEVEL_BRACKET_NAME;
   static std::shared_ptr<aaesim::avionics::FmsIntegratedApplication> ConstructAlgorithm(
         RtaToacConfiguration &configuration, RtaToacClearance &clearance);
   RtaToacFactory();
   void RegisterLoadableVariables() override;
   std::shared_ptr<aaesim::avionics::FmsIntegratedApplication> ConstructLoadedAlgorithm() override;
   bool load(DecodedStream *input) override;
   bool VariablesAreLoaded() const override;
   std::string GetTopLevelTag() const override;
   RtaToacConfiguration GetLoadedConfiguration() const;
   RtaToacClearance GetLoadedClearance() const;

  private:
   static RTAUtils::SpeedLimiterType GetSpeedLimiterTypeFromLoadedParameter(std::string speed_limiter_type);
   RtaToacConfiguration m_configuration;
   RtaToacClearance m_clearance;
   bool m_loaded;
};

inline RtaToacConfiguration RtaToacFactory::GetLoadedConfiguration() const { return m_configuration; };
inline RtaToacClearance RtaToacFactory::GetLoadedClearance() const { return m_clearance; };
inline bool RtaToacFactory::VariablesAreLoaded() const { return m_loaded; };

}  // namespace required_time_of_arrival
