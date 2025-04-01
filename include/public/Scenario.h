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

#include <string>

namespace aaesim::open_source {
class Scenario {
  public:
   Scenario() : m_scenario_name() {}
   virtual ~Scenario() = default;
   virtual void SimulateAllIterations() = 0;
   virtual const std::string &GetScenarioName() const;
   void SetScenarioName(const std::string &in);

   //  const std::string& GetScenarioPath() { return m_scenario_path};
   //  const std::string& GetOutputPath() {return m_output_path};
   //  const double GetStartTimeRngSeed() {return m_start_time_rng_seed};

  private:
   std::string m_scenario_name;
};

inline const std::string &Scenario::GetScenarioName() const { return m_scenario_name; }

inline void Scenario::SetScenarioName(const std::string &scenario_name) { m_scenario_name = scenario_name; }
}  // namespace aaesim::open_source
