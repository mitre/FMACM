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

#include "imalgs/IMAlgorithm.h"
#include "public/FlightDeckApplication.h"
#include "public/FlightDeckApplication.h"
#include "public/ASSAP.h"

namespace aaesim {
class InternalAlgorithmAdapter : public aaesim::open_source::FlightDeckApplication {
  public:
   InternalAlgorithmAdapter(std::shared_ptr<interval_management::open_source::IMAlgorithm> im_algorithm,
                            IMUtils::IMAlgorithmTypes algorithm_type);
   ~InternalAlgorithmAdapter() = default;
   void Initialize(aaesim::open_source::FlightDeckApplicationInitializer &initializer_visitor) override;
   aaesim::open_source::Guidance Update(const aaesim::open_source::SimulationTime &simtime,
                                        const aaesim::open_source::Guidance &prevguidance,
                                        const aaesim::open_source::DynamicsState &dynamicsstate,
                                        const aaesim::open_source::AircraftState &owntruthstate) override;
   bool IsActive() const override;
   std::shared_ptr<interval_management::open_source::IMAlgorithm> GetImAlgorithm() const;
   IMUtils::IMAlgorithmTypes GetImAlgorithmType() const;

   const interval_management::open_source::IMClearance &GetImClearance() const {
      return m_im_algorithm->GetClearance();
   }

  private:
   void UpdateTargetHistory(const aaesim::open_source::SimulationTime &simtime);
   std::shared_ptr<interval_management::open_source::IMAlgorithm> m_im_algorithm;
   IMUtils::IMAlgorithmTypes m_im_algorithm_type;
   std::shared_ptr<const aaesim::open_source::ASSAP> m_assap;
   int m_target_id;
   aaesim::open_source::GuidanceFlightPhase m_current_guidance_phase;
   std::vector<interval_management::open_source::AircraftState> m_target_history;
   bool m_initialized;
};

inline std::shared_ptr<interval_management::open_source::IMAlgorithm> aaesim::InternalAlgorithmAdapter::GetImAlgorithm()
      const {
   return m_im_algorithm;
}

inline IMUtils::IMAlgorithmTypes InternalAlgorithmAdapter::GetImAlgorithmType() const { return m_im_algorithm_type; }
}  // namespace aaesim