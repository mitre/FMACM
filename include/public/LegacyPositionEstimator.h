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

#include "public/EllipsoidalPositionEstimator.h"
#include "public/TangentPlaneSequence.h"

namespace aaesim::open_source {
class LegacyPositionEstimator final : public EllipsoidalPositionEstimator {
  public:
   LegacyPositionEstimator(const std::shared_ptr<TangentPlaneSequence> &position_converter,
                           const EarthModel::GeodeticPosition &initial_position)
      : m_tangent_plane_sequence(position_converter), m_last_resolved_position(initial_position) {}
   ~LegacyPositionEstimator() = default;
   void ComputePosition(const SimulationTime &simtime, const EquationsOfMotionState &eqm_state,
                        const EquationsOfMotionStateDeriv &eqm_state_derivative, EarthModel::GeodeticPosition &position,
                        LatLonDerivative &position_rate) override;

  private:
   EarthModel::GeodeticPosition ComputeLatLon(const EquationsOfMotionState &eqm_state) const;
   std::shared_ptr<TangentPlaneSequence> m_tangent_plane_sequence;
   EarthModel::GeodeticPosition m_last_resolved_position{};
};
}  // namespace aaesim::open_source