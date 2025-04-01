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

#include "public/WGS84EarthModelConstants.h"

namespace aaesim {
class Wgs84PositionEstimator final : public aaesim::open_source::EllipsoidalPositionEstimator {
  public:
   Wgs84PositionEstimator(const EarthModel::GeodeticPosition &initial_position)
      : m_last_resolved_position{initial_position} {}
   ~Wgs84PositionEstimator() = default;
   void ComputePosition(const aaesim::open_source::SimulationTime &simtime, const EquationsOfMotionState &eqm_state,
                        const EquationsOfMotionStateDeriv &eqm_state_derivative, EarthModel::GeodeticPosition &position,
                        aaesim::open_source::LatLonDerivative &position_rate) override;

  private:
   inline static const Units::Area m_semi_major_radius_squared{aaesim::open_source::WGS84_SEMIMAJOR_AXIS *
                                                               aaesim::open_source::WGS84_SEMIMAJOR_AXIS};
   inline static const Units::Area m_semi_minor_radius_squared{aaesim::open_source::WGS84_SEMIMINOR_AXIS *
                                                               aaesim::open_source::WGS84_SEMIMINOR_AXIS};
   static Units::Angle ComputeGeocentricLatitude(const Units::Angle &geodetic_latitude);
   static Units::Length ComputeEarthRadius(const Units::Angle &geocentric_latitude);
   static Units::Length ComputeEarthRadiusDerivativeTerm(const Units::Angle geocentric_latitude,
                                                         const Units::Angle geodetic_latitude);
   static double ComputeUnnamedTerm(const Units::Angle &geodetic_latitude);

   aaesim::open_source::LatLonDerivative ComputeLatLonRates(
         const EquationsOfMotionState &eqm_state, const EquationsOfMotionStateDeriv &eqm_state_derivative) const;
   EarthModel::GeodeticPosition ComputeLatLon(const aaesim::open_source::SimulationTime &simtime,
                                              const aaesim::open_source::LatLonDerivative &derivatives) const;
   Units::Length ConvertAltitudeMslToHae(const Units::Length &altitude_msl) const;

   EarthModel::GeodeticPosition m_last_resolved_position{};
};
}  // namespace aaesim