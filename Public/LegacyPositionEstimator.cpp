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

#include "public/LegacyPositionEstimator.h"

using namespace aaesim::open_source;

EarthModel::GeodeticPosition LegacyPositionEstimator::ComputeLatLon(const EquationsOfMotionState &eqm_state) const {
   auto local_position = EarthModel::LocalPositionEnu{};
   local_position.x = eqm_state.enu_x;
   local_position.y = eqm_state.enu_y;
   local_position.z = eqm_state.altitude_msl;
   EarthModel::GeodeticPosition geodetic_position;
   m_tangent_plane_sequence->ConvertLocalToGeodetic(local_position, geodetic_position);
   geodetic_position.altitude = eqm_state.altitude_msl;
   return geodetic_position;
}

void LegacyPositionEstimator::ComputePosition(const SimulationTime &simtime, const EquationsOfMotionState &eqm_state,
                                              const EquationsOfMotionStateDeriv &eqm_state_derivative,
                                              EarthModel::GeodeticPosition &position, LatLonDerivative &position_rate) {
   position = ComputeLatLon(eqm_state);
   position_rate.latitude_time_derivative =
         (position.latitude - m_last_resolved_position.latitude) / simtime.GetSimulationTimeStep();
   position_rate.longitude_time_derivative =
         (position.longitude - m_last_resolved_position.longitude) / simtime.GetSimulationTimeStep();
   m_last_resolved_position = position;
}
