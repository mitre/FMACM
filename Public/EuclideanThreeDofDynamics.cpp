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

#include "public/EuclideanThreeDofDynamics.h"

#include "public/AircraftState.h"
#include "public/Guidance.h"
#include "public/Wind.h"

using namespace aaesim::open_source;

log4cplus::Logger EuclideanThreeDofDynamics::m_logger = log4cplus::Logger::getInstance("EuclideanThreeDofDynamics");

void EuclideanThreeDofDynamics::CalculateEnvironmentalWind(WindStack &wind_east, WindStack &wind_north,
                                                           Units::Frequency &dVwx_dh, Units::Frequency &dVwy_dh) {
   // Have to do update whether using wind or not.  Otherwise density and pressure are not updated during descent.
   EarthModel::LocalPositionEnu localPosition;
   localPosition.x = m_equations_of_motion_state.enu_x;
   localPosition.y = m_equations_of_motion_state.enu_y;
   localPosition.z = m_equations_of_motion_state.altitude_msl;
   m_tangent_plane_sequence->convertLocalToGeodetic(localPosition, m_equations_of_motion_state.geodetic_position);
   m_true_weather->LoadConditionsAt(m_equations_of_motion_state.geodetic_position.latitude,
                                    m_equations_of_motion_state.geodetic_position.longitude,
                                    m_equations_of_motion_state.altitude_msl);

   ThreeDOFDynamics::CalculateEnvironmentalWind(wind_east, wind_north, dVwx_dh, dVwy_dh);
}

void EuclideanThreeDofDynamics::Initialize(
      std::shared_ptr<const aaesim::open_source::FixedMassAircraftPerformance> aircraft_performance,
      const Waypoint &initial_position, std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
      Units::Length initial_altitude_msl, Units::Speed initial_true_airspeed, Units::Angle initial_ground_course_enu,
      double initial_mass_fraction, std::shared_ptr<aaesim::open_source::WeatherTruth> true_weather) {
   this->m_tangent_plane_sequence = std::move(tangent_plane_sequence);
   EarthModel::LocalPositionEnu initial_enu_position;
   m_tangent_plane_sequence->convertGeodeticToLocal(EarthModel::GeodeticPosition::CreateFromWaypoint(initial_position),
                                                    initial_enu_position);
   m_equations_of_motion_state.geodetic_position = EarthModel::GeodeticPosition::CreateFromWaypoint(initial_position);

   ThreeDOFDynamics::Initialize(aircraft_performance, initial_enu_position, initial_altitude_msl, initial_true_airspeed,
                                initial_ground_course_enu, initial_mass_fraction, true_weather);
}
