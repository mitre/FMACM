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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <public/AircraftState.h>
#include <public/Guidance.h>
#include <public/EuclideanThreeDofDynamics.h>
#include "public/Wind.h"

using namespace std;
using namespace aaesim::open_source;

log4cplus::Logger EuclideanThreeDofDynamics::m_logger = log4cplus::Logger::getInstance("EuclideanThreeDofDynamics");

void EuclideanThreeDofDynamics::CalculateEnvironmentalWind(WindStack &wind_east,
                                                           WindStack &wind_north,
                                                           Units::Frequency &dVwx_dh,
                                                           Units::Frequency &dVwy_dh) {
   // Have to do update whether using wind or not.  Otherwise density and pressure are not updated during descent.
   EarthModel::LocalPositionEnu localPosition;
   localPosition.x = m_equations_of_motion_state.enu_x;
   localPosition.y = m_equations_of_motion_state.enu_y;
   localPosition.z = m_equations_of_motion_state.altitude_msl;
   m_tangent_plane_sequence->convertLocalToGeodetic(localPosition, m_equations_of_motion_state.geodetic_position);
   m_true_weather.LoadConditionsAt(m_equations_of_motion_state.geodetic_position.latitude,
                                   m_equations_of_motion_state.geodetic_position.longitude,
                                   m_equations_of_motion_state.altitude_msl);

   if (Wind::UseWind()) {
      wind_east = m_true_weather.east_west;
      wind_north = m_true_weather.north_south;

      // Get Winds and Wind Gradients at altitude
      m_true_weather.getAtmosphere()->CalculateWindGradientAtAltitude(m_equations_of_motion_state.altitude_msl, wind_east,
                                                                      m_wind_velocity_east, dVwx_dh);
      m_true_weather.getAtmosphere()->CalculateWindGradientAtAltitude(m_equations_of_motion_state.altitude_msl, wind_north,
                                                                      m_wind_velocity_north, dVwy_dh);
   } else {
      // return zeros
      static const WindStack zero_stack = WindStack::CreateZeroSpeedStack();
      wind_east = zero_stack;
      wind_north = zero_stack;
      dVwx_dh = Units::zero();
      dVwy_dh = Units::zero();
   }

}
void EuclideanThreeDofDynamics::Initialize(std::shared_ptr<const BadaPerformanceCalculator> aircraft_performance,
                                           const Waypoint &initial_position,
                                           std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                                           Units::Length initial_altitude_msl,
                                           Units::Speed initial_true_airspeed,
                                           Units::Angle initial_ground_course_enu,
                                           double initial_mass_fraction,
                                           const WeatherTruth &true_weather,
                                           const string &aircraft_type) {
   // Handle Euclidean initializations
   this->m_tangent_plane_sequence = std::move(tangent_plane_sequence);
   EarthModel::LocalPositionEnu initial_enu_position;
   m_tangent_plane_sequence->convertGeodeticToLocal(EarthModel::GeodeticPosition::CreateFromWaypoint(initial_position), initial_enu_position);
   m_dynamics_state.x = initial_enu_position.x;
   m_dynamics_state.y = initial_enu_position.y;
   m_equations_of_motion_state.enu_x = Units::MetersLength(m_dynamics_state.x);
   m_equations_of_motion_state.enu_y = Units::MetersLength(m_dynamics_state.y);
   m_equations_of_motion_state.geodetic_position = EarthModel::GeodeticPosition::CreateFromWaypoint(initial_position);

   // Finish with base class
   ThreeDOFDynamics::Initialize(aircraft_performance,
                                initial_position,
                                tangent_plane_sequence,
                                initial_altitude_msl,
                                initial_true_airspeed,
                                initial_ground_course_enu,
                                initial_mass_fraction,
                                true_weather,
                                aircraft_type);

}

AircraftState EuclideanThreeDofDynamics::Update(const Guidance &guidance,
                                                const shared_ptr<AircraftControl> &aircraft_control) {
   LOG4CPLUS_TRACE(m_logger, "starting...");
   return ThreeDOFDynamics::Update(guidance, aircraft_control);
}

