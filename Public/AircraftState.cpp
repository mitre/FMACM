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

#include "public/AircraftState.h"

#include <iomanip>
#include <cmath>

#include "public/CustomMath.h"

using namespace aaesim::open_source;

Units::UnsignedRadiansAngle AircraftState::GetHeadingCcwFromEastRadians() const {
   double result = atan3(m_yd.value(), m_xd.value());
   return Units::UnsignedRadiansAngle(result);
}

Units::Speed AircraftState::GetGroundSpeed() const {
   return Units::sqrt(Units::sqr(GetSpeedEnuX()) + Units::sqr(GetSpeedEnuY()));
}

AircraftState AircraftState::FromAdsbReport(const aaesim::open_source::ADSBSVReport &adsb_report) {
   return aaesim::open_source::AircraftState::Builder(adsb_report.GetId(), adsb_report.GetTime())
         .Position(adsb_report.GetX(), adsb_report.GetY())
         ->Latitude(adsb_report.GetLatitude())
         ->Longitude(adsb_report.GetLongitude())
         ->AltitudeMsl(adsb_report.GetAltitudeMsl())
         ->GroundSpeed(adsb_report.GetXd(), adsb_report.GetYd())
         ->AltitudeRate(adsb_report.GetVerticalSpeed())
         ->Build();
}

/**
 * Interpolates between (or extrapolates from) a pair of input states.
 * This interpolation is linear on position and velocity fields,
 * which is only mathematically consistent when velocity does not change.
 * Only id, time, position, and velocity fields are written.
 * Other fields, such as acceleration, orientation, and wind,
 * are left unchanged.
 */
AircraftState &AircraftState::Interpolate(const AircraftState &a, const AircraftState &b, const double time) {
   if (a.GetUniqueId() != b.GetUniqueId()) {
      LOG4CPLUS_ERROR(logger, "Interpolating between states that have different ids:  " + std::to_string(a.m_id) +
                                    " and " + std::to_string(b.m_id) + ".");
   }

   const double baTimeDiff = Units::SecondsTime(b.GetTime() - a.GetTime()).value();
   double aWeight, bWeight;
   if (baTimeDiff == 0) {
      // avoid divide by zero
      aWeight = 1;
      bWeight = 0;
      LOG4CPLUS_ERROR(logger, "Attempt to interpolate between two states for the same time.");
   } else {
      aWeight = (b.m_time.value() - time) / baTimeDiff;
      bWeight = 1 - aWeight;
   }
   m_id = a.GetUniqueId();
   m_time = Units::SecondsTime(time);
   m_x = a.m_x * aWeight + b.m_x * bWeight;
   m_y = a.m_y * aWeight + b.m_y * bWeight;
   m_z = a.m_z * aWeight + b.m_z * bWeight;
   m_xd = a.m_xd * aWeight + b.m_xd * bWeight;
   m_yd = a.m_yd * aWeight + b.m_yd * bWeight;
   m_zd = a.m_zd * aWeight + b.m_zd * bWeight;
   m_latitude = a.m_latitude * aWeight + b.m_latitude * bWeight;
   m_longitude = a.m_longitude * aWeight + b.m_longitude * bWeight;
   return *this;
}

AircraftState &AircraftState::Extrapolate(const AircraftState &in, const Units::SecondsTime &time) {
   Units::SecondsTime dt = time - in.m_time;
   m_time = time;
   m_id = in.m_id;
   m_x = in.m_x + in.m_xd * dt;
   m_y = in.m_y + in.m_yd * dt;
   m_z = in.m_z + in.m_zd * dt;
   m_xd = in.m_xd;
   m_yd = in.m_yd;
   m_zd = in.m_zd;
   m_latitude = in.m_latitude + in.m_latitude_rate * dt;
   m_longitude = in.m_longitude + in.m_longitude_rate * dt;
   return *this;
}

Units::Speed AircraftState::GetTrueAirspeed() const {
   Units::MetersPerSecondSpeed tas_x, tas_y;
   tas_x = Units::FeetPerSecondSpeed(m_xd) - Units::MetersPerSecondSpeed(m_sensed_wind_east);
   tas_y = Units::FeetPerSecondSpeed(m_yd) - Units::MetersPerSecondSpeed(m_sensed_wind_north);
   Units::MetersPerSecondSpeed tas = Units::sqrt(Units::sqr(tas_x) + Units::sqr(tas_y));
   return tas;
}

AircraftState::AircraftState(const AircraftState::Builder &builder) {
   m_id = builder.GetUniqueId();
   m_time = builder.GetTimestamp();
   m_x = builder.GetPositionEnuX();
   m_y = builder.GetPositionEnuY();
   m_z = builder.GetAltitudeMsl();
   m_xd = builder.GetGroundSpeedEast();
   m_yd = builder.GetGroundSpeedNorth();
   m_zd = builder.GetAltitudeRate();
   m_xdd = builder.GetGroundAccelerationEastComponent();
   m_ydd = builder.GetGroundAccelerationNorthComponent();
   m_zdd = builder.GetAltitudeAcceleration();
   m_gamma = builder.GetFlightPathAngle();
   m_sensed_wind_east = builder.GetSensedWindEastComponent();
   m_sensed_wind_north = builder.GetSensedWindNorthComponent();
   m_sensed_wind_parallel = builder.GetSensedWindParallelComponent();
   m_sensed_wind_perpendicular = builder.GetSensedWindPerpendicularComponent();
   m_Vwx_dh = builder.GetVerticalWindDerivativeEastComponent();
   m_Vwy_dh = builder.GetVerticalWindDerivativeNorthComponent();
   m_dynamics_state = builder.GetDynamicsState();
   m_sensed_temperature = builder.GetSensedTemperature();
   m_sensed_density = builder.GetSensedDensity();
   m_sensed_pressure = builder.GetSensedPressure();
   m_latitude = builder.GetLatitude();
   m_longitude = builder.GetLongitude();
   m_latitude_rate = builder.GetLatitudeRate();
   m_longitude_rate = builder.GetLongitudeRate();
   m_psi = builder.GetPsi();
}

AircraftState::Builder::Builder(int unique_acid, int time_since_epoch_seconds) {
   id_ = unique_acid;
   timestamp_ = Units::SecondsTime(time_since_epoch_seconds);
}

AircraftState::Builder::Builder(int unique_acid, Units::Time timestamp) {
   id_ = unique_acid;
   timestamp_ = timestamp;
}

AircraftState::Builder::Builder(const AircraftState &state_to_copy) {
   id_ = state_to_copy.GetUniqueId();
   timestamp_ = state_to_copy.GetTime();
   enu_x_ = state_to_copy.GetPositionEnuX();
   enu_y_ = state_to_copy.GetPositionEnuY();
   altitude_msl_ = state_to_copy.GetAltitudeMsl();
   enu_east_ = state_to_copy.GetSpeedEnuX();
   enu_north_ = state_to_copy.GetSpeedEnuY();
   altitude_rate_ = state_to_copy.GetVerticalSpeed();
   enu_accel_east_ = state_to_copy.m_xdd;
   enu_accel_north_ = state_to_copy.m_ydd;
   altitude_accel_ = state_to_copy.m_zdd;
   flight_path_angle_ = state_to_copy.m_gamma;
   sensed_wind_east_ = state_to_copy.m_sensed_wind_east;
   sensed_wind_north_ = state_to_copy.m_sensed_wind_north;
   sensed_wind_parallel_ = state_to_copy.m_sensed_wind_parallel;
   sensed_wind_perpendicular_ = state_to_copy.m_sensed_wind_perpendicular;
   sensed_wind_vertical_derivative_east_ = state_to_copy.m_Vwx_dh;
   sensed_wind_vertical_derivative_north_ = state_to_copy.m_Vwy_dh;
   sensed_temperature_ = state_to_copy.m_sensed_temperature;
   sensed_density_ = state_to_copy.m_sensed_density;
   sensed_pressure_ = state_to_copy.m_sensed_pressure;
   latitude_ = state_to_copy.m_latitude;
   longitude_ = state_to_copy.m_longitude;
   latitude_rate_ = state_to_copy.m_latitude_rate;
   longitude_rate_ = state_to_copy.m_longitude_rate;
   psi_ = state_to_copy.GetPsi();
   dynamics_state_ = state_to_copy.GetDynamicsState();
}

AircraftState AircraftState::Builder::Build() { return AircraftState{*this}; }

AircraftState::Builder *AircraftState::Builder::Position(Units::FeetLength enu_x, Units::FeetLength enu_y) {
   enu_x_ = enu_x;
   enu_y_ = enu_y;
   return this;
}

AircraftState::Builder *AircraftState::Builder::AltitudeMsl(Units::FeetLength altitude_msl) {
   altitude_msl_ = altitude_msl;
   return this;
}

AircraftState::Builder *AircraftState::Builder::GroundSpeed(Units::FeetPerSecondSpeed enu_east,
                                                            Units::FeetPerSecondSpeed enu_north) {
   enu_east_ = enu_east;
   enu_north_ = enu_north;
   return this;
}

AircraftState::Builder *AircraftState::Builder::AltitudeRate(Units::FeetPerSecondSpeed altitude_rate) {
   altitude_rate_ = altitude_rate;
   return this;
}

AircraftState::Builder *AircraftState::Builder::GroundAcceleration(Units::FeetSecondAcceleration enu_east,
                                                                   Units::FeetSecondAcceleration enu_north) {
   enu_accel_east_ = enu_east;
   enu_accel_north_ = enu_north;
   return this;
}

AircraftState::Builder *AircraftState::Builder::AltitudeAcceleration(
      Units::FeetSecondAcceleration altitude_acceleration) {
   altitude_accel_ = altitude_acceleration;
   return this;
}

AircraftState::Builder *AircraftState::Builder::FlightPathAngle(Units::SignedAngle fpa) {
   flight_path_angle_ = fpa;
   return this;
}

AircraftState::Builder *AircraftState::Builder::SensedWindComponents(Units::Speed east, Units::Speed north) {
   sensed_wind_east_ = east;
   sensed_wind_north_ = north;
   return this;
}

AircraftState::Builder *AircraftState::Builder::VerticalWindDerivatives(Units::Frequency east, Units::Frequency north) {
   sensed_wind_vertical_derivative_east_ = east;
   sensed_wind_vertical_derivative_north_ = north;
   return this;
}

AircraftState::Builder *AircraftState::Builder::SensedTemperature(Units::Temperature temperature) {
   sensed_temperature_ = temperature;
   return this;
}

AircraftState::Builder *AircraftState::Builder::SensedDensity(Units::Density density) {
   sensed_density_ = density;
   return this;
}

AircraftState::Builder *AircraftState::Builder::SensedPressure(Units::Pressure pressure) {
   sensed_pressure_ = pressure;
   return this;
}

AircraftState::Builder *AircraftState::Builder::Latitude(Units::SignedAngle latitude) {
   latitude_ = latitude;
   return this;
}

AircraftState::Builder *AircraftState::Builder::LatitudeRate(Units::AngularSpeed latitude_rate) {
   latitude_rate_ = latitude_rate;
   return this;
}

AircraftState::Builder *AircraftState::Builder::Longitude(Units::SignedAngle longitude) {
   longitude_ = longitude;
   return this;
}

AircraftState::Builder *AircraftState::Builder::LongitudeRate(Units::AngularSpeed longitude_rate) {
   longitude_rate_ = longitude_rate;
   return this;
}

AircraftState::Builder *AircraftState::Builder::Psi(Units::Angle psi) {
   psi_ = psi;
   return this;
}

AircraftState::Builder *AircraftState::Builder::DynamicsState(
      const aaesim::open_source::DynamicsState &dynamics_state) {
   dynamics_state_ = dynamics_state;
   return this;
}

AircraftState::Builder *AircraftState::Builder::SensedWindsPerpendicular(Units::Speed wind_perpendicular_component) {
   sensed_wind_perpendicular_ = wind_perpendicular_component;
   return this;
}

AircraftState::Builder *AircraftState::Builder::SensedWindsParallel(Units::Speed wind_parallel_component) {
   sensed_wind_parallel_ = wind_parallel_component;
   return this;
}