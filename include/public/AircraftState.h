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

#include "public/Logging.h"
#include "public/ADSBSVReport.h"
#include "public/BadaUtils.h"
#include "public/DynamicsState.h"

#include "scalar/Density.h"
#include "scalar/Frequency.h"
#include "scalar/Pressure.h"
#include "scalar/Speed.h"
#include "scalar/Temperature.h"
#include "scalar/UnsignedAngle.h"
#include "scalar/AngularSpeed.h"

namespace aaesim {
namespace open_source {

class AircraftState final {
  public:
   static AircraftState FromAdsbReport(const ADSBSVReport &adsb_report);

   AircraftState() = default;
   virtual ~AircraftState() = default;

   AircraftState &Interpolate(const AircraftState &a, const AircraftState &b, const double time);
   AircraftState &Extrapolate(const AircraftState &in, const Units::SecondsTime &time);

   int GetUniqueId() const;
   Units::Length GetPositionEnuX() const;
   Units::Length GetPositionEnuY() const;
   Units::Length GetAltitudeMsl() const;
   Units::Speed GetSpeedEnuX() const;
   Units::Speed GetSpeedEnuY() const;
   Units::Speed GetVerticalSpeed() const;
   Units::Speed GetTrueAirspeed() const;
   Units::SignedAngle GetLatitude() const;
   Units::SignedAngle GetLongitude() const;
   Units::AngularSpeed GetLatitudeRate() const;
   Units::AngularSpeed GetLongitudeRate() const;
   Units::Angle GetFlightPathAngle() const;
   Units::SecondsTime GetTime() const;
   aaesim::open_source::DynamicsState GetDynamicsState() const;
   Units::Angle GetPsi() const;
   Units::UnsignedRadiansAngle GetHeadingCcwFromEastRadians() const;
   Units::Speed GetGroundSpeed() const;
   Units::Acceleration GetAccelerationEnuX() const;
   Units::Acceleration GetAccelerationEnuY() const;
   Units::Acceleration GetVerticalAcceleration() const;
   Units::Temperature GetSensedTemperature() const;
   Units::Speed GetSensedWindEast() const;
   Units::Speed GetSensedWindNorth() const;
   Units::Speed GetSensedWindParallel() const;
   Units::Speed GetSensedWindPerpendicular() const;
   Units::Frequency GetVerticalWindDerivativeEastComponent() const;
   Units::Frequency GetVerticalWindDerivativeNorthComponent() const;

   class Builder {
     private:
      int id_{-1};
      Units::Time timestamp_{Units::SecondsTime(-1)};
      Units::FeetLength enu_x_{Units::zero()}, enu_y_{Units::zero()}, altitude_msl_{Units::zero()};
      Units::FeetPerSecondSpeed enu_east_{Units::zero()}, enu_north_{Units::zero()}, altitude_rate_{Units::zero()};
      Units::FeetSecondAcceleration enu_accel_east_{Units::zero()}, enu_accel_north_{Units::zero()},
            altitude_accel_{Units::zero()};
      Units::RadiansAngle flight_path_angle_{Units::zero()};
      Units::FeetPerSecondSpeed sensed_wind_east_{Units::zero()}, sensed_wind_north_{Units::zero()};
      Units::FeetPerSecondSpeed sensed_wind_parallel_{Units::zero()}, sensed_wind_perpendicular_{Units::zero()};
      Units::HertzFrequency sensed_wind_vertical_derivative_east_{Units::zero()},
            sensed_wind_vertical_derivative_north_{Units::zero()};
      Units::Temperature sensed_temperature_{Units::zero()};
      Units::KilogramsMeterDensity sensed_density_{Units::zero()};
      Units::AtmospheresPressure sensed_pressure_{Units::zero()};
      Units::SignedAngle latitude_{Units::zero()}, longitude_{Units::zero()};
      Units::AngularSpeed latitude_rate_{Units::zero()}, longitude_rate_{Units::zero()};
      Units::RadiansAngle psi_{Units::zero()};
      aaesim::open_source::DynamicsState dynamics_state_{};

     public:
      Builder(int unique_acid, int time_since_epoch_seconds);
      Builder(int unique_acid, Units::Time timestamp);
      Builder(const AircraftState &state_to_copy);
      ~Builder() = default;
      AircraftState Build();
      Builder *Position(Units::FeetLength enu_x, Units::FeetLength enu_y);
      Builder *AltitudeMsl(Units::FeetLength altitude_msl);
      Builder *GroundSpeed(Units::FeetPerSecondSpeed enu_east, Units::FeetPerSecondSpeed enu_north);
      Builder *AltitudeRate(Units::FeetPerSecondSpeed altitude_rate);
      Builder *GroundAcceleration(Units::FeetSecondAcceleration enu_east, Units::FeetSecondAcceleration enu_north);
      Builder *AltitudeAcceleration(Units::FeetSecondAcceleration altitude_acceleration);
      Builder *FlightPathAngle(Units::SignedAngle fpa);
      Builder *SensedWindComponents(Units::Speed east, Units::Speed north);
      Builder *VerticalWindDerivatives(Units::Frequency east, Units::Frequency north);
      Builder *SensedTemperature(Units::Temperature temperature);
      Builder *SensedDensity(Units::Density density);
      Builder *SensedPressure(Units::Pressure pressure);
      Builder *Latitude(Units::SignedAngle latitude);
      Builder *Longitude(Units::SignedAngle longitude);
      Builder *LatitudeRate(Units::AngularSpeed latitude_rate);
      Builder *LongitudeRate(Units::AngularSpeed longitude_rate);
      Builder *DynamicsState(const aaesim::open_source::DynamicsState &dynamics_state);
      Builder *Psi(Units::Angle psi);
      Builder *SensedWindsPerpendicular(Units::Speed wind_perpendicular_component);
      Builder *SensedWindsParallel(Units::Speed wind_parallel_component);

      int GetUniqueId() const { return id_; }
      Units::SecondsTime GetTimestamp() const { return timestamp_; }
      Units::FeetLength GetPositionEnuX() const { return enu_x_; }
      Units::FeetLength GetPositionEnuY() const { return enu_y_; }
      Units::FeetLength GetAltitudeMsl() const { return altitude_msl_; }
      Units::FeetPerSecondSpeed GetGroundSpeedEast() const { return enu_east_; }
      Units::FeetPerSecondSpeed GetGroundSpeedNorth() const { return enu_north_; }
      Units::FeetPerSecondSpeed GetAltitudeRate() const { return altitude_rate_; }
      Units::FeetSecondAcceleration GetGroundAccelerationEastComponent() const { return enu_accel_east_; };
      Units::FeetSecondAcceleration GetGroundAccelerationNorthComponent() const { return enu_accel_north_; };
      Units::FeetSecondAcceleration GetAltitudeAcceleration() const { return altitude_accel_; };
      Units::SignedRadiansAngle GetFlightPathAngle() const { return flight_path_angle_; };
      Units::MetersPerSecondSpeed GetSensedWindEastComponent() const { return sensed_wind_east_; };
      Units::MetersPerSecondSpeed GetSensedWindNorthComponent() const { return sensed_wind_north_; };
      Units::MetersPerSecondSpeed GetSensedWindParallelComponent() const { return sensed_wind_parallel_; };
      Units::MetersPerSecondSpeed GetSensedWindPerpendicularComponent() const { return sensed_wind_perpendicular_; };
      Units::Frequency GetVerticalWindDerivativeEastComponent() const { return sensed_wind_vertical_derivative_east_; };
      Units::Frequency GetVerticalWindDerivativeNorthComponent() const {
         return sensed_wind_vertical_derivative_north_;
      };
      Units::Temperature GetSensedTemperature() const { return sensed_temperature_; };
      Units::Density GetSensedDensity() const { return sensed_density_; };
      Units::Pressure GetSensedPressure() const { return sensed_pressure_; };
      Units::SignedAngle GetLatitude() const { return latitude_; };
      Units::SignedAngle GetLongitude() const { return longitude_; };
      Units::AngularSpeed GetLatitudeRate() const { return latitude_rate_; };
      Units::AngularSpeed GetLongitudeRate() const { return longitude_rate_; };
      aaesim::open_source::DynamicsState GetDynamicsState() const { return dynamics_state_; };
      Units::Angle GetPsi() const { return psi_; };
   };

  private:
   inline static log4cplus::Logger logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AircraftState"))};

   AircraftState(const Builder &builder);

   int m_id{-1};
   Units::SecondsTime m_time{-1};
   Units::FeetLength m_x{0}, m_y{0}, m_z{0};
   Units::FeetPerSecondSpeed m_xd{0}, m_yd{0}, m_zd{0};
   Units::FeetSecondAcceleration m_xdd{0}, m_ydd{0}, m_zdd{0};
   Units::MetersPerSecondSpeed m_sensed_wind_east{0}, m_sensed_wind_north{0};
   Units::RadiansAngle m_psi{Units::zero()};
   Units::RadiansAngle m_gamma{0};
   Units::MetersPerSecondSpeed m_sensed_wind_parallel{0}, m_sensed_wind_perpendicular{0};
   Units::Frequency m_Vwx_dh{Units::zero()}, m_Vwy_dh{Units::zero()};
   Units::Temperature m_sensed_temperature{Units::zero()};
   Units::Density m_sensed_density{Units::zero()};
   Units::Pressure m_sensed_pressure{Units::zero()};
   Units::SignedAngle m_latitude{Units::zero()}, m_longitude{Units::zero()};
   Units::AngularSpeed m_latitude_rate{Units::zero()}, m_longitude_rate{Units::zero()};
   aaesim::open_source::DynamicsState m_dynamics_state{};
};

inline Units::Length AircraftState::GetPositionEnuX() const { return m_x; }
inline Units::Length AircraftState::GetPositionEnuY() const { return m_y; }
inline Units::Length AircraftState::GetAltitudeMsl() const { return m_z; }
inline Units::Speed AircraftState::GetSpeedEnuX() const { return m_xd; }
inline Units::Speed AircraftState::GetSpeedEnuY() const { return m_yd; }
inline Units::Speed AircraftState::GetVerticalSpeed() const { return m_zd; }
inline Units::SignedAngle AircraftState::GetLatitude() const { return m_latitude; }
inline Units::SignedAngle AircraftState::GetLongitude() const { return m_longitude; }
inline Units::AngularSpeed AircraftState::GetLatitudeRate() const { return m_latitude_rate; }
inline Units::AngularSpeed AircraftState::GetLongitudeRate() const { return m_longitude_rate; }
inline aaesim::open_source::DynamicsState AircraftState::GetDynamicsState() const { return m_dynamics_state; }
inline Units::SecondsTime AircraftState::GetTime() const { return m_time; }
inline int AircraftState::GetUniqueId() const { return m_id; }
inline Units::Angle AircraftState::GetPsi() const { return m_psi; }
inline Units::Angle AircraftState::GetFlightPathAngle() const { return m_gamma; }
inline Units::Acceleration AircraftState::GetAccelerationEnuX() const { return m_xdd; }
inline Units::Acceleration AircraftState::GetAccelerationEnuY() const { return m_ydd; }
inline Units::Acceleration AircraftState::GetVerticalAcceleration() const { return m_zdd; }
inline Units::Temperature AircraftState::GetSensedTemperature() const { return m_sensed_temperature; }
inline Units::Speed AircraftState::GetSensedWindEast() const { return m_sensed_wind_east; }
inline Units::Speed AircraftState::GetSensedWindNorth() const { return m_sensed_wind_north; }
inline Units::Speed AircraftState::GetSensedWindParallel() const { return m_sensed_wind_parallel; }
inline Units::Speed AircraftState::GetSensedWindPerpendicular() const { return m_sensed_wind_perpendicular; }
inline Units::Frequency AircraftState::GetVerticalWindDerivativeEastComponent() const { return m_Vwx_dh; }
inline Units::Frequency AircraftState::GetVerticalWindDerivativeNorthComponent() const { return m_Vwy_dh; }

}  // namespace open_source
}  // namespace aaesim
