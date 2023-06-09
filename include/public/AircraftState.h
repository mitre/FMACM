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

#pragma once

#include <string>

#include "utility/Logging.h"
#include "public/ADSBSVReport.h"
#include "public/BadaUtils.h"
#include "public/DynamicsState.h"

#include <scalar/Density.h>
#include <scalar/Frequency.h>
#include <scalar/Pressure.h>
#include <scalar/Speed.h>
#include <scalar/Temperature.h>
#include <scalar/UnsignedAngle.h>

namespace aaesim {
namespace open_source {

class AircraftState {
  public:
   static AircraftState CreateFromADSBReport(const Sensor::ADSB::ADSBSVReport &adsbsvReport);

   AircraftState();

   virtual ~AircraftState();

   void Clear();

   const Units::UnsignedRadiansAngle GetHeadingCcwFromEastRadians() const;

   void SetPsi(const Units::Angle psi_in);

   const Units::Speed GetGroundSpeed() const;

   void DumpParms(std::string str) const;

   void CsvHdrDump(std::string str) const;

   void CsvDataDump(std::string str) const;

   AircraftState &Interpolate(const AircraftState &a, const AircraftState &b, const double time);

   /**
    * @deprecated use ::extrapolate(const AircraftState &in, const Units::SecondsTime &time)
    * @param in
    * @param time
    * @return
    */
   AircraftState &Extrapolate(const AircraftState &in,
                              const double time);  // deprecated!

   /**
    * Returns a state that has been fully populated assuming constant ground speed.
    * @param in
    * @param time
    * @return
    */
   AircraftState &Extrapolate(const AircraftState &in, const Units::SecondsTime &time);

   const bool IsTurning() const;

   double GetZd() const;

   void SetZd(const double zd);

   Units::Length GetPositionX() const;

   Units::Length GetPositionY() const;

   Units::Length GetPositionZ() const;

   Units::Speed GetSpeedXd() const;

   Units::Speed GetSpeedYd() const;

   Units::Speed GetSpeedZd() const;

   Units::Speed GetTrueAirspeed() const;

   Units::SignedAngle GetLatitude() const;

   Units::SignedAngle GetLongitude() const;

   Units::Time GetTime() const;

   void SetPosition(Units::SignedAngle latitude, Units::SignedAngle longitude);

   aaesim::open_source::DynamicsState GetDynamicsState() const;

   bool operator==(const AircraftState &in) const;

   bool operator<(const AircraftState &in) const;

   int m_id;
   double m_time;
   double m_x, m_y, m_z;        // position (ft)
   double m_xd, m_yd, m_zd;     // speed (ft/s)
   double m_xdd, m_ydd, m_zdd;  // acceleration (ft/s^2)
   double m_gamma;
   double m_Vwx, m_Vwy;          // true wind direction meters/second
   double m_Vw_para, m_Vw_perp;  // true wind factors meters/second
   aaesim::open_source::DynamicsState m_dynamics_state;

   // Atmospheric conditions at current position
   Units::Temperature m_sensed_temperature;
   Units::Density m_sensed_density;
   Units::Pressure m_sensed_pressure;
   Units::SignedAngle m_latitude, m_longitude;

   Units::RadiansAngle m_psi;  // aircraft psi measured from east counter-clockwise

   // true wind vertical derivatives (speed per length == 1/time == frequency)
   Units::Frequency m_Vwx_dh;
   Units::Frequency m_Vwy_dh;

  private:
   static log4cplus::Logger logger;
};

inline void AircraftState::SetPsi(const Units::Angle psi_in) { m_psi = psi_in; }

inline double AircraftState::GetZd() const { return m_zd; }

inline Units::Length AircraftState::GetPositionX() const { return Units::FeetLength(m_x); }

inline Units::Length AircraftState::GetPositionY() const { return Units::FeetLength(m_y); }

inline Units::Length AircraftState::GetPositionZ() const { return Units::FeetLength(m_z); }

inline Units::Speed AircraftState::GetSpeedXd() const { return Units::FeetPerSecondSpeed(m_xd); }

inline Units::Speed AircraftState::GetSpeedYd() const { return Units::FeetPerSecondSpeed(m_yd); }

inline Units::Speed AircraftState::GetSpeedZd() const { return Units::FeetPerSecondSpeed(m_zd); }

inline Units::SignedAngle AircraftState::GetLatitude() const { return m_latitude; }

inline Units::SignedAngle AircraftState::GetLongitude() const { return m_longitude; }

inline void AircraftState::SetPosition(Units::SignedAngle latitude, Units::SignedAngle longitude) {
   m_latitude = latitude;
   m_longitude = longitude;
}

inline aaesim::open_source::DynamicsState AircraftState::GetDynamicsState() const { return m_dynamics_state; }

inline Units::Time AircraftState::GetTime() const { return Units::SecondsTime(m_time); }

}  // namespace open_source
}  // namespace aaesim
