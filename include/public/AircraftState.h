// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************
#pragma once

#include <string>

#include "utility/Logging.h"
#include "public/ADSBSVReport.h"

#include "UnsignedAngle.h"
#include "Speed.h"
#include "Frequency.h"

// Aircraft data storage class, for the purpose of the various coversion methods the internal values are assumed to be in feet
class AircraftState
{
public:
   static AircraftState CreateFromADSBReport(const Sensor::ADSB::ADSBSVReport &adsbsvReport);

   AircraftState();

   virtual ~AircraftState();

   AircraftState(const AircraftState &in);

   const Units::UnsignedRadiansAngle GetHeadingCcwFromEastRadians() const;

   void SetPsi(const Units::Angle psi_in);

   const Units::Speed GetGroundSpeed() const;

   void DumpParms(std::string str) const;

   void CsvHdrDump(std::string str) const;

   void CsvDataDump(std::string str) const;

   AircraftState &Interpolate(const AircraftState &a,
                              const AircraftState &b,
                              const double time);

   /**
    * @deprecated use ::extrapolate(const AircraftState &in, const Units::SecondsTime &time)
    * @param in
    * @param time
    * @return
    */
   AircraftState &Extrapolate(const AircraftState &in,
                              const double time); // deprecated!

   /**
    * Returns a state that has been fully populated assuming constant ground speed.
    * @param in
    * @param time
    * @return
    */
   AircraftState &Extrapolate(const AircraftState &in,
                              const Units::SecondsTime &time);


   double GetZd() const;

   void SetZd(const double zd);

   AircraftState &operator=(const AircraftState &in);

   bool operator==(const AircraftState &in) const;

   bool operator<(const AircraftState &in) const;

   int m_id;
   double m_time;
   double m_x, m_y, m_z; //position (ft)
   double m_xd, m_yd, m_zd; //speed (ft/s)
   double m_xdd, m_ydd, m_zdd; //acceleration (ft/s^2)
   double m_gamma;
   double m_Vwx, m_Vwy; // true wind direction meters/second
   double m_Vw_para, m_Vw_perp; // true wind factors meters/second
   Units::RadiansAngle m_psi; // aircraft psi measured from east counter-clockwise
   double m_distance_to_go_meters;

   // true wind vertical derivatives (speed per length == 1/time == frequency)
   Units::Frequency m_Vwx_dh;
   Units::Frequency m_Vwy_dh;

private:
   static log4cplus::Logger logger;
};

inline void AircraftState::SetPsi(const Units::Angle psi_in) {
   m_psi = psi_in;
}

inline double AircraftState::GetZd() const {
   return m_zd;
}
