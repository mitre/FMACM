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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "utility/Logging.h"
#include "ADSBSVReport.h"
#include <UnsignedAngle.h>
#include <string>
#include "Speed.h"
#include <Frequency.h>


// Aircraft data storage class, for the purpose of the various coversion methods the internal values are assumed to be in feet

class AircraftState
{
public:
   // Creational methods
   static AircraftState CreateFromADSBReport(const Sensor::ADSB::ADSBSVReport &adsbsvReport);

   AircraftState();

   ~AircraftState();

   AircraftState(const AircraftState &in);

   AircraftState &operator=(const AircraftState &in);

   const bool IsTurning() const;

   bool operator==(const AircraftState &in) const;

   // operator < to allow sorting
   bool operator<(const AircraftState &in) const;

   // heading methods

   // get the aircraft heading in radians, clockwise from North (mathematical 90 degrees)
   const double GetHeading() const;


   // gets the aircraft heading in radians, counter-clockwise from 0 degrees (mathematical)
   const Units::UnsignedRadiansAngle GetHeadingInRadiansMathematical() const;

   // psi getter/setters
   void SetPsi(const double psi_in);

   // speed methods
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

   inline double GetZd() const {
      return m_zd;
   };

   void SetZd(const double zd);

//Other Data:
   int m_id;
   double m_time;
   double m_x, m_y, m_z; //position (ft)
   double m_xd, m_yd, m_zd; //speed (ft/s)
   double m_xdd, m_ydd, m_zdd; //acceleration (ft/s^2)
   double m_gamma;
   double m_Vwx, m_Vwy; // true wind direction meters/second
   double m_Vw_para, m_Vw_perp; // true wind factors meters/second
   double m_psi; // aircraft psi measured from east counter-clockwise
   double m_distance_to_go; // For state-model-output in meters.  FIXME remove this silly parameter from this class!
   Units::Frequency m_Vwx_dh, m_Vwy_dh; // true wind vertical derivatives (speed per length == 1/time == frequency)


private:

   static log4cplus::Logger logger;

};
