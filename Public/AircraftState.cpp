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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <iomanip>
#include "public/AircraftCalculations.h"
#include "math/CustomMath.h"

log4cplus::Logger AircraftState::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AircraftState"));


AircraftState::AircraftState()
      : m_id(-1),
        m_time(-1),
        m_x(0),
        m_y(0),
        m_z(0),
        m_xd(0),
        m_yd(0),
        m_zd(0),
        m_xdd(0),
        m_ydd(0),
        m_zdd(0.0),
        m_gamma(0.0),
        m_Vwx(0.0),
        m_Vwy(0.0),
        m_Vw_para(0.0),
        m_Vw_perp(0.0),
        m_psi(0.0) {
   SetZd(0);   // assumed to be in FPS
   m_Vwx_dh = Units::zero();
   m_Vwy_dh = Units::zero();
   m_distance_to_go_meters = -99999.99999;
}

AircraftState::~AircraftState() = default;

void AircraftState::Clear() {
   m_id = -1;
   m_time = -1;
   m_x = 0;
   m_y = 0;
   m_z = 0;
   m_xd = 0;
   m_yd = 0;
   m_zd = 0;
   m_xdd = 0;
   m_ydd = 0;
   m_zdd = 0;
   m_gamma = 0;
   m_Vwx = 0;
   m_Vwy = 0;
   m_Vw_para = 0;
   m_Vw_perp = 0;
   m_psi = Units::RadiansAngle(0);

   SetZd(0);
   m_Vwx_dh = Units::zero();
   m_Vwy_dh = Units::zero();
   m_distance_to_go_meters = -99999.99999;
}

bool AircraftState::operator==(const AircraftState& in) const {

   bool same = true;

   same = same && (m_id == in.m_id) && (m_time == in.m_time);
   same = same && (m_x == in.m_x) && (m_y == in.m_y) && (m_z == in.m_z);
   same = same && (m_xd == in.m_xd) && (m_yd == in.m_yd) && (m_zd == in.m_zd);
   same = same && (m_xdd == in.m_xdd) && (m_ydd == in.m_ydd) && (m_zdd == in.m_zdd);
   same = same && (m_gamma == in.m_gamma);
   same = same && (m_Vwx == in.m_Vwx) && (m_Vwy == in.m_Vwy);
   same = same && (m_Vw_para == in.m_Vw_para) && (m_Vw_perp == in.m_Vw_perp);
   same = same && (m_Vwx_dh == in.m_Vwx_dh) && (m_Vwy_dh == in.m_Vwy_dh);
   same = same && (m_psi == in.m_psi) && (m_distance_to_go_meters == in.m_distance_to_go_meters);

   return same;
}

bool AircraftState::operator<(const AircraftState& in) const {
   if (m_id < in.m_id || (m_id == in.m_id && m_time < in.m_time)) {
      return true;
   }

   return false;
}

const bool AircraftState::IsTurning() const {
   //Determine if a turn is taking place: source nav_NSE.cpp of WinSS

   double spd = sqrt(std::pow(m_xd, 2) + std::pow(m_yd, 2));
   double turn_rate = (m_xd * m_ydd - m_yd * m_xdd) / spd;

   return std::fabs(turn_rate) > 1.5;
}

const Units::UnsignedRadiansAngle AircraftState::GetHeadingCcwFromEastRadians() const {
   double result = atan3(m_yd, m_xd);
   return Units::UnsignedRadiansAngle(result);
}

const Units::Speed AircraftState::GetGroundSpeed() const {
   return Units::FeetPerSecondSpeed(sqrt(pow(m_xd, 2) + pow(m_yd, 2)));
}

AircraftState AircraftState::CreateFromADSBReport(const Sensor::ADSB::ADSBSVReport& adsbsvReport) {
   AircraftState result;
   result.m_id = adsbsvReport.GetId();
   result.m_time = adsbsvReport.GetTime().value();
   result.m_x = adsbsvReport.GetX().value();
   result.m_xd = adsbsvReport.GetXd().value();
   result.m_y = adsbsvReport.GetY().value();
   result.m_yd = adsbsvReport.GetYd().value();
   result.m_z = adsbsvReport.GetZ().value();
   result.SetZd(adsbsvReport.GetZd().value());

   return result;
}

void AircraftState::DumpParms(std::string str) const {
   LOG4CPLUS_DEBUG(AircraftState::logger, "aircraft state for " << str.c_str() << ":");
   LOG4CPLUS_DEBUG(AircraftState::logger, std::setprecision(12) << "time " << m_time << ", id " << m_id << ", distToGo (m) " << m_distance_to_go_meters);
   LOG4CPLUS_DEBUG(AircraftState::logger, std::setprecision(12) << "position (ft): x " << m_x << ", y " << m_y << ", z " << m_z);
   LOG4CPLUS_DEBUG(AircraftState::logger, std::setprecision(12) << "speed (ft/s): x " << m_xd << ", y " << m_yd << ", z " << m_zd);
   LOG4CPLUS_DEBUG(AircraftState::logger, std::setprecision(12) << "Wind Horiz Components (m/s): vwpara " << m_Vw_para << ", vwperp " << m_Vw_perp << ", Vwx " << m_Vwx << ", Vwy " << m_Vwy);
   LOG4CPLUS_DEBUG(AircraftState::logger, std::setprecision(12) << "Wind Vert Components (1/s): vwx_dh " << Units::HertzFrequency(m_Vwx_dh) << ", vwy_dh " << Units::HertzFrequency(m_Vwy_dh));
   LOG4CPLUS_DEBUG(AircraftState::logger, std::setprecision(12) << "psi_enu (deg): " << Units::SignedDegreesAngle (m_psi) << ", gamma (deg): " << Units::DegreesAngle(Units::RadiansAngle(m_gamma)) );
}

void AircraftState::CsvDataDump(std::string str) const {

   LOG4CPLUS_DEBUG(AircraftState::logger,
                   std::endl << str.c_str() << ","
                             << m_time << "," << m_id << "," << m_x << "," << m_y << ","
                             << m_z << "," << m_xd << "," << m_yd << "," << m_zd << ","
                             << m_xdd << "," << m_ydd << "," << m_zdd << "," << m_gamma << ","
                             << m_Vwx << "," << m_Vwy << "," << m_Vw_para << "," << m_Vw_perp << ","
                             << Units::HertzFrequency(m_Vwx_dh) << "," << Units::HertzFrequency(m_Vwy_dh) << ","
                             << m_psi << "," << m_distance_to_go_meters << "," << std::endl);

}

void AircraftState::CsvHdrDump(std::string str) const {

   LOG4CPLUS_DEBUG(AircraftState::logger, std::endl
         << str.c_str()
         << ",time,id,x,y,z,x_Speed,y_Speed,z_Speed,x_Accel,y_Accel,z_Accel,gamma,Vwx,Vwy,Vw_para,Vw_perp,psi,distToGo"
         << std::endl);

}

/**
 * Interpolates between (or extrapolates from) a pair of input states.
 * This interpolation is linear on position and velocity fields,
 * which is only mathematically consistent when velocity does not change.
 * Only id, time, position, and velocity fields are written.
 * Other fields, such as acceleration, orientation, and wind,
 * are left unchanged.
 */
AircraftState& AircraftState::Interpolate(const AircraftState& a,
                                          const AircraftState& b,
                                          const double time) {

   if (a.m_id != b.m_id) {
      LOG4CPLUS_ERROR(logger, "Interpolating between states that have different ids:  "
            << a.m_id + " and " << b.m_id << ".");
   }

   const double baTimeDiff = b.m_time - a.m_time;
   double aWeight, bWeight;
   if (baTimeDiff == 0) {
      // avoid divide by zero
      aWeight = 1;
      bWeight = 0;
      LOG4CPLUS_ERROR(logger, "Attempt to interpolate between two states for the same time.");
   } else {
      aWeight = (b.m_time - time) / baTimeDiff;
      bWeight = 1 - aWeight;
   }
   m_id = a.m_id;
   m_time = time;
   m_x = a.m_x * aWeight + b.m_x * bWeight;
   m_y = a.m_y * aWeight + b.m_y * bWeight;
   m_z = a.m_z * aWeight + b.m_z * bWeight;
   m_xd = a.m_xd * aWeight + b.m_xd * bWeight;
   m_yd = a.m_yd * aWeight + b.m_yd * bWeight;
   SetZd(a.m_zd * aWeight + b.m_zd * bWeight);

   return *this;
}

/**
 * Extrapolates from an input state using the velocity.
 * Only id, time, position, and velocity fields are written.
 * Other fields, such as acceleration, orientation, and wind,
 * are left unchanged.
 */
AircraftState& AircraftState::Extrapolate(const AircraftState& in,
                                          const double time_in) {
   double dt = time_in - in.m_time;

   m_time = Units::SecondsTime(time_in).value();
   m_id = in.m_id;
   m_x = in.m_x + in.m_xd * dt;
   m_y = in.m_y + in.m_yd * dt;
   m_z = in.m_z + in.m_zd * dt;
   m_xd = in.m_xd;
   m_yd = in.m_yd;
   SetZd(in.m_zd);

   return *this;
}

AircraftState& AircraftState::Extrapolate(const AircraftState& in,
                                          const Units::SecondsTime& time) {
   Extrapolate(in, time.value());
   return *this;
}


void AircraftState::SetZd(double zd) {
   if (fabs(zd) > 200) {
      LOG4CPLUS_ERROR(logger, "zd out of range:  " << zd);
   }
   m_zd = zd;
}


Units::Speed AircraftState::GetTrueAirspeed() const {
   Units::MetersPerSecondSpeed tas_x, tas_y;
   tas_x = Units::FeetPerSecondSpeed(m_xd) - Units::MetersPerSecondSpeed(m_Vwx);
   tas_y = Units::FeetPerSecondSpeed(m_yd) - Units::MetersPerSecondSpeed(m_Vwy);
   Units::MetersPerSecondSpeed tas = Units::sqrt(Units::sqr(tas_x) + Units::sqr(tas_y));
   return tas;
}
