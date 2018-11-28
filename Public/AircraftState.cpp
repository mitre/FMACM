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

#include "public/AircraftCalculations.h"
#include "math/CustomMath.h"

log4cplus::Logger AircraftState::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AircraftState"));


AircraftState::AircraftState()
      :
      m_id(-1),
      m_time(-1),
      m_x(0),
      m_y(0),
      m_z(0),
      m_xd(0),
      m_yd(0),
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
   m_distance_to_go = -99999.99999; // meters
}

AircraftState::~AircraftState(void) {

}

AircraftState::AircraftState(const AircraftState &in) {
   m_id = in.m_id;
   m_time = in.m_time;
   m_x = in.m_x;
   m_y = in.m_y;
   m_z = in.m_z;
   m_xd = in.m_xd;
   m_yd = in.m_yd;
   SetZd(in.m_zd);
   m_xdd = in.m_xdd;
   m_ydd = in.m_ydd;
   m_zdd = in.m_zdd;


   m_gamma = in.m_gamma;
   m_psi = in.m_psi;

   m_Vwx = in.m_Vwx;
   m_Vwy = in.m_Vwy;

   m_Vw_para = in.m_Vw_para;
   m_Vw_perp = in.m_Vw_perp;
   m_Vwy_dh = in.m_Vwy_dh;
   m_Vwx_dh = in.m_Vwx_dh;

   m_distance_to_go = in.m_distance_to_go;
}

AircraftState &AircraftState::operator=(const AircraftState &in) {
   if (this != &in) {
      m_id = in.m_id;
      m_time = in.m_time;
      m_x = in.m_x;
      m_y = in.m_y;
      m_z = in.m_z;
      m_xd = in.m_xd;
      m_yd = in.m_yd;
      SetZd(in.m_zd);
      m_xdd = in.m_xdd;
      m_ydd = in.m_ydd;
      m_zdd = in.m_zdd;

      m_gamma = in.m_gamma;
      m_psi = in.m_psi;

      m_Vwx = in.m_Vwx;
      m_Vwy = in.m_Vwy;

      m_Vw_para = in.m_Vw_para;
      m_Vw_perp = in.m_Vw_perp;
      m_Vwy_dh = in.m_Vwy_dh;
      m_Vwx_dh = in.m_Vwx_dh;

      m_distance_to_go = in.m_distance_to_go;
   }

   return *this;
}

bool AircraftState::operator==(const AircraftState &in) const {

   bool same = true;

   same = same && (m_id == in.m_id) && (m_time == in.m_time);
   same = same && (m_x == in.m_x) && (m_y == in.m_y) && (m_z == in.m_z);
   same = same && (m_xd == in.m_xd) && (m_yd == in.m_yd) && (m_zd == in.m_zd);
   same = same && (m_xdd == in.m_xdd) && (m_ydd == in.m_ydd) && (m_zdd == in.m_zdd);
   same = same && (m_gamma == in.m_gamma);
   same = same && (m_Vwx == in.m_Vwx) && (m_Vwy == in.m_Vwy);
   same = same && (m_Vw_para == in.m_Vw_para) && (m_Vw_perp == in.m_Vw_perp);
   same = same && (m_Vwx_dh == in.m_Vwx_dh) && (m_Vwy_dh == in.m_Vwy_dh);
   same = same && (m_psi == in.m_psi) && (m_distance_to_go == in.m_distance_to_go);

   return same;
}

const bool AircraftState::IsTurning() const {
   //Determine if a turn is taking place: source nav_NSE.cpp of WinSS

   double spd = sqrt(pow(m_xd, 2) + pow(m_yd, 2));
   double turn_rate = (m_xd * m_ydd - m_yd * m_xdd) / spd;

   return ((std::fabs(turn_rate) > 1.5));
}

// operator < to allow sorting

bool AircraftState::operator<(const AircraftState &in) const {
   bool result = false; // return value initialized to false

   // check if id is less, or if matching the time is less
   if (m_id < in.m_id || (m_id == in.m_id && m_time < in.m_time)) {
      result = true; // sets return value to true
   }

   return result;
}


// heading methods
// get the aircraft heading in radians, clockwise from North (mathematical 90 degrees)
//gwang 2013-10: the function name should be get_ground_track, because xd and yd are ground speeds
const double AircraftState::GetHeading() const {
   double result = 0.0;

   result = atan3(m_xd,
                  m_yd); // takes the atan of x/y instead of y/x to find the angle from North clockwise, the result is in radians

   return result;
}

// gets the aircraft heading in radians, counter-clockwise from 0 degrees (mathematical)
const Units::UnsignedRadiansAngle AircraftState::GetHeadingInRadiansMathematical() const {
   // gets mathematical 0 degrees counterclockwise position in radians

   double result = 0.0;

   result = atan3(m_yd, m_xd);

   return Units::UnsignedRadiansAngle(result);
}


// speed methods
const Units::Speed AircraftState::GetGroundSpeed() const {
   return Units::FeetPerSecondSpeed(sqrt(pow(m_xd, 2) + pow(m_yd, 2)));
}


void AircraftState::SetPsi(const double psi_in) {
   m_psi = psi_in;
}

AircraftState AircraftState::CreateFromADSBReport(const Sensor::ADSB::ADSBSVReport &adsbsvReport) {
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

   // Dumps selected AircraftState objects.
   // To use this, logger properties level must be set to DEBUG.
   //
   // str:Header string for output.
   LOG4CPLUS_DEBUG(AircraftState::logger, std::endl << "(Subset) Aircraft state parms for " << str.c_str());

   LOG4CPLUS_DEBUG(AircraftState::logger,
                   std::endl << "time " << m_time << "  id " << m_id << "  distToGo " << m_distance_to_go);
   LOG4CPLUS_DEBUG(AircraftState::logger, std::endl << "position   x " << m_x << "  y " << m_y << "  z " << m_z);
   LOG4CPLUS_DEBUG(AircraftState::logger, std::endl << "speed      x " << m_xd << "  y " << m_yd << "  z " << m_zd);
   LOG4CPLUS_DEBUG(AircraftState::logger, std::endl << "Vw_para      " << m_Vw_para << "  Vw_perp " << m_Vw_perp);
   LOG4CPLUS_DEBUG(AircraftState::logger, std::endl << "Vwx      " << m_Vwx << "  Vwy " << m_Vwy);
   LOG4CPLUS_DEBUG(AircraftState::logger, std::endl << "Vwx_dh     " << Units::HertzFrequency(m_Vwx_dh) << "  Vwy_dh "
                                                    << Units::HertzFrequency(m_Vwy_dh));

}

void AircraftState::CsvDataDump(std::string str) const {

   LOG4CPLUS_DEBUG(AircraftState::logger,
                   std::endl << str.c_str() << ","
                             << m_time << "," << m_id << "," << m_x << "," << m_y << ","
                             << m_z << "," << m_xd << "," << m_yd << "," << m_zd << ","
                             << m_xdd << "," << m_ydd << "," << m_zdd << "," << m_gamma << ","
                             << m_Vwx << "," << m_Vwy << "," << m_Vw_para << "," << m_Vw_perp << ","
                             << Units::HertzFrequency(m_Vwx_dh) << "," << Units::HertzFrequency(m_Vwy_dh) << ","
                             << m_psi << "," << m_distance_to_go << "," << std::endl);

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
AircraftState &AircraftState::Interpolate(const AircraftState &a,
                                          const AircraftState &b,
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
AircraftState &AircraftState::Extrapolate(const AircraftState &in,
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

AircraftState &AircraftState::Extrapolate(const AircraftState &in,
                                          const Units::SecondsTime &time) {
   Extrapolate(in, time.value());
   return *this;
}


void AircraftState::SetZd(double zd) {
   if (abs(zd) > 200) {
      LOG4CPLUS_ERROR(logger, "zd out of range:  " << zd);
   }
   m_zd = zd;
}
