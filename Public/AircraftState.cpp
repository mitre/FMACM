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
// Copyright 2017 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************
#include "public/AircraftCalculations.h"
#include "math/CustomMath.h"
#include "utility/micros.h"

log4cplus::Logger AircraftState::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AircraftState"));


AircraftState::AircraftState(void)
{
    id = -1;
    time = -1;
    x = 0; // assumed to be in feet
    y = 0; // assumed to be in feet
    z = 0; // assumed to be in feet
    xd = 0; // assumed to be in FPS
    yd = 0; // assumed to be in FPS
    setZd(0);	// assumed to be in FPS
    xdd = 0; // assumed to be in FPSS
    ydd = 0; // assumed to be in FPSS
    zdd = 0; // assumed to be in FPSS


    gamma = 0.0;
    psi							   = 0.0;

    Vwx = 0.0; // meters/second
    Vwy = 0.0; // meters/second

    Vw_para = 0.0; // meters/second
    Vw_perp = 0.0; // meters/second

    distToGo = -99999.99999; // meters
}
AircraftState::~AircraftState(void)
{

}

AircraftState::AircraftState (const AircraftState &in)
{
    id   = in.id;
    time = in.time;
    x    = in.x;
    y    = in.y;
    z    = in.z;
    xd   = in.xd;
    yd   = in.yd;
    setZd(in.zd);
    xdd  = in.xdd;
    ydd  = in.ydd;
    zdd  = in.zdd;


    gamma = in.gamma;
    psi							 = in.psi;

    Vwx = in.Vwx;
    Vwy = in.Vwy;

    Vw_para = in.Vw_para;
    Vw_perp = in.Vw_perp;

    distToGo = in.distToGo;
}

AircraftState& AircraftState::operator= (const AircraftState &in)
{
    if (this != &in) {
        id   = in.id;
        time = in.time;
        x    = in.x;
        y    = in.y;
        z    = in.z;
        xd   = in.xd;
        yd   = in.yd;
        setZd(in.zd);
        xdd  = in.xdd;
        ydd  = in.ydd;
        zdd  = in.zdd;

        gamma = in.gamma;
        psi							   = in.psi;

        Vwx = in.Vwx;
        Vwy = in.Vwy;

        Vw_para = in.Vw_para;
        Vw_perp = in.Vw_perp;

        distToGo = in.distToGo;
    }

    return *this;
}

bool AircraftState::operator== (const AircraftState &in) const
{

    bool same = true;

    same = same && (id == in.id) && (time == in.time);
    same = same && (x == in.x) && (y == in.y) && (z == in.z);
    same = same && (xd == in.xd) && (yd == in.yd) && (zd == in.zd);
    same = same && (xdd == in.xdd) && (ydd == in.ydd) && (zdd == in.zdd);
    same = same && (gamma == in.gamma);
    same = same && (Vwx == in.Vwx) && (Vwy == in.Vwy);
    same = same && (Vw_para == in.Vw_para) && (Vw_perp == in.Vw_perp);
    same = same && (psi == in.psi) && (distToGo == in.distToGo);

    return same;
}

bool AircraftState::is_turning() const
{
    //Determine if a turn is taking place: source nav_NSE.cpp of WinSS

    double spd = sqrt(SQR(xd) + SQR(yd));
    double turn_rate = (xd*ydd - yd*xdd) / spd;

    return ((ABS(turn_rate) > 1.5));
}

// operator < to allow sorting

bool AircraftState::operator<(const AircraftState &in) const
{
    bool result = false; // return value initialized to false

    // check if id is less, or if matching the time is less
    if( this->id < in.id || ( this->id == in.id && this->time < in.time ))
    {
        result = true; // sets return value to true
    }

    return result;
}


// heading methods
// get the aircraft heading in radians, clockwise from North (mathematical 90 degrees)
//gwang 2013-10: the function name should be get_ground_track, because xd and yd are ground speeds
double AircraftState::get_heading() const
{
    double result = 0.0;

    result = atan3(xd, yd); // takes the atan of x/y instead of y/x to find the angle from North clockwise, the result is in radians

    return result;
}

// gets the aircraft heading in radians, counter-clockwise from 0 degrees (mathematical)
Units::UnsignedRadiansAngle AircraftState::get_heading_in_radians_mathematical() const
{
    // gets mathematical 0 degrees counterclockwise position in radians

    double result = 0.0;

    result = atan3(yd, xd);

    return Units::UnsignedRadiansAngle(result);
}

// speed methods
Units::Speed AircraftState::getGroundSpeed(void) const
{
    return Units::FeetPerSecondSpeed(sqrt(SQR(xd) + SQR(yd)));
}

// psi getter/setters
void AircraftState::set_psi(double psi_in)
{
    psi = psi_in;
}

AircraftState AircraftState::createFromADSBReport(const Sensor::ADSB::ADSBSVReport &adsbsvReport) {
    AircraftState result;
    result.id = adsbsvReport.getId();
    result.time = adsbsvReport.getTime().value();
    result.x = adsbsvReport.getX().value();
    result.xd = adsbsvReport.getXd().value();
    result.y = adsbsvReport.getY().value();
    result.yd = adsbsvReport.getYd().value();
    result.z = adsbsvReport.getZ().value();
    result.setZd(adsbsvReport.getZd().value());

    return result;
}

void AircraftState::dumpParms(std::string str) const
{

    // Dumps selected AircraftState objects.
    // To use this, logger properties level must be set to DEBUG.
    //
    // str:Header string for output.
    LOG4CPLUS_DEBUG(AircraftState::logger,std::endl << "(Subset) Aircraft state parms for " << str.c_str());

    LOG4CPLUS_DEBUG(AircraftState::logger,std::endl << "time " << time << "  id " << id << "  distToGo " << distToGo);
    LOG4CPLUS_DEBUG(AircraftState::logger,std::endl << "position   x " << x << "  y " << y << "  z " << z);
    LOG4CPLUS_DEBUG(AircraftState::logger,std::endl << "speed      x " << xd << "  y " << yd << "  z " << zd);
    LOG4CPLUS_DEBUG(AircraftState::logger,std::endl << "Vw_para      " << Vw_para << "  Vw_perp " << Vw_perp);

}

void AircraftState::csvDataDump(std::string str) const
{

    // Dumps AircraftState data in csv format.
    // To use this, logger properties level must be set to DEBUG.
    //
    // str:Beginning of line string to output.
    LOG4CPLUS_DEBUG(AircraftState::logger,std::endl << str.c_str() << ","
                                                    << time << "," << id << "," << x << "," << y << ","
                                                    << z << "," << xd << "," << yd << "," << zd << ","
                                                    << xdd << "," << ydd << "," << zdd << "," << gamma << ","
                                                    << Vwx << "," << Vwy << "," << Vw_para << "," << Vw_perp << ","
                                                    << psi << "," << distToGo << "," << std::endl);

}

void AircraftState::csvHdrDump(std::string str) const
{

    // Dumps AircraftState header in csv format, based on
    // csvDataDump.  To use this, logger properties level
    // must be set to DEBUG.
    //
    // str:Beginning of line string to output.

    LOG4CPLUS_DEBUG(AircraftState::logger,std::endl
            << str.c_str() << ",time,id,x,y,z,x_Speed,y_Speed,z_Speed,x_Accel,y_Accel,z_Accel,gamma,Vwx,Vwy,Vw_para,Vw_perp,psi,distToGo"
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
AircraftState& AircraftState::interpolate(const AircraftState& a,
                                          const AircraftState& b, const double time) {

    if (a.id != b.id) {
        LOG4CPLUS_ERROR(logger, "Interpolating between states that have different ids:  "
                << a.id + " and " << b.id << ".");
    }

    const double baTimeDiff = b.time - a.time;
    double aWeight, bWeight;
    if (baTimeDiff == 0) {
        // avoid divide by zero
        aWeight = 1;
        bWeight = 0;
        LOG4CPLUS_ERROR(logger, "Attempt to interpolate between two states for the same time.");
    }
    else {
        aWeight = (b.time - time) / baTimeDiff;
        bWeight = 1 - aWeight;
    }
    this->id = a.id;
    this->time = time;
    this->x = a.x * aWeight + b.x * bWeight;
    this->y = a.y * aWeight + b.y * bWeight;
    this->z = a.z * aWeight + b.z * bWeight;
    this->xd = a.xd * aWeight + b.xd * bWeight;
    this->yd = a.yd * aWeight + b.yd * bWeight;
    this->setZd(a.zd * aWeight + b.zd * bWeight);

    return *this;
}

/**
 * Extrapolates from an input state using the velocity.
 * Only id, time, position, and velocity fields are written.
 * Other fields, such as acceleration, orientation, and wind,
 * are left unchanged.
 */
void AircraftState::extrapolate(const AircraftState& in,
                                const double         time,
                                AircraftState&       aircraft_state_to_return)
{
    double dt = time - in.time;

    aircraft_state_to_return.time = time;
    aircraft_state_to_return.id = in.id;
    aircraft_state_to_return.x = in.x + in.xd * dt;
    aircraft_state_to_return.y = in.y + in.yd * dt;
    aircraft_state_to_return.z = in.z + in.zd * dt;
    aircraft_state_to_return.xd = in.xd;
    aircraft_state_to_return.yd = in.yd;
    aircraft_state_to_return.setZd(in.zd);
}

double AircraftState::getZd() const {
    return zd;
}

void AircraftState::setZd(double zd) {
    if (abs(zd) > 200) {
        LOG4CPLUS_ERROR(logger, "zd out of range:  " << zd);
    }
    this->zd = zd;
}
