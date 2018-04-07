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

#pragma once

#include "utility/Logging.h"
#include "public/ADSBSVReport.h"
#include <UnsignedAngle.h>
#include <string>
#include "Speed.h"


// Aircraft data storage class, for the purpose of the various coversion methods the internal values are assumed to be in feet
class AircraftState
{
public:
    // Creational methods
    static AircraftState createFromADSBReport(const Sensor::ADSB::ADSBSVReport &adsbsvReport);

    AircraftState(void);
    ~AircraftState(void);

    AircraftState(const AircraftState &in);
    AircraftState& operator=(const AircraftState &in);

    bool is_turning() const;

    bool operator==(const AircraftState &in) const;

    // operator < to allow sorting
    bool operator<(const AircraftState &in) const;

    // heading methods
    // get the aircraft heading in radians, clockwise from North (mathematical 90 degrees)
    double get_heading() const;


    // gets the aircraft heading in radians, counter-clockwise from 0 degrees (mathematical)
    Units::UnsignedRadiansAngle get_heading_in_radians_mathematical() const;

    // psi getter/setters
    void set_psi(double psi_in);

    // speed methods
    Units::Speed getGroundSpeed(void) const;

    void dumpParms(std::string str) const;
    void csvHdrDump(std::string str) const;
    void csvDataDump(std::string str) const;

    AircraftState &interpolate(const AircraftState &a, const AircraftState &b, const double time);
    static void extrapolate(const AircraftState& in,
                            const double         time,
                            AircraftState&       aircraft_state_to_return);

    double getZd() const;
    void setZd(double zd);

    //Other Data:
    int id;
    double time;
    double x, y, z; //position (ft)
    double xd, yd, zd; //speed (ft/s)
    double xdd, ydd, zdd; //acceleration (f/s^2)
    double gamma;


    // Additional variables required by sense & avoid algorithm server
    //
    double Vwx, Vwy; // true wind direction meters/second
    double Vw_para, Vw_perp; // true wind factors meters/second

    double psi; // aircraft psi measured from east counter-clockwise

    double distToGo; // For state-model-output in meters.


private:

    static log4cplus::Logger logger;

};
