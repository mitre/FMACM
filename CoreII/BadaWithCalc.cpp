// ****************************************************************************
//  Copyright © 2015 The MITRE Corporation. All Rights Reserved.  
// ****************************************************************************

#include "BadaWithCalc.h"
#include "AircraftCalculations.h"
#include "micros.h"
#include "constants.h"
#include <cstdlib>

using namespace std;

BadaWithCalc::BadaWithCalc(void)
{
    cout << "BadaWithCalc.cpp: Implement this class!" << endl;
    ac_mass = mass.m_ref;   // default
}

BadaWithCalc::~BadaWithCalc(void)
{
}

/**
 * getAircraftParameters() does two things:
 *
 * 1.  Loads the data files by calling init(), if not already done.
 * 2.  Sets ac_mass based on the percentile.
 */
void BadaWithCalc::getAircraftParameters(string aircraft_type, double mass_percentile)
{
// INPUTS:
//   aircraft type
// OUTPUTS:
//   struct ac:
//       ac.mass, ac.ref_mass: kg
//       ac.S: m^2
//       ac.__.cd0,cd2,gear: unitless
//       ac.__.Vstall: kts (CAS)


}


// Gets configuration when flying trajectory 

//-----------------
// INPUTS:
//   ac: aircraft parameter structure
//   Calibrated air speed, V_cas: kts
//   alt: altitude in meters.
//   altAtFAF: altitude at FAF (or
//   last waypoint) in meters.
// OUTPUTS:
//   cd0,cd2,gear: drag coefficients for the specific configuration
void BadaWithCalc::getConfig(double V_cas, double alt, double altAtFAF, int modeLast, double &cd0, double &cd2, double &gear, int &mode)
{

}



// Gets configuration for trajectory building.

//-----------------
// INPUTS:
//   ac: aircraft parameter structure
//   Calibrated air speed, V_cas: kts
//   alt: altitude in meters.
//   altAtFAF: altitude at FAF (or
//   last waypoint) in meters.
// OUTPUTS:
//   cd0,cd2,gear: drag coefficients for the specific configuration
void BadaWithCalc::getConfigTrajGen(double V_cas, double alt, double altAtFAF, double &cd0, double &cd2, double &gear, int &mode)
{

}

// Configures Aircraft is extra drag is needed and speed is less than max configuration speed

//-----------------
// INPUTS:
//   ac: aircraft parameter structure
//   Calibrated air speed, V_cas: kts
//   alt: altitude in meters.
//   altAtFAF: altitude at FAF (or
//   last waypoint) in meters.
// OUTPUTS:
//   mode: configuration
void BadaWithCalc::getConfigForDrag(double V_cas, double alt, double altAtFAF, int modeLast, int &mode)
{
	
}


// gets maximum aircraft thrust, takes input as altitude in Meters
double BadaWithCalc::getMaxThrust(double h, int mode, string type)
{
	double Tmax = 0;

	return Tmax;
}

void BadaWithCalc::setFlapsSpeeds(string acType) {

}
