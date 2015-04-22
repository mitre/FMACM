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
// Copyright 2015 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

/*
 * custom_units.h
 *
 *  Created on: Feb 2, 2015
 *      Author: klewis
 *
 * This header file contains definitions for unit types which are not
 * already defined in unitsLib.
 */

#ifndef CUSTOM_UNITS_H_
#define CUSTOM_UNITS_H_

#include <Acceleration.h>
#include <Angle.h>
#include <Pressure.h>
#include <Speed.h>
#include <Time.h>
#include "constants.h"

namespace Units
{

// Acceleration
UNITS_DECLARE_SPECIFIC_UNIT(Acceleration, KnotsPerSecondAcceleration,
                            "kts/s", 1852.0 / 3600.0);
// acceleration constants
const Units::Acceleration ONE_G_ACCELERATION = MetersSecondAcceleration(G);

// angle constants
const Units::Angle PI_RADIANS_ANGLE = DegreesAngle(180);
const Units::Angle ONE_RADIAN_ANGLE = RadiansAngle(1);

// change rate of force
UNITS_DECLARE_BASE_UNIT(ForceChange, 1, 1, -3, 0, 0, 0, 0, 0);
UNITS_DECLARE_SPECIFIC_UNIT(ForceChange, NewtonsPerSecondForceChange, "N/s",  1.0);

// inverted length, for per-meter
UNITS_DECLARE_BASE_UNIT(InvertedLength, 0, -1, 0, 0, 0, 0, 0, 0);
UNITS_DECLARE_SPECIFIC_UNIT(InvertedLength, PerMeterInvertedLength, "1/m", 1.0);

// pressure in kilograms(weight) per square meter
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, KilogramsMeterPressure, "kg/m^2", G);

// inverted speed
//
//
// 1 m/s  = 3.280839895 ft/s = 2.236936292 mi/hr
// 1 knot = 1 nmi/hr = 1852 m/h
//

UNITS_DECLARE_BASE_UNIT(InvertedSpeed, 0, -1, 1, 0, 0, 0, 0, 0);
UNITS_DECLARE_SPECIFIC_UNIT(InvertedSpeed, SecondsPerNauticalMileInvertedSpeed, "s/nmi", (1.0/1852.0));
UNITS_DECLARE_SPECIFIC_UNIT(InvertedSpeed, KnotsInvertedSpeed, "1/kts", (3600.0/1852.0));
UNITS_DECLARE_SPECIFIC_UNIT(InvertedSpeed, SecondsPerMeterInvertedSpeed, "s/m", 1.0);

UNITS_DECLARE_BASE_UNIT(LengthGain, 0, 1, -2, 0, 0, 0, 0, 0);
UNITS_DECLARE_SPECIFIC_UNIT(LengthGain, MetersPerSecondSquaredLengthGain, "m/s^2", 1.0);

UNITS_DECLARE_BASE_UNIT(InvertedLengthGain, 0, -1, 2, 0, 0, 0, 0, 0);
UNITS_DECLARE_SPECIFIC_UNIT(InvertedLengthGain, SecondsSquaredPerMeterInvertedLengthGain, "s^2/m", 1.0);

// Speed
UNITS_DECLARE_SPECIFIC_UNIT(Speed, FeetPerMinute, "fpm", 0.3048 * 60.0);

// Convenience Zeros
const Units::Speed ZERO_SPEED = Units::MetersPerSecondSpeed(0.0);
const Units::Time ZERO_TIME = Units::SecondsTime(0.0);
const Units::Length ZERO_LENGTH = Units::MetersLength(0.0);
const InvertedSpeed ZERO_INVERTED_SPEED = Units::SecondsPerMeterInvertedSpeed(0.0);
const InvertedLengthGain ZERO_INVERTED_LENGTH_GAIN = Units::SecondsSquaredPerMeterInvertedLengthGain(0.0);

}
#endif /* CUSTOM_UNITS_H_ */
