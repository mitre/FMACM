//
// $Id: Speed.h,v 1.6.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of speed.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Speed_h
#define Units_Speed_h

#include "SpecificUnit.h"
#include "RatioUnitFormat.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Speed, 0, 1, -1, 0, 0, 0, 0, 0);


#define SpeedFormat RatioUnitFormat


//
// 1 m/s  = 3.280839895 ft/s = 2.236936292 mi/hr
// 1 knot = 1 nmi/hr = 1852 m/h
//
UNITS_DECLARE_SPECIFIC_UNIT(Speed, MetersPerSecondSpeed,   "m/s", 1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Speed, FeetPerSecondSpeed,     "fps", 0.3048);
UNITS_DECLARE_SPECIFIC_UNIT(Speed, KilometersPerHourSpeed, "kph", 1/3.6);
// Statute miles per hour
UNITS_DECLARE_SPECIFIC_UNIT(Speed, MilesPerHourSpeed,      "mph", 1/2.236936292);
// Nautical miles per hour
UNITS_DECLARE_SPECIFIC_UNIT(Speed, KnotsSpeed,             "kts", 1852/3600.0);


} // namespace Units


#endif  // Units_Speed_h
