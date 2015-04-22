//
// $Id: Time.h,v 1.7.4.2 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of time.  Subclasses allow interpretation
// in specified units.  The internal representation of the units is hidden
// from the user.
//

#ifndef Units_Time_h
#define Units_Time_h

#include "SpecificUnit.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Time, 0, 0, 1, 0, 0, 0, 0, 0);

UNITS_DECLARE_SPECIFIC_UNIT(Time, MicrosecondsTime, "us",      0.000001);
UNITS_DECLARE_SPECIFIC_UNIT(Time, MillisecondsTime, "ms",      0.001);
UNITS_DECLARE_SPECIFIC_UNIT(Time, SecondsTime,      "s",       1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Time, MinutesTime,      "min",    60.0);
UNITS_DECLARE_SPECIFIC_UNIT(Time, HoursTime,        "hr",   3600.0);
UNITS_DECLARE_SPECIFIC_UNIT(Time, DaysTime,         "days", 24*3600.0);


} // namespace Units


#endif  // Units_Time_h
