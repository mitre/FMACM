//
// $Id: Power.h,v 1.7.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of power.  Subclasses allow interpretation
// in specified units.  The internal representation of the units is hidden
// from the user.
//

#ifndef Units_Power_h
#define Units_Power_h

#include "SpecificUnit.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Power, 1, 2, -3, 0, 0, 0, 0, 0);


UNITS_DECLARE_SPECIFIC_UNIT(Power, MilliwattsPower, "mW", 1e-3);
UNITS_DECLARE_SPECIFIC_UNIT(Power, WattsPower,      "W",  1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Power, KilowattsPower,  "kW", 1e3);
UNITS_DECLARE_SPECIFIC_UNIT(Power, MegawattsPower,  "MW", 1e6);
UNITS_DECLARE_SPECIFIC_UNIT(Power, GigawattsPower,  "GW", 1e9);


} // namespace Units


#endif  // Units_Power_h
