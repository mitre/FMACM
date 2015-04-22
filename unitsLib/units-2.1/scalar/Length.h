//
// $Id: Length.h,v 1.6.4.2 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of length.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Length_h
#define Units_Length_h

#include "SpecificUnit.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Length, 0, 1, 0, 0, 0, 0, 0, 0);

UNITS_DECLARE_SPECIFIC_UNIT(Length, MicrometersLength,   "um",  0.000001);
UNITS_DECLARE_SPECIFIC_UNIT(Length, MillimetersLength,   "mm",  0.001);
UNITS_DECLARE_SPECIFIC_UNIT(Length, CentimetersLength,   "cm",  0.01);
UNITS_DECLARE_SPECIFIC_UNIT(Length, MetersLength,        "m",   1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Length, KilometersLength,    "km",  1000.0);
UNITS_DECLARE_SPECIFIC_UNIT(Length, InchesLength,        "in",  0.3048/12);
UNITS_DECLARE_SPECIFIC_UNIT(Length, FeetLength,          "ft",  0.3048);
UNITS_DECLARE_SPECIFIC_UNIT(Length, NauticalMilesLength, "nmi", 1852.0);
UNITS_DECLARE_SPECIFIC_UNIT(Length, StatuteMilesLength,  "mi",  1609.344);


} // namespace Units


#endif  // Units_Length_h
