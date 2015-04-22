//
// $Id: Mass.h,v 1.7.4.2 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of mass.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Mass_h
#define Units_Mass_h

#include "SpecificUnit.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Mass, 1, 0, 0, 0, 0, 0, 0, 0);

UNITS_DECLARE_SPECIFIC_UNIT(Mass, GramsMass,     "g",   1e-3);
UNITS_DECLARE_SPECIFIC_UNIT(Mass, KilogramsMass, "kg",  1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Mass, PoundsMass,    "lb",  1/2.204622622);
UNITS_DECLARE_SPECIFIC_UNIT(Mass, OuncesMass,    "oz",  1/35.27);


} // namespace Units


#endif  // Units_Mass_h
