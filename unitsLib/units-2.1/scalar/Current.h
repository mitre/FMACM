//
// $Id: Current.h,v 1.4.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of current.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Current_h
#define Units_Current_h

#include "SpecificUnit.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Current, 0, 0, 0, 1, 0, 0, 0, 0);


UNITS_DECLARE_SPECIFIC_UNIT(Current, MicroampsCurrent, "uA", 1e-6);
UNITS_DECLARE_SPECIFIC_UNIT(Current, MilliampsCurrent, "mA", 1e-3);
UNITS_DECLARE_SPECIFIC_UNIT(Current, AmpsCurrent,      "A",  1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Current, KiloampsCurrent,  "kA", 1e3);


} // namespace Units


#endif  // Units_Current_h
