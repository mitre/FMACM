//
// $Id: Force.h,v 1.7.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of force.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Force_h
#define Units_Force_h

#include "SpecificUnit.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Force, 1, 1, -2, 0, 0, 0, 0, 0);

UNITS_DECLARE_SPECIFIC_UNIT(Force, NewtonsForce, "N",  1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Force, PoundsForce,  "lb", 4.4482216);
UNITS_DECLARE_SPECIFIC_UNIT(Force, OuncesForce,  "oz", 0.2780);


} // namespace Units


#endif  // Units_Force_h
