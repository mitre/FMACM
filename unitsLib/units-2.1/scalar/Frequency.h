//
// $Id: Frequency.h,v 1.6.4.2 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of frequency.  Subclasses allow
// interpretation in specified units.  The internal representation of
// the units is hidden from the user.
//

#ifndef Units_Frequency_h
#define Units_Frequency_h

#include "SpecificUnit.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Frequency, 0, 0, -1, 0, 0, 0, 0, 0);

UNITS_DECLARE_SPECIFIC_UNIT(Frequency, HertzFrequency,     "Hz",  1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Frequency, KilohertzFrequency, "KHz", 1.0e3);
UNITS_DECLARE_SPECIFIC_UNIT(Frequency, MegahertzFrequency, "MHz", 1.0e6);
UNITS_DECLARE_SPECIFIC_UNIT(Frequency, GigahertzFrequency, "GHz", 1.0e9);
UNITS_DECLARE_SPECIFIC_UNIT(Frequency, TerahertzFrequency, "THz", 1.0e12);


} // namespace Units


#endif  // Units_Frequency_h
