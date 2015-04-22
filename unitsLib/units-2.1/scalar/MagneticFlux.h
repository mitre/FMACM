//
// $Id: MagneticFlux.h,v 1.1.2.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of magnetic flux
// (magnetic field density applied over an area).  The internal
// representation of the units is hidden from the user.
//

#ifndef Units_MagneticFlux_h
#define Units_MagneticFlux_h

#include "SpecificUnit.h"
#include "ProductUnitFormat.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(MagneticFlux, 1, 4, -2, -1, 0, 0, 0, 0);

#define MagneticFluxFormat ProductUnitFormat


UNITS_DECLARE_SPECIFIC_UNIT(MagneticFlux, WebersMagneticFlux,   "Wb", 1.0);
UNITS_DECLARE_SPECIFIC_UNIT(MagneticFlux, MaxwellsMagneticFlux, "Mx", 1e8);


} // namespace Units


#endif  // Units_MagneticFlux_h
