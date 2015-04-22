//
// $Id: MagneticFluxDensity.h,v 1.1.2.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of magnetic flux (force
// per magnetic field).  The internal representation of the units is
// hidden from the user.
//

#ifndef Units_MagneticFluxDensity_h
#define Units_MagneticFluxDensity_h

#include "SpecificUnit.h"
#include "RatioUnitFormat.h"


namespace Units
{

UNITS_DECLARE_BASE_UNIT(MagneticFluxDensity, 1, 2, -2, -1, 0, 0, 0, 0);

#define MagneticFluxDensityFormat RatioUnitFormat


UNITS_DECLARE_SPECIFIC_UNIT(MagneticFluxDensity,
                            TeslaMagneticFluxDensity, "T", 1.0);
UNITS_DECLARE_SPECIFIC_UNIT(MagneticFluxDensity,
                            GaussMagneticFluxDensity, "G", 1e-4);


} // namespace Units


#endif  // Units_MagneticFluxDensity_h
