//
// $Id: MagneticField.h,v 1.1.2.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of magnetic field
// (current per units length).  The internal representation of the
// units is hidden from the user.
//

#ifndef Units_MagneticField_h
#define Units_MagneticField_h

#include "SpecificUnit.h"
#include "RatioUnitFormat.h"


namespace Units
{

UNITS_DECLARE_BASE_UNIT(MagneticField, 0, -1, 0, 1, 0, 0, 0, 0);

#define MagneticFieldFormat RatioUnitFormat


UNITS_DECLARE_SPECIFIC_UNIT(MagneticField,
                            AmpsPerMeterMagneticField, "A/m", 1.0);
UNITS_DECLARE_SPECIFIC_UNIT(MagneticField,
                            OerstedsMagneticField, "Oe",
                            1000.0/(4*3.14159265358979323846));


} // namespace Units


#endif  // Units_MagneticField_h
