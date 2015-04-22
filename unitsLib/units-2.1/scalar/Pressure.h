//
// $Id: Pressure.h,v 1.7.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of pressure.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Pressure_h
#define Units_Pressure_h

#include "SpecificUnit.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Pressure, 1, -1, -2, 0, 0, 0, 0, 0);


//
// 1 psi       = 6894.757 Pa
// 1 atm       = 101325   Pa
// 1 inches Hg = 3386.38  Pa
// 1 bar       = 0.1     MPa
// 1 Torr      = 1       mm Hg
//
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, MillipascalsPressure, "mPa",       1e-3);
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, PascalsPressure,      "Pa",        1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, KilopascalsPressure,  "kPa",       1e3);
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, MegapascalsPressure,  "MPa",       1e6);
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, AtmospheresPressure,  "atm",
                            101325.0);
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, InchesHgPressure,     "inches Hg",
                            3386.38);
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, PsiPressure,          "psi",
                            6894.75726951454);
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, MillibarPressure,     "mbar",      100.0);
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, BarPressure,          "bar",       1e5);
UNITS_DECLARE_SPECIFIC_UNIT(Pressure, TorrPressure,         "Torr",      133.3);


} // namespace Units


#endif  // Units_Pressure_h
