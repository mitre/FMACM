//
// $Id: Voltage.h,v 1.4.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of voltage.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Voltage_h
#define Units_Voltage_h

#include "SpecificUnit.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Voltage, 1, 2, -3, -1, 0, 0, 0, 0);


UNITS_DECLARE_SPECIFIC_UNIT(Voltage, MicrovoltsVoltage, "uV", 1e-6);
UNITS_DECLARE_SPECIFIC_UNIT(Voltage, MillivoltsVoltage, "mV", 1e-3);
UNITS_DECLARE_SPECIFIC_UNIT(Voltage, VoltsVoltage,      "V",  1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Voltage, KilovoltsVoltage,  "kV", 1e3);


} // namespace Units


#endif  // Units_Voltage_h
