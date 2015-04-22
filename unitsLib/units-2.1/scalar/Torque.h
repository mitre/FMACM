//
// $Id: Torque.h,v 1.7.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of torque.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Torque_h
#define Units_Torque_h

#include "SpecificUnit.h"
#include "ProductUnitFormat.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Torque, 1, 2, -2, 0, 0, 0, 0, 0);

#define TorqueFormat ProductUnitFormat


UNITS_DECLARE_SPECIFIC_UNIT(Torque, NewtonMeterTorque, "N-m", 1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Torque, FootPoundTorque, 
                            "ft-lb", 0.3048*4.4482216);


} // namespace Units


#endif  // Units_Torque_h
