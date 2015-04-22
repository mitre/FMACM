//
// $Id: MassFlowRate.h,v 1.6.4.2 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of mass flow rate.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_MassFlowRate_h
#define Units_MassFlowRate_h

#include "SpecificUnit.h"
#include "RatioUnitFormat.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(MassFlowRate, 1, 0, -1, 0, 0, 0, 0, 0);


#define MassFlowRateFormat RatioUnitFormat


//
// 1 kg = 2.204622622 lb
//
UNITS_DECLARE_SPECIFIC_UNIT(MassFlowRate,
                            PoundsPerHourMassFlowRate,
                            "lb/hr", 1.0/(2.204622622*3600));

UNITS_DECLARE_SPECIFIC_UNIT(MassFlowRate,
                            KilogramsPerHourMassFlowRate,
                            "kg/hr", 1.0/3600);


} // namespace Units


#endif  // Units_MassFlowRate_h
