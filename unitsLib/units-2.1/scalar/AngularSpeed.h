//
// $Id: AngularSpeed.h,v 1.7.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of angular speed.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_AngularSpeed_h
#define Units_AngularSpeed_h

#include "SpecificUnit.h"
#include "RatioUnitFormat.h"
#include "Angle.h"  // for M_PI


namespace Units
{


UNITS_DECLARE_BASE_UNIT(AngularSpeed, 0, 0, -1, 0, 0, 0, 0, 1);


#define AngularSpeedFormat RatioUnitFormat


UNITS_DECLARE_SPECIFIC_UNIT(AngularSpeed,
                            RadiansPerSecondAngularSpeed, "rad/s", 1.0);
UNITS_DECLARE_SPECIFIC_UNIT(AngularSpeed,
                            DegreesPerSecondAngularSpeed, "deg/s", M_PI/180.0);
UNITS_DECLARE_SPECIFIC_UNIT(AngularSpeed,
                            RpmAngularSpeed,              "rpm", 2*M_PI/60.0);


} // namespace Units


#endif  // Units_AngularSpeed_h
