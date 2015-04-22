//
// $Id: AngularAcceleration.h,v 1.3.4.2 2005-12-22 06:15:56 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of angular acceleration.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_AngularAcceleration_h
#define Units_AngularAcceleration_h

#include "SpecificUnit.h"
#include "Format.h"
#include "Angle.h"  // for M_PI


namespace Units
{


UNITS_DECLARE_BASE_UNIT(AngularAcceleration, 0, 0, -2, 0, 0, 0, 0, 1);


template <typename AngleUnit,
          typename TimeUnit>
class AngularAccelerationFormat: public Format<typename AngleUnit::ValueType,
                                               UnspecifiedUnit,
                                               UnspecifiedUnit,
                                               TimeUnit,
                                               UnspecifiedUnit,
                                               UnspecifiedUnit,
                                               UnspecifiedUnit,
                                               UnspecifiedUnit,
                                               AngleUnit>
{
public:
  AngularAccelerationFormat(AngularAcceleration const & acceleration):
    Format<typename AngleUnit::ValueType,
           UnspecifiedUnit,
           UnspecifiedUnit,
           TimeUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           AngleUnit>(acceleration)
  {
  }
};


UNITS_DECLARE_SPECIFIC_UNIT(AngularAcceleration,
                            RadiansSecondAngularAcceleration,
                            "rad/s^2", 1);
UNITS_DECLARE_SPECIFIC_UNIT(AngularAcceleration,
                            DegreesSecondAngularAcceleration,
                            "deg/s^2", M_PI/180);


} // namespace Units


#endif  // Units_AngularAcceleration_h
