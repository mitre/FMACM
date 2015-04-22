//
// $Id: Acceleration.h,v 1.7.4.2 2005-12-22 06:15:56 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of acceleration.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Acceleration_h
#define Units_Acceleration_h

#include "SpecificUnit.h"
#include "Format.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Acceleration, 0, 1, -2, 0, 0, 0, 0, 0);


template <typename LengthUnit,
          typename TimeUnit>
class AccelerationFormat: public Format<typename LengthUnit::ValueType,
                                        UnspecifiedUnit,
                                        LengthUnit,
                                        TimeUnit,
                                        UnspecifiedUnit,
                                        UnspecifiedUnit,
                                        UnspecifiedUnit,
                                        UnspecifiedUnit,
                                        UnspecifiedUnit>
{
public:
  AccelerationFormat(Acceleration const & acceleration):
    Format<typename LengthUnit::ValueType,
           UnspecifiedUnit,
           LengthUnit,
           TimeUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit>(acceleration)
  {
  }
};


UNITS_DECLARE_SPECIFIC_UNIT(Acceleration, MetersSecondAcceleration,
                            "m/s^2",  1);
UNITS_DECLARE_SPECIFIC_UNIT(Acceleration, FeetSecondAcceleration,
                            "ft/s^2", 0.3048);
UNITS_DECLARE_SPECIFIC_UNIT(Acceleration, GsAcceleration,
                            "g",      9.80665);


} // namespace Units


#endif  // Units_Acceleration_h
