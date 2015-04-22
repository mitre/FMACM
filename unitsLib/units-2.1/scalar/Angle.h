//
// $Id: Angle.h,v 1.6.4.2 2005-12-16 06:27:29 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of angle.
//
// This is *not* a circular angle -- it does not wrap around.
// That means DegreesAngle(370) != DegreesAngle(10).  The rationale
// for this is that some applications may need to measure angular
// distances which are greater than 360 degrees, such as measuring
// how far a wheel has turned.
//
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Angle_h
#define Units_Angle_h

#include <math.h>
#include "SpecificUnit.h"

#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Angle, 0, 0, 0, 0, 0, 0, 0, 1);

//TODO:
//UNITS_DECLARE_BASE_UNIT(SolidAngle, 0, 0, 0, 0, 0, 0, 0, 2);


// Shorthand
#define UNITS_ANGLE_TYPE(ValueType_) Unit<ValueType_, 0, 0, 0, 0, 0, 0, 0, 1>


//
// Trigonometric functions.
//

#define UNITS_DECLARE_TRIG_FUNCTION_FOR_TYPE(function, template_args, ValueType_) \
template <template_args> \
ValueType_ function(UNITS_ANGLE_TYPE(ValueType_) const & angle); \
\
template <template_args> \
UNITS_ANGLE_TYPE(ValueType_) arc##function(ValueType_ const value)



#define UNITS_DECLARE_TRIG_FUNCTION(function) \
UNITS_DECLARE_TRIG_FUNCTION_FOR_TYPE(function, typename ValueType, ValueType); \
UNITS_DECLARE_TRIG_FUNCTION_FOR_TYPE(function, , double); \
UNITS_DECLARE_TRIG_FUNCTION_FOR_TYPE(function, , float)


UNITS_DECLARE_TRIG_FUNCTION(sin);
UNITS_DECLARE_TRIG_FUNCTION(cos);
UNITS_DECLARE_TRIG_FUNCTION(tan);


template <typename ValueType>
UNITS_ANGLE_TYPE(ValueType) arctan2(ValueType const value1,
                                    ValueType const value2);
template <>
UNITS_ANGLE_TYPE(double)    arctan2(double const value1,
                                    double const value2);
template <>
UNITS_ANGLE_TYPE(float)     arctan2(float const value1,
                                    float const value2);


//
// Normalize internal representation to compensate for "drift".
// This keeps the angle in the range -360..360 degrees.
// You might want to use this after doing a lot of Angle arithmetic,
// but it shouldn't usually be necessary except for in extreme cases.
//
template <typename ValueType>
UNITS_ANGLE_TYPE(ValueType) &
normalize(UNITS_ANGLE_TYPE(ValueType) & angle);

template <>
UNITS_ANGLE_TYPE(double) &
normalize(UNITS_ANGLE_TYPE(double) & angle);

template <>
UNITS_ANGLE_TYPE(float) &
normalize(UNITS_ANGLE_TYPE(float) & angle);


UNITS_DECLARE_SPECIFIC_UNIT(Angle, RadiansAngle, "rad", 1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Angle, DegreesAngle, "deg", M_PI/180.0);


} // namespace Units


#include "Angle.i"

#endif  // Units_Angle_h
