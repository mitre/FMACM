//
// $Id: Angle.i,v 1.1.1.1.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Inline implementations for the Angle class.
//

#ifndef Units_Angle_i
#define Units_Angle_i


namespace Units
{


// Trigonometric functions.

#define UNITS_DEFINE_TRIG_FUNCTION(function) \
template <> \
inline \
double function(UNITS_ANGLE_TYPE(double) const & angle) \
{ \
  return ::function(dRadiansAngle(angle).value()); \
} \
\
template <> \
inline \
UNITS_ANGLE_TYPE(double) arc##function(double const value) \
{ \
  return dRadiansAngle(::a##function(value)); \
} \
\
template <> \
inline \
float     function(UNITS_ANGLE_TYPE(float) const & angle) \
{ \
  return ::function##f(fRadiansAngle(angle).value()); \
} \
\
template <> \
inline \
UNITS_ANGLE_TYPE(float) arc##function(float const value) \
{ \
  return fRadiansAngle(::a##function##f(value)); \
}

UNITS_DEFINE_TRIG_FUNCTION(sin)
UNITS_DEFINE_TRIG_FUNCTION(cos)
UNITS_DEFINE_TRIG_FUNCTION(tan)


template <>
inline
UNITS_ANGLE_TYPE(double) arctan2(double const value1,
                                 double const value2)
{
  return dRadiansAngle(atan2(value1,
                             value2));
}

template <>
inline
UNITS_ANGLE_TYPE(float) arctan2(float const value1,
                                float const value2)
{
  return fRadiansAngle(atan2f(value1,
                              value2));
}


template <>
inline
UNITS_ANGLE_TYPE(double) &
normalize(UNITS_ANGLE_TYPE(double) & angle)
{
  angle = dRadiansAngle(fmod(dRadiansAngle(angle).value(),
                             2.0*M_PI));
  return angle;
}

template <>
inline
UNITS_ANGLE_TYPE(float) &
normalize(UNITS_ANGLE_TYPE(float) & angle)
{
  angle = fRadiansAngle(fmodf(fRadiansAngle(angle).value(),
                              2.0*M_PI));
  return angle;
}


} // namespace Units


#endif  // Units_Angle_i
