//
// $Id: UnsignedAngle.h,v 1.10.4.2 2007-10-01 05:44:30 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent an unsigned circular angle (0..360 degrees).
// (A circular angle wraps around; a regular Angle does not.)
//

#ifndef Units_UnsignedAngle_h
#define Units_UnsignedAngle_h

#ifndef UNITS_NO_IOSTREAM_OPERATORS
#include <iostream>
#endif

#include "SignedAngle.h"


namespace Units
{


template <typename ValueType>
class UnsignedAngleType: public UNITS_ANGLE_TYPE(ValueType)
{
public:
  //
  // Default constructor gives uninitialized value!
  //
  UnsignedAngleType();

  //
  // Allow implicit conversion because it's still just an angle.
  //
  UnsignedAngleType(UNITS_ANGLE_TYPE(ValueType) const & angle);

  UnsignedAngleType & operator+= (UNITS_ANGLE_TYPE(ValueType) const & angle);
  UnsignedAngleType & operator-= (UNITS_ANGLE_TYPE(ValueType) const & angle);

  // Return the smallest difference between the two angles.
  SignedAngleType<ValueType> operator- (UnsignedAngleType const & angle) const;

  // Make sure internal value is in range 0..360.
  UnsignedAngleType & normalize();


  ///
  /// Test the proper operation of the public API.  WARNING: This is
  /// for library testing only and not exported.
  ///
  static
  bool test();
};


//
// Comparison operators (with wrap-around)
//
template <typename ValueType>
bool similar(UnsignedAngleType<ValueType> const & a,
             UnsignedAngleType<ValueType> const & b,
             UNITS_ANGLE_TYPE(ValueType) const & tol
#ifndef _MSC_VER
             = UNITS_ANGLE_TYPE(ValueType)::defaultTolerance()
#endif
             );

#ifdef _MSC_VER
//
// MSVC++ doesn't appear to handle default args to template functions.
//
template <typename ValueType>
bool similar(UnsignedAngleType<ValueType> const & a,
             UnsignedAngleType<ValueType> const & b);
#endif

typedef UnsignedAngleType<double> dUnsignedAngle;
typedef UnsignedAngleType<float>  fUnsignedAngle;
typedef UnsignedAngleType<UNITS_DEFAULT_VALUE_TYPE> UnsignedAngle;


template <typename SpecificAngleUnits>
class SpecificUnsignedAngle:
    public UnsignedAngleType<typename SpecificAngleUnits::ValueType>
{
public:
  typedef typename SpecificAngleUnits::ValueType ValueType;

  //
  // Default constructor gives uninitialized value!
  //
  SpecificUnsignedAngle();
  SpecificUnsignedAngle(UNITS_ANGLE_TYPE(ValueType) const & angle);

  explicit
  SpecificUnsignedAngle(ValueType const value);

  //
  // Return this angle in 0..360 degrees.
  //
  ValueType value() const;
};

#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <typename SpecificAngleUnits>
std::ostream &
operator<< (std::ostream                                  & stream,
            SpecificUnsignedAngle<SpecificAngleUnits> const & angle);
#endif

typedef SpecificUnsignedAngle<dDegreesAngle> dUnsignedDegreesAngle;
typedef SpecificUnsignedAngle<fDegreesAngle> fUnsignedDegreesAngle;
typedef SpecificUnsignedAngle<DegreesAngle>   UnsignedDegreesAngle;

typedef SpecificUnsignedAngle<dRadiansAngle> dUnsignedRadiansAngle;
typedef SpecificUnsignedAngle<fRadiansAngle> fUnsignedRadiansAngle;
typedef SpecificUnsignedAngle<RadiansAngle>   UnsignedRadiansAngle;


} // namespace Units


#include "UnsignedAngle.i"

#endif  // Units_UnsignedAngle_h
