//
// $Id: SignedAngle.h,v 1.10.4.2 2007-10-01 05:44:30 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent a signed circular angle (-180..180 degrees).
// (A circular angle wraps around; a regular Angle does not.)
//

#ifndef Units_SignedAngle_h
#define Units_SignedAngle_h

#ifndef UNITS_NO_IOSTREAM_OPERATORS
#include <iostream>
#endif

#include "Angle.h"


namespace Units
{


template <typename ValueType>
class SignedAngleType: public UNITS_ANGLE_TYPE(ValueType)
{
public:
  //
  // Default constructor gives uninitialized value!
  //
  SignedAngleType();

  //
  // Allow implicit conversion because it's still just an angle.
  //
  SignedAngleType(UNITS_ANGLE_TYPE(ValueType) const & angle);

  SignedAngleType & operator+= (UNITS_ANGLE_TYPE(ValueType) const & angle);
  SignedAngleType & operator-= (UNITS_ANGLE_TYPE(ValueType) const & angle);

  //
  // Return the smallest difference between the two angles.
  //
  SignedAngleType   operator- (SignedAngleType const & angle) const;

  //
  // Make sure internal value is in range -180..180.
  //
  SignedAngleType & normalize();


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
bool similar(SignedAngleType<ValueType> const & a,
             SignedAngleType<ValueType> const & b,
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
bool similar(SignedAngleType<ValueType> const & a,
             SignedAngleType<ValueType> const & b);
#endif

typedef SignedAngleType<double> dSignedAngle;
typedef SignedAngleType<float>  fSignedAngle;
typedef SignedAngleType<UNITS_DEFAULT_VALUE_TYPE> SignedAngle;


template <typename SpecificAngleUnits>
class SpecificSignedAngle:
    public SignedAngleType<typename SpecificAngleUnits::ValueType>
{
public:
  typedef typename SpecificAngleUnits::ValueType ValueType;

  //
  // Default constructor gives uninitialized value!
  //
  SpecificSignedAngle();
  SpecificSignedAngle(UNITS_ANGLE_TYPE(ValueType) const & angle);

  explicit
  SpecificSignedAngle(ValueType const value);

  //
  // Return this angle in -180..180 degrees.
  //
  ValueType value() const;
};

#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <typename SpecificAngleUnits>
std::ostream &
operator<< (std::ostream                                  & stream,
            SpecificSignedAngle<SpecificAngleUnits> const & angle);
#endif

typedef SpecificSignedAngle<dDegreesAngle> dSignedDegreesAngle;
typedef SpecificSignedAngle<fDegreesAngle> fSignedDegreesAngle;
typedef SpecificSignedAngle<DegreesAngle>   SignedDegreesAngle;

typedef SpecificSignedAngle<dRadiansAngle> dSignedRadiansAngle;
typedef SpecificSignedAngle<fRadiansAngle> fSignedRadiansAngle;
typedef SpecificSignedAngle<RadiansAngle>   SignedRadiansAngle;


} // namespace Units


#include "SignedAngle.i"

#endif  // Units_SignedAngle_h
