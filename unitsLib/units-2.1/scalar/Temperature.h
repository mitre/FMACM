//
// $Id: Temperature.h,v 1.8.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of temperature.
// The internal representation of the units is hidden from the user.
//
// NOTE: There are two types of temperatures: relative and absolute.
// The generic Temperature class is relative and can be used in
// arbitrary arithmetic and combined with other units of measure
// (i.e., Temperature/Voltage).  AbsoluteTemperature has an offset and
// therefore cannot be scaled or combined with other units of measure.
//

#ifndef Units_Temperature_h
#define Units_Temperature_h

#ifndef UNITS_NO_IOSTREAM_OPERATORS
#include <iostream>
#endif

#include "SpecificUnit.h"



namespace Units
{


UNITS_DECLARE_BASE_UNIT(Temperature, 0, 0, 0, 0, 1, 0, 0, 0);

UNITS_DECLARE_SPECIFIC_UNIT(Temperature, CelsiusTemperature,    "C", 1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Temperature, KelvinTemperature,     "K", 1.0);
UNITS_DECLARE_SPECIFIC_UNIT(Temperature, FahrenheitTemperature, "F", 5.0/9.0);
UNITS_DECLARE_SPECIFIC_UNIT(Temperature, RankineTemperature,    "R", 5.0/9.0);


///
/// Represents an absolute temperature.
///
template <typename ValueType_>
class AbsTemperature_
{
public:
  typedef AbsTemperature_ BaseUnitType;

  // access the template parameters
  typedef ValueType_ ValueType;


  ///
  /// Default constructor gives uninitialized value!
  ///
  AbsTemperature_();


  // Comparison operators

  bool      similarTo  (AbsTemperature_ const & b,
                        Temperature     const & tol =
                        Temperature::defaultTolerance()) const;

  bool      operator== (AbsTemperature_ const & unit)     const;
  bool      operator!= (AbsTemperature_ const & unit)     const;
  bool      operator<  (AbsTemperature_ const & unit)     const;
  bool      operator>  (AbsTemperature_ const & unit)     const;
  bool      operator<= (AbsTemperature_ const & unit)     const;
  bool      operator>= (AbsTemperature_ const & unit)     const;


  // Arithmetic operators

  Temperature       operator-  (AbsTemperature_ const & unit) const;
  AbsTemperature_   operator+  (Temperature     const & unit) const;
  AbsTemperature_   operator-  (Temperature     const & unit) const;

  AbsTemperature_ & operator+= (Temperature     const & unit);
  AbsTemperature_ & operator-= (Temperature     const & unit);

protected:
  ValueType _value;

  explicit
  AbsTemperature_(ValueType const value);
};


template <typename ValueType_>
bool similar (AbsTemperature_<ValueType_> const & a,
              AbsTemperature_<ValueType_> const & b,
              Temperature                 const & tol =
              Temperature::defaultTolerance());


typedef AbsTemperature_<double> dAbsTemperature;
typedef AbsTemperature_<float> fAbsTemperature;
typedef AbsTemperature_<UNITS_DEFAULT_VALUE_TYPE> AbsTemperature;


template <typename ValueType,
          typename SpecificTemperature>
struct SpecificAbsTemperatureTraits
{
  typedef SpecificTemperature RelativeTemperature;
  static const double celsiusOffset;
};

template <typename ValueType,
          typename SpecificTemperatureTraits>
class SpecificAbsTemperature: public AbsTemperature_<ValueType>
{
public:
  typedef AbsTemperature_<ValueType> BaseUnitType;

  ///
  /// Default constructor gives uninitialized value!
  ///
  SpecificAbsTemperature();

  explicit
  SpecificAbsTemperature(ValueType    const   value);

  SpecificAbsTemperature(BaseUnitType const & temperature);

  static
  char const * const & unitsString();

  ValueType value() const;
};


#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <typename ValueType,
          typename SpecificTemperatureTraits>
std::ostream &
operator<< (std::ostream                                            & stream,
            SpecificAbsTemperature<ValueType,
                                   SpecificTemperatureTraits> const & temperature);
#endif


//
// Macro to allow declaration of subclasses.
//
// Notes:  No comments are allowed in the macro definition.
//         No blanks can follow the backslashes (\).
//
#define UNITS_DECLARE_SPECIFIC_TEMPERATURE(TemperatureType, celsiusOffset_) \
template <typename ValueType> \
struct SpecificAbsTemperatureTraits<ValueType, TemperatureType> \
{ \
  typedef TemperatureType RelativeTemperature; \
  static double const celsiusOffset; \
}; \
\
template <typename ValueType> \
double const \
SpecificAbsTemperatureTraits<ValueType, TemperatureType>::celsiusOffset = celsiusOffset_; \
\
typedef SpecificAbsTemperature<double, \
                  SpecificAbsTemperatureTraits<double, d##TemperatureType> > dAbs##TemperatureType; \
typedef SpecificAbsTemperature<float, \
                  SpecificAbsTemperatureTraits<float, f##TemperatureType> > fAbs##TemperatureType; \
typedef SpecificAbsTemperature<UNITS_DEFAULT_VALUE_TYPE, \
                  SpecificAbsTemperatureTraits<UNITS_DEFAULT_VALUE_TYPE, TemperatureType> > Abs##TemperatureType


UNITS_DECLARE_SPECIFIC_TEMPERATURE(CelsiusTemperature,    0);
UNITS_DECLARE_SPECIFIC_TEMPERATURE(KelvinTemperature,     -273.15);
UNITS_DECLARE_SPECIFIC_TEMPERATURE(FahrenheitTemperature, -32*5/9.0);
UNITS_DECLARE_SPECIFIC_TEMPERATURE(RankineTemperature,    -(459.67 + 32)*5/9.0);


} // namespace Units


#include "Temperature.i"

#endif  // Units_Temperature_h
