//
// $Id: Format.h,v 1.1.2.2 2005-12-22 06:15:56 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Provide generic printing of any Unit<> class.
//

#ifndef Units_Format_h
#define Units_Format_h

#ifndef UNITS_NO_IOSTREAM_OPERATORS
#include <iostream>
#include <string>
#endif
// These are only needed for the default Format typedefs.
#include "Time.h"
#include "Mass.h"
#include "Length.h"
#include "Current.h"
#include "Temperature.h"
#include "Angle.h"


namespace Units
{


#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <typename SpecificUnit,
          int      exponent>
struct UnitsString
{
  static bool print(bool   const   havePrinted,
                    std::ostream & stream);
};

template <typename SpecificUnit>
struct UnitsString<SpecificUnit, 0>
{
  static bool print(bool   const   havePrinted,
                    std::ostream & stream);
};

template <typename SpecificUnit>
struct UnitsString<SpecificUnit, 1>
{
  static bool print(bool   const   havePrinted,
                    std::ostream & stream);
};
#endif  // UNITS_NO_IOSTREAM_OPERATORS


#define UNITS_FORMAT_TEMPLATE_DECL_ARGS \
          typename ValueType, \
          typename SpecificUnitMass, \
          typename SpecificUnitLength, \
          typename SpecificUnitTime, \
          typename SpecificUnitCurrent, \
          typename SpecificUnitTemperature, \
          typename SpecificUnitAmount, \
          typename SpecificUnitIntensity, \
          typename SpecificUnitAngle

#define UNITS_FORMAT_TEMPLATE_ARGS \
          ValueType, \
          SpecificUnitMass, \
          SpecificUnitLength, \
          SpecificUnitTime, \
          SpecificUnitCurrent, \
          SpecificUnitTemperature, \
          SpecificUnitAmount, \
          SpecificUnitIntensity, \
          SpecificUnitAngle


#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <UNITS_UNIT_TEMPLATE_DECL_ARGS,
          UNITS_FORMAT_TEMPLATE_DECL_ARGS>
struct UnitsStringFormatter
{
  static void print(std::ostream & stream);
};
#endif


#define UNITS_POWER_TEMPLATE_ARGS \
                    typename SpecificUnit::ValueType, \
                    SpecificUnit::massExp        * exponent, \
                    SpecificUnit::lengthExp      * exponent, \
                    SpecificUnit::timeExp        * exponent, \
                    SpecificUnit::currentExp     * exponent, \
                    SpecificUnit::temperatureExp * exponent, \
                    SpecificUnit::amountExp      * exponent, \
                    SpecificUnit::intensityExp   * exponent, \
                    SpecificUnit::angleExp       * exponent

///
/// Raise a unit type to the given power.
///
template <typename SpecificUnit,
          int      exponent>
struct PositivePowerComputer
{
  static const Unit<UNITS_POWER_TEMPLATE_ARGS> value;
};

template <typename SpecificUnit>
struct PositivePowerComputer<SpecificUnit, 0>
{
  static const Unit<typename SpecificUnit::ValueType,
                    0, 0, 0, 0, 0, 0, 0, 0> value;
};


template <typename SpecificUnit,
          int      exponent,
          bool     exponentIsPositive>
struct PowerComputer
{
  static const Unit<UNITS_POWER_TEMPLATE_ARGS> value;
};

template <typename SpecificUnit,
          int      exponent>
struct PowerComputer<SpecificUnit, exponent, true>
{
  static const Unit<UNITS_POWER_TEMPLATE_ARGS> value;
};

template <typename SpecificUnit,
          int      exponent>
struct PowerComputer<SpecificUnit, exponent, false>
{
  static const Unit<UNITS_POWER_TEMPLATE_ARGS> value;
};


template <typename SpecificUnit,
          int      exponent>
struct IntPower
{
  static const Unit<UNITS_POWER_TEMPLATE_ARGS> value;
};


template <UNITS_UNIT_TEMPLATE_DECL_ARGS,
          UNITS_FORMAT_TEMPLATE_DECL_ARGS>
struct ConversionFactor
{
  static const Unit<UNITS_UNIT_TEMPLATE_ARGS> value;
};


template <UNITS_FORMAT_TEMPLATE_DECL_ARGS>
class Format
{
public:
  template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
  Format(Unit<UNITS_UNIT_TEMPLATE_ARGS> const & unit);

  ValueType value() const;

#ifndef UNITS_NO_IOSTREAM_OPERATORS
  std::string const & unitsString() const;
#endif

private:
  ValueType   _value;

#ifndef UNITS_NO_IOSTREAM_OPERATORS
  std::string _unitsString;
#endif
};



#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <UNITS_FORMAT_TEMPLATE_DECL_ARGS>
std::ostream & operator<< (std::ostream                             & stream,
                           Format<UNITS_FORMAT_TEMPLATE_ARGS> const & format);
#endif


struct UnspecifiedUnit
{
  typedef UNITS_DEFAULT_VALUE_TYPE ValueType;
  static const int massExp        = 0;
  static const int lengthExp      = 0;
  static const int timeExp        = 0;
  static const int currentExp     = 0;
  static const int temperatureExp = 0;
  static const int amountExp      = 0;
  static const int intensityExp   = 0;
  static const int angleExp       = 0;
};

typedef Format<UNITS_DEFAULT_VALUE_TYPE,
               KilogramsMass,
               MetersLength,
               SecondsTime,
               AmpsCurrent,
               KelvinTemperature,
               UnspecifiedUnit,
               UnspecifiedUnit,
               DegreesAngle> MksFormat;

typedef Format<UNITS_DEFAULT_VALUE_TYPE,
               GramsMass,
               CentimetersLength,
               SecondsTime,
               AmpsCurrent,
               KelvinTemperature,
               UnspecifiedUnit,
               UnspecifiedUnit,
               DegreesAngle> CgsFormat;


} // namespace Units


#include "Format.i"

#endif  // Units_Format_h
