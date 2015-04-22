//
// $Id: Format.i,v 1.1.2.2 2005-12-22 06:15:56 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Inline implementation of the Format classes.
//

#ifndef Units_Format_i
#define Units_Format_i

#ifndef UNITS_NO_IOSTREAM_OPERATORS
#include <sstream>
#endif


namespace Units
{


#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <typename SpecificUnit,
          int      exponent>
inline
bool UnitsString<SpecificUnit,
                 exponent>::print(bool   const   havePrinted,
                                  std::ostream & stream)
{
  if (exponent < 0)
  {
    return havePrinted;
  }
  if (havePrinted)
  {
    stream << '-';
  }
  stream << SpecificUnit::unitsString() << '^' << exponent;
  return true;
}

template <typename SpecificUnit>
inline
bool UnitsString<SpecificUnit,
                 0>::print(bool   const   havePrinted,
                           std::ostream & stream)
{
  return havePrinted;
}

template <typename SpecificUnit>
inline
bool UnitsString<SpecificUnit,
                 1>::print(bool   const   havePrinted,
                           std::ostream & stream)
{
  if (havePrinted)
  {
    stream << '-';
  }
  stream << SpecificUnit::unitsString();
  return true;
}


template <UNITS_UNIT_TEMPLATE_DECL_ARGS,
          UNITS_FORMAT_TEMPLATE_DECL_ARGS>
inline
void UnitsStringFormatter<UNITS_UNIT_TEMPLATE_ARGS,
                          UNITS_FORMAT_TEMPLATE_ARGS>::
print(std::ostream & stream)
{
  bool havePrinted = false;
  havePrinted = UnitsString<SpecificUnitMass,
                            massExp_>       ::print(havePrinted,
                                                    stream);
  havePrinted = UnitsString<SpecificUnitLength,
                            lengthExp_>     ::print(havePrinted,
                                                    stream);
  havePrinted = UnitsString<SpecificUnitTime,
                            timeExp_>       ::print(havePrinted,
                                                    stream);
  havePrinted = UnitsString<SpecificUnitCurrent,
                            currentExp_>    ::print(havePrinted,
                                                    stream);
  havePrinted = UnitsString<SpecificUnitTemperature,
                            temperatureExp_>::print(havePrinted,
                                                    stream);
  havePrinted = UnitsString<SpecificUnitAmount,
                            amountExp_>     ::print(havePrinted,
                                                    stream);
  havePrinted = UnitsString<SpecificUnitIntensity,
                            intensityExp_>  ::print(havePrinted,
                                                    stream);
  havePrinted = UnitsString<SpecificUnitAngle,
                            angleExp_>      ::print(havePrinted,
                                                    stream);
  if (!havePrinted)
  {
    stream << '1';
  }
  if ((massExp_ >= 0) &&
      (lengthExp_ >= 0) &&
      (timeExp_ >= 0) &&
      (currentExp_ >= 0) &&
      (temperatureExp_ >= 0) &&
      (amountExp_ >= 0) &&
      (intensityExp_ >= 0) &&
      (angleExp_ >= 0))
  {
    return;
  }
  stream << '/';
  havePrinted = false;
  havePrinted = UnitsString<SpecificUnitMass,
                            -massExp_>       ::print(havePrinted,
                                                     stream);
  havePrinted = UnitsString<SpecificUnitLength,
                            -lengthExp_>     ::print(havePrinted,
                                                     stream);
  havePrinted = UnitsString<SpecificUnitTime,
                            -timeExp_>       ::print(havePrinted,
                                                     stream);
  havePrinted = UnitsString<SpecificUnitCurrent,
                            -currentExp_>    ::print(havePrinted,
                                                     stream);
  havePrinted = UnitsString<SpecificUnitTemperature,
                            -temperatureExp_>::print(havePrinted,
                                                     stream);
  havePrinted = UnitsString<SpecificUnitAmount,
                            -amountExp_>     ::print(havePrinted,
                                                     stream);
  havePrinted = UnitsString<SpecificUnitIntensity,
                            -intensityExp_>  ::print(havePrinted,
                                                     stream);
  havePrinted = UnitsString<SpecificUnitAngle,
                            -angleExp_>      ::print(havePrinted,
                                                     stream);
}
#endif  // UNITS_NO_IOSTREAM_OPERATORS


template <typename SpecificUnit,
          int      exponent>
const Unit<UNITS_POWER_TEMPLATE_ARGS>
PositivePowerComputer<SpecificUnit, exponent>::value =
SpecificUnit(1.0)*PositivePowerComputer<SpecificUnit, exponent - 1>::value;


template <typename SpecificUnit>
const Unit<typename SpecificUnit::ValueType,
           0, 0, 0, 0, 0, 0, 0, 0>
PositivePowerComputer<SpecificUnit, 0>::value = 1.0;


template <typename SpecificUnit,
          int      exponent>
const Unit<UNITS_POWER_TEMPLATE_ARGS>
PowerComputer<SpecificUnit, exponent, true>::value =
PositivePowerComputer<SpecificUnit, exponent>::value;

template <typename SpecificUnit,
          int      exponent>
const Unit<UNITS_POWER_TEMPLATE_ARGS>
PowerComputer<SpecificUnit, exponent, false>::value =
(1.0/PositivePowerComputer<SpecificUnit, -exponent>::value);

template <typename SpecificUnit,
          int      exponent>
const Unit<UNITS_POWER_TEMPLATE_ARGS> IntPower<SpecificUnit,
                                               exponent>::value =
PowerComputer<SpecificUnit, exponent, (exponent > 0)>::value;


template <UNITS_UNIT_TEMPLATE_DECL_ARGS,
          UNITS_FORMAT_TEMPLATE_DECL_ARGS>
const Unit<UNITS_UNIT_TEMPLATE_ARGS>
ConversionFactor<UNITS_UNIT_TEMPLATE_ARGS,
                 UNITS_FORMAT_TEMPLATE_ARGS>::value =
IntPower<SpecificUnitMass,        massExp_>       ::value*
IntPower<SpecificUnitLength,      lengthExp_>     ::value*
IntPower<SpecificUnitTime,        timeExp_>       ::value*
IntPower<SpecificUnitCurrent,     currentExp_>    ::value*
IntPower<SpecificUnitTemperature, temperatureExp_>::value*
IntPower<SpecificUnitAmount,      amountExp_>     ::value*
IntPower<SpecificUnitIntensity,   intensityExp_>  ::value*
IntPower<SpecificUnitAngle,       angleExp_>      ::value;



template <UNITS_FORMAT_TEMPLATE_DECL_ARGS>
template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Format<UNITS_FORMAT_TEMPLATE_ARGS>::
Format(Unit<UNITS_UNIT_TEMPLATE_ARGS> const & unit):
  _value(unit/ConversionFactor<UNITS_UNIT_TEMPLATE_ARGS,
         UNITS_FORMAT_TEMPLATE_ARGS>::value)
{
#ifndef UNITS_NO_IOSTREAM_OPERATORS
  std::ostringstream unitsStringStream;
  UnitsStringFormatter<UNITS_UNIT_TEMPLATE_ARGS,
    UNITS_FORMAT_TEMPLATE_ARGS>::print(unitsStringStream);
  _unitsString = unitsStringStream.str();
#endif
}

template <UNITS_FORMAT_TEMPLATE_DECL_ARGS>
inline
ValueType Format<UNITS_FORMAT_TEMPLATE_ARGS>::value() const
{
  return _value;
}

#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <UNITS_FORMAT_TEMPLATE_DECL_ARGS>
inline
std::string const & Format<UNITS_FORMAT_TEMPLATE_ARGS>::unitsString() const
{
  return _unitsString;
}


template <UNITS_FORMAT_TEMPLATE_DECL_ARGS>
inline
std::ostream & operator<< (std::ostream                             & stream,
                           Format<UNITS_FORMAT_TEMPLATE_ARGS> const & format)
{
  return stream << format.value() << ' ' << format.unitsString();
}
#endif


} // namespace Units


#endif  // Units_Format_i
