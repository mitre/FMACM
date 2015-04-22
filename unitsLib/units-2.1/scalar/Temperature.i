//
// $Id: Temperature.i,v 1.4.4.2 2009-05-14 02:53:55 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Inline implementations for the Temperature class.
//

#ifndef Units_Temperature_i
#define Units_Temperature_i


namespace Units
{


template <typename ValueType_>
inline
AbsTemperature_<ValueType_>::AbsTemperature_()
{
}

template <typename ValueType_>
inline
AbsTemperature_<ValueType_>::AbsTemperature_(ValueType_ const value):
  _value(value)
{
}


// Comparison operators

template <typename ValueType_>
inline
bool AbsTemperature_<ValueType_>::
similarTo(AbsTemperature_ const & b,
          Temperature     const & tol) const
{
  return (*this - b).similarTo(Temperature(zero()),
                               tol);
}

template <typename ValueType_>
bool similar (AbsTemperature_<ValueType_> const & a,
              AbsTemperature_<ValueType_> const & b,
              Temperature                 const & tol)
{
  return a.similarTo(b, tol);
}

template <typename ValueType_>
inline
bool AbsTemperature_<ValueType_>::operator== (AbsTemperature_ const & unit) const
{
  return (_value == unit._value);
}

template <typename ValueType_>
inline
bool AbsTemperature_<ValueType_>::operator!= (AbsTemperature_ const & unit) const
{
  return (_value != unit._value);
}

template <typename ValueType_>
inline
bool AbsTemperature_<ValueType_>::operator< (AbsTemperature_ const & unit) const
{
  return (_value < unit._value);
}

template <typename ValueType_>
inline
bool AbsTemperature_<ValueType_>::operator> (AbsTemperature_ const & unit) const
{
  return (_value > unit._value);
}

template <typename ValueType_>
inline
bool AbsTemperature_<ValueType_>::operator<= (AbsTemperature_ const & unit) const
{
  return (_value <= unit._value);
}

template <typename ValueType_>
inline
bool AbsTemperature_<ValueType_>::operator>= (AbsTemperature_ const & unit) const
{
  return (_value >= unit._value);
}



// Arithmetic operators

template <typename ValueType_>
inline
AbsTemperature_<ValueType_> AbsTemperature_<ValueType_>::
operator+ (Temperature const & temperature) const
{
  return AbsTemperature_(_value + CelsiusTemperature(temperature).value());
}

template <typename ValueType_>
inline
AbsTemperature_<ValueType_> AbsTemperature_<ValueType_>::
operator- (Temperature const & temperature) const
{
  return AbsTemperature_(_value - CelsiusTemperature(temperature).value());
}

template <typename ValueType_>
inline
Temperature AbsTemperature_<ValueType_>::
operator- (AbsTemperature_ const & temperature) const
{
  return (CelsiusTemperature(_value) -
          CelsiusTemperature(temperature._value));
}


template <typename ValueType_>
inline
AbsTemperature_<ValueType_> &
AbsTemperature_<ValueType_>::operator+=(Temperature const & temperature)
{
  _value += temperature._value;
  return *this;
}

template <typename ValueType_>
inline
AbsTemperature_<ValueType_> &
AbsTemperature_<ValueType_>::operator-=(Temperature const & temperature)
{
  _value -= temperature._value;
  return *this;
}



template <typename ValueType,
          typename SpecificTemperatureTraits>
inline
SpecificAbsTemperature<ValueType,
                       SpecificTemperatureTraits>::
SpecificAbsTemperature()
{
}

template <typename ValueType,
          typename SpecificTemperatureTraits>
inline
SpecificAbsTemperature<ValueType,
                       SpecificTemperatureTraits>::
SpecificAbsTemperature(ValueType const value):
  AbsTemperature_<ValueType>(value*SpecificTemperatureTraits::
                             RelativeTemperature::Traits::internalsPerUnit +
                             SpecificTemperatureTraits::celsiusOffset)
{
}

template <typename ValueType,
          typename SpecificTemperatureTraits>
inline
SpecificAbsTemperature<ValueType,
                       SpecificTemperatureTraits>::
SpecificAbsTemperature(AbsTemperature_<ValueType> const & temperature):
  AbsTemperature_<ValueType>(temperature)
{
}


template <typename ValueType,
          typename SpecificTemperatureTraits>
inline
char const * const &
SpecificAbsTemperature<ValueType,
                       SpecificTemperatureTraits>::unitsString()
{
  return SpecificTemperatureTraits::RelativeTemperature::unitsString();
}


template <typename ValueType,
          typename SpecificTemperatureTraits>
inline
ValueType SpecificAbsTemperature<ValueType,
                                 SpecificTemperatureTraits>::value() const
{
  return ((BaseUnitType::_value -
           SpecificTemperatureTraits::celsiusOffset)/
          SpecificTemperatureTraits::RelativeTemperature::
          Traits::internalsPerUnit);
}


#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <typename ValueType,
          typename SpecificTemperatureTraits>
inline
std::ostream &
operator<< (std::ostream                                            & stream,
            SpecificAbsTemperature<ValueType,
                                   SpecificTemperatureTraits> const & temperature)
{
  return stream << temperature.value() << ' ' << temperature.unitsString();
}
#endif


} // namespace Units


#endif  // Units_Temperature_i
