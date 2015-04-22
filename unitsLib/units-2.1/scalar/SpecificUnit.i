//
// $Id: SpecificUnit.i,v 1.6.2.2 2005-12-22 04:31:39 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Inline implementations for the SpecificUnit class.
//

#ifndef Units_SpecificUnit_i
#define Units_SpecificUnit_i



namespace Units
{


template <typename ValueType,
          typename SpecificUnitTraits>
inline
SpecificUnit<ValueType,
             SpecificUnitTraits>::SpecificUnit()
{
}

template <typename ValueType,
          typename SpecificUnitTraits>
inline
SpecificUnit<ValueType,
             SpecificUnitTraits>::SpecificUnit(ValueType const value):
  BaseUnitType(value*SpecificUnitTraits::internalsPerUnit)
{}

template <typename ValueType,
          typename SpecificUnitTraits>
inline
SpecificUnit<ValueType,
             SpecificUnitTraits>::SpecificUnit(BaseUnitType const & unit):
  BaseUnitType(unit)
{}


template <typename ValueType,
          typename SpecificUnitTraits>
inline
char const * const & SpecificUnit<ValueType,
                                  SpecificUnitTraits>::unitsString()
{
  return SpecificUnitTraits::unitsString;
}


template <typename ValueType,
          typename SpecificUnitTraits>
inline
ValueType SpecificUnit<ValueType,
                       SpecificUnitTraits>::value() const
{
  return BaseUnitType::_value/SpecificUnitTraits::internalsPerUnit;
}


#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <typename ValueType,
          typename SpecificUnitTraits>
std::ostream & operator<< (std::ostream                           & stream,
                           SpecificUnit<ValueType,
                                        SpecificUnitTraits> const & units)
{
  return stream << units.value() << ' ' << units.unitsString();
}
#endif


} // namespace Units


#endif  // Units_SpecificUnit_i
