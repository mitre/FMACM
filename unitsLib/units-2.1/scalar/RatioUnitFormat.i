//
// $Id: RatioUnitFormat.i,v 1.3.4.2 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Inline implementations for the RatioUnitFormat class.
//

#ifndef Units_RatioUnitFormat_i
#define Units_RatioUnitFormat_i


namespace Units
{


//
// RatioUnitFormat inlines
//

template<class UnitNum, class UnitDen>
inline
typename UnitNum::ValueType RatioUnitFormat<UnitNum, UnitDen>::value() const
{
  return _unit.value();
}

#ifndef UNITS_NO_IOSTREAM_OPERATORS
template<class UnitNum, class UnitDen>
inline
std::ostream & operator<< (std::ostream                            & stream,
                           RatioUnitFormat<UnitNum, UnitDen> const & format)
{
  return stream << format.value() << ' '
		<< UnitNum::unitsString() << '/' << UnitDen::unitsString();
}
#endif


} // namespace Units


#endif  // Units_RatioUnitFormat_i
