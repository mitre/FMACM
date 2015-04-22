//
// $Id: ProductUnitFormat.i,v 1.4.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Inline implementations for the ProductUnitFormat class.
//

#ifndef Units_ProductUnitFormat_i
#define Units_ProductUnitFormat_i


namespace Units
{


//
// ProductUnitFormat inlines
//

template<typename UnitA,
         typename UnitB>
inline
typename UnitA::ValueType ProductUnitFormat<UnitA, UnitB>::value() const
{
  return _unit.value();
}

#ifndef UNITS_NO_IOSTREAM_OPERATORS
template<class UnitA, class UnitB>
inline
std::ostream & operator<< (std::ostream                          & stream,
                           ProductUnitFormat<UnitA, UnitB> const & format)
{
  return stream << format.value() << ' '
		<< UnitA::unitsString() << '-'
                << UnitB::unitsString();
}
#endif


} // namespace Units


#endif  // Units_ProductUnitFormat_i
