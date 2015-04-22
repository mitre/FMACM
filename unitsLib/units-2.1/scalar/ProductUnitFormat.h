//
// $Id: ProductUnitFormat.h,v 1.4.4.1 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Template class for converting or formating a product of two units of
// measure.
//

#ifndef Units_ProductUnitFormat_h
#define Units_ProductUnitFormat_h

#ifndef UNITS_NO_IOSTREAM_OPERATORS
#include <iostream>
#endif


namespace Units
{

template<typename UnitA,
         typename UnitB>
class ProductUnitFormat
{
public:
  template<class ProductUnit>
  ProductUnitFormat(ProductUnit const & product):
    _unit(product/UnitB(1))
  //
  // MSVC++ 6.0 won't let me put this in the inline file!
  //
  {
  }

  typename UnitA::ValueType value() const;

private:
  UnitA _unit;
};


#ifndef UNITS_NO_IOSTREAM_OPERATORS
template<typename UnitA,
         typename UnitB>
std::ostream & operator<< (std::ostream                          & stream,
                           ProductUnitFormat<UnitA, UnitB> const & format);
#endif


} // namespace Units


#include "ProductUnitFormat.i"

#endif  // Units_ProductUnitFormat_h
