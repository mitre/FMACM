//
// $Id: RatioUnitFormat.h,v 1.4.4.2 2005-12-16 06:22:31 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Template class for converting or formating a ratio of two units of
// measure.
//

#ifndef Units_RatioUnitFormat_h
#define Units_RatioUnitFormat_h

#ifndef UNITS_NO_IOSTREAM_OPERATORS
#include <iostream>
#endif


namespace Units
{

template <typename UnitNum,
          typename UnitDen>
class RatioUnitFormat
{
public:
  template <typename RatioUnit>
  RatioUnitFormat(RatioUnit const & ratio):
    _unit(ratio*UnitDen(1))
  //
  // MSVC++ 6.0 won't let me put this in the inline file!
  //
  {
  }

  typename UnitNum::ValueType value() const;

private:
  UnitNum _unit;
};


#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <typename UnitNum,
          typename UnitDen>
std::ostream & operator<< (std::ostream                            & stream,
                           RatioUnitFormat<UnitNum, UnitDen> const & format);
#endif


} // namespace Units


#include "RatioUnitFormat.i"

#endif  // Units_RatioUnitFormat_h
