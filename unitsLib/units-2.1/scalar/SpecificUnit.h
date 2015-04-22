//
// $Id: SpecificUnit.h,v 1.7.2.4 2007-09-28 04:54:01 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// This template allows subclassing for a specific unit of measure.
//

#ifndef Units_SpecificUnit_h
#define Units_SpecificUnit_h

#ifndef UNITS_NO_IOSTREAM_OPERATORS
#include <iostream>
#endif

#include "Unit.h"


namespace Units
{


///
/// This is a bit hokey because neither template parameter are used.
/// But the first parameter allows us to create a unique type for each
/// specific unit, and the second allows us to do a partial
/// specialization which prevents the memory for the traits from being
/// allocated unless it's actually being used somewhere.
///
template <typename SpecificUnitType,
          typename ValueType>
struct SpecificUnitTraits
{
  static const char * const unitsString;
  static const double internalsPerUnit;
};

template <typename ValueType,
          typename SpecificUnitTraits>
class SpecificUnit: public SpecificUnitTraits::BaseUnitType
{
public:
  typedef SpecificUnitTraits Traits;
  typedef typename SpecificUnitTraits::BaseUnitType BaseUnitType;

  ///
  /// Default constructor gives uninitialized value!
  ///
  SpecificUnit();

  explicit
  SpecificUnit(ValueType    const   value);

  SpecificUnit(BaseUnitType const & unit);

  ValueType value() const;

  static
  char const * const & unitsString();
};

#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <typename ValueType,
          typename SpecificUnitTraits>
std::ostream & operator<< (std::ostream                           & stream,
                           SpecificUnit<ValueType,
                                        SpecificUnitTraits> const & units);
#endif


//
// Macro to allow declaration of subclasses.
//
// Notes:  No comments are allowed in the macro definition.
//         No blanks can follow the backslashes (\).
//
#define UNITS_DECLARE_SPECIFIC_UNIT(Base, Name, unitsString_, internalsPerUnit_) \
struct Name##Traits \
{ \
}; \
\
template <typename ValueType> \
struct SpecificUnitTraits<Name##Traits, ValueType> \
{ \
  typedef typename Base##Traits<ValueType>::BaseUnitType BaseUnitType; \
  static char const * const unitsString; \
  static double       const internalsPerUnit; \
}; \
\
template <typename ValueType> \
char const * const \
SpecificUnitTraits<Name##Traits, ValueType>::unitsString      = unitsString_; \
\
template <typename ValueType> \
double       const \
SpecificUnitTraits<Name##Traits, ValueType>::internalsPerUnit = \
  internalsPerUnit_; \
\
typedef SpecificUnit<double, \
                     SpecificUnitTraits<Name##Traits, double> > d##Name; \
typedef SpecificUnit<float, \
                     SpecificUnitTraits<Name##Traits, float> > f##Name; \
typedef SpecificUnit<UNITS_DEFAULT_VALUE_TYPE, \
                     SpecificUnitTraits<Name##Traits, \
                                        UNITS_DEFAULT_VALUE_TYPE> > Name


} // namespace Units


#include "SpecificUnit.i"

#endif  // Units_SpecificUnit_h
