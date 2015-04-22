//
// $Id: Inertia.h,v 1.4.4.2 2005-12-22 06:15:56 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of inertia.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Inertia_h
#define Units_Inertia_h

#include "SpecificUnit.h"
#include "Format.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Inertia, 1, 2, 0, 0, 0, 0, 0, 0);

UNITS_DECLARE_SPECIFIC_UNIT(Inertia, KilogramsMeterInertia, "kg-m^2", 1);

template <typename MassUnit,
          typename LengthUnit>
class InertiaFormat: public Format<typename MassUnit::ValueType,
                                   MassUnit,
                                   LengthUnit,
                                   UnspecifiedUnit,
                                   UnspecifiedUnit,
                                   UnspecifiedUnit,
                                   UnspecifiedUnit,
                                   UnspecifiedUnit,
                                   UnspecifiedUnit>
{
public:
  InertiaFormat(Inertia const & inertia):
    Format<typename MassUnit::ValueType,
           MassUnit,
           LengthUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit>(inertia)
  {
  }
};


} // namespace Units


#endif  // Units_Inertia_h
