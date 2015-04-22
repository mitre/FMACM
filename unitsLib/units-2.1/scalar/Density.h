//
// $Id: Density.h,v 1.5.4.2 2005-12-22 06:15:56 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of density.
// The internal representation of the units is hidden from the user.
//
// NOTE: A Density can only be given a value indirectly; i.e. to
//       specify 1 kg/m^3, do
//          Density density = KilogramsMass(1)/MetersVolume(1)
//

#ifndef Units_Density_h
#define Units_Density_h

#include "SpecificUnit.h"
#include "Format.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Density, 1, -3, 0, 0, 0, 0, 0, 0);

UNITS_DECLARE_SPECIFIC_UNIT(Density, KilogramsMeterDensity, "kg/m^3", 1);


template <typename MassUnit,
          typename LengthUnit>
class DensityFormat: public Format<typename MassUnit::ValueType,
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
  DensityFormat(Density const & density):
    Format<typename MassUnit::ValueType,
           MassUnit,
           LengthUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit>(density)
  {
  }
};


} // namespace Units


#endif  // Units_Density_h
