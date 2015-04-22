//
// $Id: Area.h,v 1.6.4.2 2005-12-22 06:15:56 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of area.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Area_h
#define Units_Area_h

#include "SpecificUnit.h"
#include "Format.h"
#include "Length.h"


namespace Units
{


UNITS_DECLARE_BASE_UNIT(Area, 0, 2, 0, 0, 0, 0, 0, 0);


template <typename LengthUnit>
class AreaFormat: public Format<typename LengthUnit::ValueType,
                                UnspecifiedUnit,
                                LengthUnit,
                                UnspecifiedUnit,
                                UnspecifiedUnit,
                                UnspecifiedUnit,
                                UnspecifiedUnit,
                                UnspecifiedUnit,
                                UnspecifiedUnit>
{
public:
  AreaFormat(Area const & area):
    Format<typename LengthUnit::ValueType,
           UnspecifiedUnit,
           LengthUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit>(area)
  {
  }
};



#define UNITS_AREA_DECLARE_LENGTH_SUBCLASS(LengthName, LengthUnitString) \
UNITS_DECLARE_SPECIFIC_UNIT(Area, LengthName##Area, LengthUnitString"^2", \
 (SpecificUnitTraits<LengthName##LengthTraits, double>::internalsPerUnit* \
  SpecificUnitTraits<LengthName##LengthTraits, double>::internalsPerUnit))

UNITS_AREA_DECLARE_LENGTH_SUBCLASS(Inches,        "in");
UNITS_AREA_DECLARE_LENGTH_SUBCLASS(Feet,          "ft");
UNITS_AREA_DECLARE_LENGTH_SUBCLASS(Meters,        "m");
UNITS_AREA_DECLARE_LENGTH_SUBCLASS(Kilometers,    "km");
UNITS_AREA_DECLARE_LENGTH_SUBCLASS(NauticalMiles, "nmi");
UNITS_AREA_DECLARE_LENGTH_SUBCLASS(StatuteMiles,  "mi");

//
// 1 acre = 43560 square ft
//
UNITS_DECLARE_SPECIFIC_UNIT(Area, AcresArea, "acres",  43560.0/10.76391042);


} // namespace Units


#endif  // Units_Area_h
