//
// $Id: Volume.h,v 1.6.4.2 2005-12-22 06:15:56 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of a measure of volume.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Volume_h
#define Units_Volume_h

#include "SpecificUnit.h"
#include "Format.h"
#include "Length.h"



namespace Units
{


UNITS_DECLARE_BASE_UNIT(Volume, 0, 3, 0, 0, 0, 0, 0, 0);


template <class LengthUnit>
class VolumeFormat: public Format<typename LengthUnit::ValueType,
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
  VolumeFormat(Volume const & volume):
    Format<typename LengthUnit::ValueType,
           UnspecifiedUnit,
           LengthUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit,
           UnspecifiedUnit>(volume)
  {
  }
};


#define UNITS_VOLUME_DECLARE_LENGTH_SUBCLASS(LengthName, LengthUnitString) \
UNITS_DECLARE_SPECIFIC_UNIT(Volume, LengthName##Volume, LengthUnitString"^3", \
 (SpecificUnitTraits<LengthName##LengthTraits, double>::internalsPerUnit* \
  SpecificUnitTraits<LengthName##LengthTraits, double>::internalsPerUnit* \
  SpecificUnitTraits<LengthName##LengthTraits, double>::internalsPerUnit))

UNITS_VOLUME_DECLARE_LENGTH_SUBCLASS(Inches, "in");
UNITS_VOLUME_DECLARE_LENGTH_SUBCLASS(Feet,   "ft");
UNITS_VOLUME_DECLARE_LENGTH_SUBCLASS(Meters, "m");

//
// 1 US gal      = 231         cubic inches
// 1 BI gal      = 277.4       cubic inches
//
UNITS_DECLARE_SPECIFIC_UNIT(Volume, LitersVolume,    "L",      1e-3);
// United States Gallon
UNITS_DECLARE_SPECIFIC_UNIT(Volume, USGallonsVolume, "US gal", (231.0/
                                                                61023.74409));
// British Imperial Gallon
UNITS_DECLARE_SPECIFIC_UNIT(Volume, BIGallonsVolume, "BI gal", (277.4/
                                                                61023.74409));


} // namespace Units


#endif  // Units_Volume_h
