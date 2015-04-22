//
// $Id: UnsignedAngle.i,v 1.8.4.3 2009-05-14 02:53:55 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Inline implementations for the UnsignedAngle class.
//

#ifndef Units_UnsignedAngle_i
#define Units_UnsignedAngle_i


namespace Units
{


template <typename ValueType>
inline
UnsignedAngleType<ValueType>::UnsignedAngleType()
{
}

template <typename ValueType>
inline
UnsignedAngleType<ValueType>::
UnsignedAngleType(UNITS_ANGLE_TYPE(ValueType) const & angle):
  UNITS_ANGLE_TYPE(ValueType)(angle)
{
  normalize();
}

template <typename ValueType>
inline
UnsignedAngleType<ValueType> &
UnsignedAngleType<ValueType>::
operator+= (UNITS_ANGLE_TYPE(ValueType) const & angle)
{
  return *this = *this + angle;
}

template <typename ValueType>
inline
UnsignedAngleType<ValueType> &
UnsignedAngleType<ValueType>::
operator-= (UNITS_ANGLE_TYPE(ValueType) const & angle)
{
  return *this = *this - angle;
}


template <typename ValueType>
inline
UnsignedAngleType<ValueType> & UnsignedAngleType<ValueType>::normalize()
{
  // First make sure delta is between -360..360.
  Units::normalize(*this);

  typedef SpecificUnit<ValueType,
                       SpecificUnitTraits<DegreesAngleTraits,
                                          ValueType> >
    SpecificAngleType;

  // Now wrap to 0..360.
  if (*this < UNITS_ANGLE_TYPE(ValueType)(zero()))
  {
    UNITS_ANGLE_TYPE(ValueType)::operator+=(SpecificAngleType(360.0));
  }
  return *this;
}


template <typename ValueType>
inline
SignedAngleType<ValueType>
UnsignedAngleType<ValueType>::operator- (UnsignedAngleType const & angle) const
{
  // Get the mathematical difference.
  UnsignedAngleType delta = UNITS_ANGLE_TYPE(ValueType)::operator-(angle);

  // Wrap to -180..180.
  delta.normalize();

  return delta;
}


template <typename ValueType>
inline
bool     similar   (UnsignedAngleType<ValueType>  const & a,
                    UnsignedAngleType<ValueType>  const & b,
                    UNITS_ANGLE_TYPE(ValueType) const & tol)
{
  return similar(UNITS_ANGLE_TYPE(ValueType)(a - b),
                 UNITS_ANGLE_TYPE(ValueType)(zero()),
                 tol);
}

#if defined (_MSC_VER) && (_MSC_VER < 1300)
template <typename ValueType>
inline
bool     similar   (UnsignedAngleType<ValueType> const & a,
                    UnsignedAngleType<ValueType> const & b)
{
  return similar(UNITS_ANGLE_TYPE(ValueType)(a - b),
                 UNITS_ANGLE_TYPE(ValueType)(zero()),
                 UNITS_ANGLE_TYPE(ValueType)::defaultTolerance());
}
#endif


//
// Inline functions for SpecificUnsignedAngle.
//

template <class SpecificAngleUnits>
inline
SpecificUnsignedAngle<SpecificAngleUnits>::SpecificUnsignedAngle()
{
}

template <class SpecificAngleUnits>
inline
SpecificUnsignedAngle<SpecificAngleUnits>::
SpecificUnsignedAngle(UNITS_ANGLE_TYPE(ValueType) const & angle):
  UnsignedAngleType<ValueType>(angle)
{
}

template <class SpecificAngleUnits>
inline
SpecificUnsignedAngle<SpecificAngleUnits>::
SpecificUnsignedAngle(ValueType const value):
  UnsignedAngleType<ValueType>(SpecificAngleUnits(value))
{
}

template <class SpecificAngleUnits>
inline
typename SpecificUnsignedAngle<SpecificAngleUnits>::ValueType
SpecificUnsignedAngle<SpecificAngleUnits>::value() const
{
  SpecificUnsignedAngle temp = *this;
  return SpecificAngleUnits(temp.normalize()).value();
}

#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <class SpecificAngleUnits>
inline
std::ostream &
operator<<(std::ostream                                    & stream,
           SpecificUnsignedAngle<SpecificAngleUnits> const & angle)
{
  return stream << SpecificAngleUnits(angle);
}
#endif


} // namespace Units


#endif  // Units_UnsignedAngle_i
