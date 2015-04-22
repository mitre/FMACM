//
// $Id: SignedAngle.i,v 1.8.4.3 2009-05-14 02:53:55 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Inline implementations for the SignedAngle class.
//

#ifndef Units_SignedAngle_i
#define Units_SignedAngle_i


namespace Units
{


template <typename ValueType>
inline
SignedAngleType<ValueType>::SignedAngleType()
{
}

template <typename ValueType>
inline
SignedAngleType<ValueType>::
SignedAngleType(UNITS_ANGLE_TYPE(ValueType) const & angle):
  UNITS_ANGLE_TYPE(ValueType)(angle)
{
  normalize();
}

template <typename ValueType>
inline
SignedAngleType<ValueType> &
SignedAngleType<ValueType>::
operator+= (UNITS_ANGLE_TYPE(ValueType) const & angle)
{
  return *this = *this + angle;
}

template <typename ValueType>
inline
SignedAngleType<ValueType> &
SignedAngleType<ValueType>::
operator-= (UNITS_ANGLE_TYPE(ValueType) const & angle)
{
  return *this = *this - angle;
}


template <typename ValueType>
inline
SignedAngleType<ValueType> & SignedAngleType<ValueType>::normalize()
{
  // First make sure delta is between -360..360.
  Units::normalize(*this);

  typedef SpecificUnit<ValueType,
                       SpecificUnitTraits<DegreesAngleTraits,
                                          ValueType> >
    SpecificAngleType;

  // Now wrap to -180..180.
  if (*this < SpecificAngleType(-180.0))
  {
    UNITS_ANGLE_TYPE(ValueType)::
      operator+=(SpecificAngleType(360.0));
  }
  else if (*this > SpecificAngleType(180.0))
  {
    UNITS_ANGLE_TYPE(ValueType)::
      operator-=(SpecificAngleType(360.0));
  }
  return *this;
}


template <typename ValueType>
inline
SignedAngleType<ValueType>
SignedAngleType<ValueType>::operator- (SignedAngleType const & angle) const
{
  // Get the mathematical difference.
  SignedAngleType delta = UNITS_ANGLE_TYPE(ValueType)::operator-(angle);

  // Wrap to -180..180.
  delta.normalize();

  return delta;
}


template <typename ValueType>
inline
bool     similar   (SignedAngleType<ValueType>  const & a,
                    SignedAngleType<ValueType>  const & b,
                    UNITS_ANGLE_TYPE(ValueType) const & tol)
{
  return similar(UNITS_ANGLE_TYPE(ValueType)(a - b),
                 UNITS_ANGLE_TYPE(ValueType)(zero()),
                 tol);
}

#if defined (_MSC_VER) && (_MSC_VER < 1300)
template <typename ValueType>
inline
bool     similar   (SignedAngleType<ValueType> const & a,
                    SignedAngleType<ValueType> const & b)
{
  return similar(UNITS_ANGLE_TYPE(ValueType)(a - b),
                 UNITS_ANGLE_TYPE(ValueType)(zero()),
                 UNITS_ANGLE_TYPE(ValueType)::defaultTolerance());
}
#endif


//
// Inline functions for SpecificSignedAngle.
//

template <class SpecificAngleUnits>
inline
SpecificSignedAngle<SpecificAngleUnits>::SpecificSignedAngle()
{
}

template <class SpecificAngleUnits>
inline
SpecificSignedAngle<SpecificAngleUnits>::
SpecificSignedAngle(UNITS_ANGLE_TYPE(ValueType) const & angle):
  SignedAngleType<ValueType>(angle)
{
}

template <class SpecificAngleUnits>
inline
SpecificSignedAngle<SpecificAngleUnits>::
SpecificSignedAngle(ValueType const value):
  SignedAngleType<ValueType>(SpecificAngleUnits(value))
{
}

template <class SpecificAngleUnits>
inline
typename SpecificSignedAngle<SpecificAngleUnits>::ValueType
SpecificSignedAngle<SpecificAngleUnits>::value() const
{
  SpecificSignedAngle temp = *this;
  return SpecificAngleUnits(temp.normalize()).value();
}

#ifndef UNITS_NO_IOSTREAM_OPERATORS
template <class SpecificAngleUnits>
inline
std::ostream &
operator<<(std::ostream                                  & stream,
           SpecificSignedAngle<SpecificAngleUnits> const & angle)
{
  return stream << SpecificAngleUnits(angle);
}
#endif


} // namespace Units


#endif  // Units_SignedAngle_i
