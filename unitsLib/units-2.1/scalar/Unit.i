//
// $Id: Unit.i,v 1.3.4.6 2009-05-14 02:53:55 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Inline implementations for the Unit class.
//

#ifndef Units_Unit_i
#define Units_Unit_i

#include <math.h>



namespace Units
{


inline
Zero zero()
{
  return Zero();
}
inline
Infinity infinity()
{
  return Infinity();
}
inline
NegInfinity negInfinity()
{
  return NegInfinity();
}

inline
NegInfinity Infinity::operator- () const
{
  return negInfinity();
}

inline
Infinity NegInfinity::operator- () const
{
  return infinity();
}


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS> const
Unit<UNITS_UNIT_TEMPLATE_ARGS>::_defaultTolerance(1e-6);


// Constructors

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>::Unit()
{
}


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>::Unit(Zero const &):
  _value(0)
{
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>::Unit(Infinity const &):
  _value(HUGE_VAL)
{
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>::Unit(NegInfinity const &):
  _value(-HUGE_VAL)
{
}


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>::Unit(ValueType const value):
  _value(value)
{
}


// Comparison operators

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
bool Unit<UNITS_UNIT_TEMPLATE_ARGS>::similarTo(Unit const & b,
                                               Unit const & tol) const
{
  return (abs(*this - b)._value < tol._value);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
bool similar(Unit<UNITS_UNIT_TEMPLATE_ARGS> const & a,
             Unit<UNITS_UNIT_TEMPLATE_ARGS> const & b,
             Unit<UNITS_UNIT_TEMPLATE_ARGS> const & tol)
{
  return a.similarTo(b, tol);
}

#ifdef _MSC_VER
template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
bool similar(Unit<UNITS_UNIT_TEMPLATE_ARGS> const & a,
             Unit<UNITS_UNIT_TEMPLATE_ARGS> const & b)
{
  return a.similarTo(b, Unit<UNITS_UNIT_TEMPLATE_ARGS>::defaultTolerance());
}
#endif


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
bool Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator==(Unit const & unit) const
{
  return (_value == unit._value);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
bool Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator!=(Unit const & unit) const
{
  return (_value != unit._value);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
bool Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator< (Unit const & unit) const
{
  return (_value <  unit._value);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
bool Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator> (Unit const & unit) const
{
  return (_value >  unit._value);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
bool Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator<=(Unit const & unit) const
{
  return (_value <= unit._value);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
bool Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator>=(Unit const & unit) const
{
  return (_value >= unit._value);
}


// Arithmetic operators

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator* (ValueType_ const scale) const
{
  return Unit(_value*scale);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>
operator* (ValueType_                     const   scale,
           Unit<UNITS_UNIT_TEMPLATE_ARGS> const & unit)
{
  return unit*scale;
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator/ (ValueType_ const scale) const
{
  return Unit(_value/scale);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS> & Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator*=(ValueType_ const scale)
{
  return *this = *this*scale;
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS> & Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator/=(ValueType_ const scale)
{
  return *this = *this/scale;
}


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator- () const
{
  return Unit(-_value);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator+ (Unit const & unit) const
{
  return Unit(_value + unit._value);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator- (Unit const & unit) const
{
  return Unit(_value - unit._value);
}


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS> &
Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator+= (Unit const & unit)
{
  return *this = *this + unit;
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS> &
Unit<UNITS_UNIT_TEMPLATE_ARGS>::operator-= (Unit const & unit)
{
  return *this = *this - unit;
}


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS> const &
Unit<UNITS_UNIT_TEMPLATE_ARGS>::defaultTolerance()
{
  return _defaultTolerance;
}


template <UNITS_UNIT_TEMPLATE_DECL_ARGS2>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS_MUL_RESULT>
operator* (Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & lhs,
           Unit<UNITS_UNIT_TEMPLATE_ARGS2> const & rhs)
{
  return Unit<UNITS_UNIT_TEMPLATE_ARGS_MUL_RESULT>(lhs._value *
                                                   rhs._value);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS2>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS_DIV_RESULT>
operator/ (Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & lhs,
           Unit<UNITS_UNIT_TEMPLATE_ARGS2> const & rhs)
{
  return Unit<UNITS_UNIT_TEMPLATE_ARGS_DIV_RESULT>(lhs._value /
                                                   rhs._value);
}

template <typename ValueType1,
          UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(1)>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS_INV_RESULT>
operator/ (ValueType1 const lhs,
           Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & rhs)
{
  return Unit<UNITS_UNIT_TEMPLATE_ARGS_INV_RESULT>(lhs/rhs._value);
}


template <typename ValueType>
ValueType sqrtComputer(ValueType x);

template <>
inline
double sqrtComputer(double x)
{
  return ::sqrt(x);
}

template <>
inline
float sqrtComputer(float x)
{
  return ::sqrtf(x);
}

template <typename ValueType1,
          UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(1)>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS_SQRT_RESULT>
sqrt(Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & value)
{
  return Unit<UNITS_UNIT_TEMPLATE_ARGS_SQRT_RESULT>(sqrtComputer(value._value));
}


template <typename ValueType>
ValueType absComputer(ValueType x);

template <>
inline
double absComputer(double x)
{
  return ::fabs(x);
}

template <>
inline
float absComputer(float x)
{
  return ::fabsf(x);
}

template <typename ValueType1,
          UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(1)>
inline
Unit<UNITS_UNIT_TEMPLATE_ARGS1>
abs(Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & value)
{
  return Unit<UNITS_UNIT_TEMPLATE_ARGS1>(absComputer(value._value));
}


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS>
min(Unit<UNITS_UNIT_TEMPLATE_ARGS> const & a,
    Unit<UNITS_UNIT_TEMPLATE_ARGS> const & b)
{
  return ((a < b) ? a : b);
}

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS>
max(Unit<UNITS_UNIT_TEMPLATE_ARGS> const & a,
    Unit<UNITS_UNIT_TEMPLATE_ARGS> const & b)
{
  return ((a > b) ? a : b);
}


template <typename ValueType>
inline
Unit<ValueType, 0, 0, 0, 0, 0, 0, 0, 0>::Unit()
{
}

template <typename ValueType>
inline
Unit<ValueType, 0, 0, 0, 0, 0, 0, 0, 0>::Unit(ValueType const value):
  _value(value)
{
}

template <typename ValueType>
inline
Unit<ValueType, 0, 0, 0, 0, 0, 0, 0, 0>::operator ValueType() const
{
  return _value;
}

template <typename ValueType>
inline
Unit<ValueType, 0, 0, 0, 0, 0, 0, 0, 0> &
Unit<ValueType, 0, 0, 0, 0, 0, 0, 0, 0>::operator= (ValueType const value)
{
  _value = value;
  return *this;
}


} // namespace Units


#endif  // Units_Unit_i
