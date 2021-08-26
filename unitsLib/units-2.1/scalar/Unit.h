//
// $Id: Unit.h,v 1.11.2.9 2009-05-14 02:53:55 knicewar Exp $
//
// Copyright Keith Nicewarner.  All rights reserved.
//
// Represent the abstract concept of an arbitrary unit of measure.
// The internal representation of the units is hidden from the user.
//

#ifndef Units_Unit_h
#define Units_Unit_h

#include "system/units_config.h"



namespace Units
{

//
// Call me lazy, but typing this for each template gets really old ...
//
#define UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(suffix) int massExp##suffix, \
                                                  int lengthExp##suffix, \
                                                  int timeExp##suffix, \
                                                  int currentExp##suffix, \
                                                  int temperatureExp##suffix, \
                                                  int amountExp##suffix, \
                                                  int intensityExp##suffix, \
                                                  int angleExp##suffix

#define UNITS_UNIT_TEMPLATE_DECL_ARGS typename ValueType_, \
        UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(_)

#define UNITS_UNIT_TEMPLATE_DECL_ARGS2 typename ValueType1, \
        UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(1), \
        UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(2)

#define UNITS_UNIT_TEMPLATE_ARGS ValueType_, \
                                 massExp_, \
                                 lengthExp_, \
                                 timeExp_, \
                                 currentExp_, \
                                 temperatureExp_, \
                                 amountExp_, \
                                 intensityExp_, \
                                 angleExp_

#define UNITS_UNIT_TEMPLATE_ARGS1 \
                  ValueType1, \
                  massExp1, \
                  lengthExp1, \
                  timeExp1, \
                  currentExp1, \
                  temperatureExp1, \
                  amountExp1, \
                  intensityExp1, \
                  angleExp1

#define UNITS_UNIT_TEMPLATE_ARGS2 \
                  ValueType1, \
                  massExp2, \
                  lengthExp2, \
                  timeExp2, \
                  currentExp2, \
                  temperatureExp2, \
                  amountExp2, \
                  intensityExp2, \
                  angleExp2

#define UNITS_UNIT_TEMPLATE_ARGS_SQR_RESULT \
     ValueType_, \
     2*massExp_, \
     2*lengthExp_, \
     2*timeExp_, \
     2*currentExp_, \
     2*temperatureExp_, \
     2*amountExp_, \
     2*intensityExp_, \
     AngleExponent<2*angleExp_, \
                   2*lengthExp_>::value

#define UNITS_UNIT_TEMPLATE_ARGS_MUL_RESULT \
       ValueType1, \
       massExp1        + massExp2, \
       lengthExp1      + lengthExp2, \
       timeExp1        + timeExp2, \
       currentExp1     + currentExp2, \
       temperatureExp1 + temperatureExp2, \
       amountExp1      + amountExp2, \
       intensityExp1   + intensityExp2, \
       AngleExponent<angleExp1  + angleExp2, \
                     lengthExp1 + lengthExp2>::value

#define UNITS_UNIT_TEMPLATE_ARGS_DIV_RESULT \
       ValueType1, \
       massExp1        - massExp2, \
       lengthExp1      - lengthExp2, \
       timeExp1        - timeExp2, \
       currentExp1     - currentExp2, \
       temperatureExp1 - temperatureExp2, \
       amountExp1      - amountExp2, \
       intensityExp1   - intensityExp2, \
       AngleExponent<angleExp1  - angleExp2, \
                     lengthExp1 - lengthExp2>::value

#define UNITS_UNIT_TEMPLATE_ARGS_INV_RESULT \
       ValueType1, \
       -massExp1, \
       -lengthExp1, \
       -timeExp1, \
       -currentExp1, \
       -temperatureExp1, \
       -amountExp1, \
       -intensityExp1, \
       -angleExp1

#define UNITS_UNIT_TEMPLATE_ARGS_SQRT_RESULT \
       ValueType1, \
       massExp1/2, \
       lengthExp1/2, \
       timeExp1/2, \
       currentExp1/2, \
       temperatureExp1/2, \
       amountExp1/2, \
       intensityExp1/2, \
       angleExp1/2


///
/// These types allow us to do handy things like Length x = zero and
/// have it magically to the Right Thing, rather than Length x =
/// Length::zero().
///
struct Zero {};
struct NegInfinity;
struct Infinity
{
  NegInfinity operator- () const;
};
struct NegInfinity
{
  Infinity operator- () const;
};

Zero        zero();
Infinity    infinity();
NegInfinity negInfinity();


///
/// NOTE: Technically, angular units of measure are Length/Length, so
/// any angle exponents will cancel with any number of length
/// exponents.  This awkward template struct is just so we can get the
/// exponents to cancel right.  The logic it's implementing at compile
/// time is this: if lengthExp != 0, then angleExp = 0.
///
template <int angleExp,
          bool lengthExpIsZero>
struct AngleExponentComputer
{
  static int const result = 0;
};

template <int angleExp>
struct AngleExponentComputer<angleExp,
                             true>
{
  static int const result = angleExp;
};

template <int angleExp,
          int lengthExp>
struct AngleExponent
{
  static int const value = AngleExponentComputer<angleExp,
                                                 (lengthExp == 0)>::result;
};


///
/// These help distinguish Unit and non-Unit types.
///
template <typename ValueType>
struct UnitlessType
{
  typedef ValueType Result;
};

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
class Unit;

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
struct UnitlessType<Unit<UNITS_UNIT_TEMPLATE_ARGS> >
{
  typedef ValueType_ Result;
};



///
/// Represents an arbitrary combination of basic units of measure.
///
template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
class Unit
{
public:
  typedef Unit BaseUnitType;

  // access the template parameters
  typedef ValueType_ ValueType;
  static const int massExp        = massExp_;
  static const int lengthExp      = lengthExp_;
  static const int timeExp        = timeExp_;
  static const int currentExp     = currentExp_;
  static const int temperatureExp = temperatureExp_;
  static const int amountExp      = amountExp_;
  static const int intensityExp   = intensityExp_;
  static const int angleExp       = angleExp_;


  ///
  /// Default constructor gives uninitialized value!
  ///
  Unit();

  ///
  /// Construct special values.  Note the value of the argument is
  /// ignored -- just the type matters.
  ///
  Unit(Zero        const &);
  Unit(Infinity    const &);
  Unit(NegInfinity const &);


  // Comparison operators

  bool      similarTo  (Unit const & b,
                        Unit const & tol =
                        defaultTolerance()) const;

  bool      operator== (Unit const & unit)     const;
  bool      operator!= (Unit const & unit)     const;
  bool      operator<  (Unit const & unit)     const;
  bool      operator>  (Unit const & unit)     const;
  bool      operator<= (Unit const & unit)     const;
  bool      operator>= (Unit const & unit)     const;


  // Arithmetic operators

  template <UNITS_UNIT_TEMPLATE_DECL_ARGS2>
  friend
  Unit<UNITS_UNIT_TEMPLATE_ARGS_MUL_RESULT>
  operator* (Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & lhs,
             Unit<UNITS_UNIT_TEMPLATE_ARGS2> const & rhs);

  template <UNITS_UNIT_TEMPLATE_DECL_ARGS2>
  friend
  Unit<UNITS_UNIT_TEMPLATE_ARGS_DIV_RESULT>
  operator/ (Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & lhs,
             Unit<UNITS_UNIT_TEMPLATE_ARGS2> const & rhs);

  template <typename ValueType1,
            UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(1)>
  friend
  Unit<UNITS_UNIT_TEMPLATE_ARGS_INV_RESULT>
  operator/ (ValueType1 const lhs,
             Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & rhs);

  //
  // Note:  A divide by zero will result in a value of Infinity
  //

  Unit      operator*  (ValueType const   scale)  const;
  Unit      operator/  (ValueType const   scale)  const;

  Unit    & operator*= (ValueType const   scale);
  Unit    & operator/= (ValueType const   scale);

  Unit      operator-  ()                      const;

  Unit      operator+  (Unit const & unit) const;
  Unit      operator-  (Unit const & unit) const;

  Unit    & operator+= (Unit const & unit);
  Unit    & operator-= (Unit const & unit);


  template <typename ValueType1,
            UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(1)>
  friend
  Unit<UNITS_UNIT_TEMPLATE_ARGS_SQRT_RESULT>
  sqrt(Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & value);

  template <typename ValueType1,
            UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(1)>
  friend
  Unit<UNITS_UNIT_TEMPLATE_ARGS1>
  abs(Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & value);


  static
  Unit const & defaultTolerance();


protected:
  ValueType _value;

  static
  Unit const _defaultTolerance;

  explicit
  Unit(ValueType const value);
};


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS>
min(Unit<UNITS_UNIT_TEMPLATE_ARGS> const & a,
    Unit<UNITS_UNIT_TEMPLATE_ARGS> const & b);

template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS>
max(Unit<UNITS_UNIT_TEMPLATE_ARGS> const & a,
    Unit<UNITS_UNIT_TEMPLATE_ARGS> const & b);


///
/// Specialization for unitless types to make them type-compatible with
/// the ValueType.
///
template <typename ValueType>
class Unit<ValueType, 0, 0, 0, 0, 0, 0, 0, 0>
{
public:
  ///
  /// Default constructor gives uninitialized value!
  ///
  Unit();

  ///
  /// Allow implicit conversion.
  ///
  Unit(ValueType const value);

  ///
  /// Type cast to values of type ValueType.
  ///
  operator ValueType() const;

  ///
  /// Allow assignments from values of type ValueType.
  ///
  Unit & operator= (ValueType const value);

  template <UNITS_UNIT_TEMPLATE_DECL_ARGS2>
  friend
  Unit<UNITS_UNIT_TEMPLATE_ARGS_MUL_RESULT>
  operator* (Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & lhs,
             Unit<UNITS_UNIT_TEMPLATE_ARGS2> const & rhs);

  template <UNITS_UNIT_TEMPLATE_DECL_ARGS2>
  friend
  Unit<UNITS_UNIT_TEMPLATE_ARGS_DIV_RESULT>
  operator/ (Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & lhs,
             Unit<UNITS_UNIT_TEMPLATE_ARGS2> const & rhs);

  template <typename ValueType1,
            UNITS_UNIT_TEMPLATE_EXP_DECL_ARGS(1)>
  friend
  Unit<UNITS_UNIT_TEMPLATE_ARGS_INV_RESULT>
  operator/ (ValueType1 const lhs,
             Unit<UNITS_UNIT_TEMPLATE_ARGS1> const & rhs);

private:
  ValueType _value;
};


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
bool similar (Unit<UNITS_UNIT_TEMPLATE_ARGS> const & a,
              Unit<UNITS_UNIT_TEMPLATE_ARGS> const & b,
              Unit<UNITS_UNIT_TEMPLATE_ARGS> const & tol =
              Unit<UNITS_UNIT_TEMPLATE_ARGS>::defaultTolerance());


template <UNITS_UNIT_TEMPLATE_DECL_ARGS>
Unit<UNITS_UNIT_TEMPLATE_ARGS>
operator*    (ValueType_                     const   scale,
              Unit<UNITS_UNIT_TEMPLATE_ARGS> const & unit);


#define UNITS_DECLARE_BASE_UNIT(Name, \
                                massExp, \
                                lengthExp, \
                                timeExp, \
                                currentExp, \
                                temperatureExp, \
                                amountExp, \
                                intensityExp, \
                                angleExp) \
template <typename ValueType> \
struct Name##Traits \
{ \
  typedef Unit<ValueType, \
               massExp, \
               lengthExp, \
               timeExp, \
               currentExp, \
               temperatureExp, \
               amountExp, \
               intensityExp, \
               angleExp> BaseUnitType; \
}; \
 \
typedef Name##Traits<double>::BaseUnitType d##Name; \
typedef Name##Traits<float>::BaseUnitType f##Name; \
typedef Name##Traits<UNITS_DEFAULT_VALUE_TYPE>::BaseUnitType Name



} // namespace Units


#include "Unit.i"

#endif  // Units_Unit_h
