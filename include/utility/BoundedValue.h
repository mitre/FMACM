// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001
// and is subject to Federal Aviation Administration Acquisition Management System
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE
// Corporation and do not necessarily reflect the views of the Federal Aviation
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

/*
Original idea in the public domain from: https://stackoverflow.com/a/13730310
*/

#pragma once
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

#define STRINGIZE(x) #x
#define STRINGIFY(x) STRINGIZE( x )

// handling for runtime value errors
#define BOUNDED_VALUE_ASSERT(MIN, MAX, VAL)                                                                       \
   do {                                                                                                           \
      if ((VAL) < (MIN) || (VAL) > (MAX)) {                                                                       \
         bounded_value_assert_helper(MIN, MAX, VAL, "BOUNDED_VALUE_ASSERT at " __FILE__ ":" STRINGIFY(__LINE__)); \
      }                                                                                                           \
   } while (0)

template <typename T>
struct BoundedValueException : public std::range_error {
   virtual ~BoundedValueException() noexcept = default;
   BoundedValueException() = delete;
   BoundedValueException(BoundedValueException const &other) = default;
   BoundedValueException(BoundedValueException &&source) = default;

   BoundedValueException(T min, T max, T val, std::string const &message)
      : std::range_error(message), minval_(min), maxval_(max), val_(val) {}

   T const minval_;
   T const maxval_;
   T const val_;
};

template <typename T>
void bounded_value_assert_helper(T min, T max, T val, char const *message = nullptr) {
   std::ostringstream oss;
   oss << "BoundedValueException: !(" << min << "<=" << val << "<=" << max << ")";
   if (message) {
      oss << " - " << message;
   }
   throw BoundedValueException<T>(min, max, val, oss.str());
}

template <typename T, int Tmin, int Tmax>
class BoundedValue {
   static_assert(std::is_arithmetic<T>::value, "T must be arithmetic");

  public:
   typedef T value_type;
   enum { min_value_int = Tmin, max_value_int = Tmax };
   static constexpr T min_value = static_cast<T>(Tmin);
   static constexpr T max_value = static_cast<T>(Tmax);
   typedef BoundedValue<value_type, min_value_int, max_value_int> SelfType;

   // runtime checking constructor:
   explicit BoundedValue(T runtime_value) : val_(runtime_value) {
      BOUNDED_VALUE_ASSERT(min_value, max_value, runtime_value);
   }
   // compile-time checked constructors:
   constexpr BoundedValue() : val_(min_value) {}

   template <typename otherT, int otherTmin, int otherTmax>
   BoundedValue(BoundedValue<otherT, otherTmin, otherTmax> const &other)
      : val_(static_cast<value_type>(other))  // explicitly convert underlying value
   {
      static_assert(otherTmin >= Tmin, "conversion disallowed from BoundedValue with lower min");
      static_assert(otherTmax <= Tmax, "conversion disallowed from BoundedValue with higher max");
   }

   // compile-time checked assignments:
   BoundedValue &operator=(SelfType const &other) {
      val_ = other.val_;
      return *this;
   }

   template <typename otherT, int otherTmin, int otherTmax>
   BoundedValue &operator=(BoundedValue<otherT, otherTmin, otherTmax> const &other) {
      static_assert(otherTmin >= Tmin, "conversion disallowed from BoundedValue with lower min");
      static_assert(otherTmax <= Tmax, "conversion disallowed from BoundedValue with higher max");
      val_ = static_cast<value_type>(other);  // explicit conversion of underlying value
      return *this;
   }
   // run-time checked assignment:
   BoundedValue &operator=(T const &val) {
      BOUNDED_VALUE_ASSERT(min_value, max_value, val);
      val_ = val;
      return *this;
   }

   // C++20: conversion operator
   operator T const &() const { return val_; }

   // Comparison operators (C++11 compatible)
   bool operator==(BoundedValue const &other) const { return val_ == other.val_; }
   bool operator!=(BoundedValue const &other) const { return val_ != other.val_; }
   bool operator<(BoundedValue const &other) const { return val_ < other.val_; }
   bool operator<=(BoundedValue const &other) const { return val_ <= other.val_; }
   bool operator>(BoundedValue const &other) const { return val_ > other.val_; }
   bool operator>=(BoundedValue const &other) const { return val_ >= other.val_; }

   // Comparison operators with raw values
   bool operator==(T const &val) const { return val_ == val; }
   bool operator<(T const &val) const { return val_ < val; }
   bool operator<=(T const &val) const { return val_ <= val; }
   bool operator>(T const &val) const { return val_ > val; }
   bool operator>=(T const &val) const { return val_ >= val; }
   bool operator!=(T const &val) const { return val_ != val; }

  private:
   value_type val_;
};
