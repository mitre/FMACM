/*
Original idea in the public domain from: https://stackoverflow.com/a/13730310
*/
#pragma once
#include <sstream>
#include <stdexcept>

#define STRINGIZE(x) #x
#define STRINGIFY(x) STRINGIZE( x )

// handling for runtime value errors
#define BOUNDED_VALUE_ASSERT(MIN, MAX, VAL)                                                                    \
   if ((VAL) < (MIN) || (VAL) > (MAX)) {                                                                       \
      bounded_value_assert_helper(MIN, MAX, VAL, "BOUNDED_VALUE_ASSERT at " __FILE__ ":" STRINGIFY(__LINE__)); \
   }

template <typename T, int Tmin, int Tmax>
class BoundedValue {
  public:
   typedef T value_type;
   enum { min_value = Tmin, max_value = Tmax };
   typedef BoundedValue<value_type, min_value, max_value> SelfType;

   // runtime checking constructor:
   explicit BoundedValue(T runtime_value) : val_(runtime_value) {
      BOUNDED_VALUE_ASSERT(min_value, max_value, runtime_value);
   }
   // compile-time checked constructors:
   BoundedValue(SelfType const &other) : val_(other) {}
   BoundedValue(SelfType &&other) : val_(other) {}
   BoundedValue() : val_(Tmin) {}

   template <typename otherT, int otherTmin, int otherTmax>
   BoundedValue(BoundedValue<otherT, otherTmin, otherTmax> const &other)
      : val_(other)  // will just fail if T, otherT not convertible
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
      val_ = other;  // will just fail if T, otherT not convertible
      return *this;
   }
   // run-time checked assignment:
   BoundedValue &operator=(T const &val) {
      BOUNDED_VALUE_ASSERT(min_value, max_value, val);
      val_ = val;
      return *this;
   }

   operator T const &() const { return val_; }

  private:
   value_type val_;
};

template <typename T>
struct BoundedValueException : public std::range_error {
   virtual ~BoundedValueException() throw() {}
   BoundedValueException() = delete;
   BoundedValueException(BoundedValueException const &other) = default;
   BoundedValueException(BoundedValueException &&source) = default;

   BoundedValueException(int min, int max, T val, std::string const &message)
      : std::range_error(message), minval_(min), maxval_(max), val_(val) {}

   int const minval_;
   int const maxval_;
   T const val_;
};

template <typename T>
void bounded_value_assert_helper(int min, int max, T val, char const *message = NULL) {
   std::ostringstream oss;
   oss << "BoundedValueException: !(" << min << "<=" << val << "<=" << max << ")";
   if (message) {
      oss << " - " << message;
   }
   throw BoundedValueException<T>(min, max, val, oss.str());
}
