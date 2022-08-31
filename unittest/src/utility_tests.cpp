#include <gtest/gtest.h>
#include "utility/BoundedValue.h"

TEST(BoundedValue, doubles) {
   const double not_zero = -0.5;
   BoundedValue<double, -1, 1> bounded_value_1(not_zero);
   EXPECT_DOUBLE_EQ(not_zero, bounded_value_1); // test that double values are equal with no cast operation

   try
   {
      bounded_value_1 = 10; // out-of-bounds: throw a run-time exception
      FAIL();
   }
   catch(const BoundedValueException<double>& e)
   {
      // std::cerr << e.what() << '\n';
   }
   
   const double updated_value(1);
   try
   {
      BoundedValue<double, 0, 1> bounded_value_2(updated_value);
      bounded_value_1 = bounded_value_2;  
      EXPECT_DOUBLE_EQ(updated_value, bounded_value_1);
   }
   catch(const BoundedValueException<double>& e)
   {
      FAIL();
   }

}