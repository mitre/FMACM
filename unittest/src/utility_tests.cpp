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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <gtest/gtest.h>
#include "utility/BoundedValue.h"

TEST(BoundedValue, doubles) {
   const double not_zero = -0.5;
   BoundedValue<double, -1, 1> bounded_value_1(not_zero);
   EXPECT_DOUBLE_EQ(not_zero, bounded_value_1);  // test that double values are equal with no cast operation

   try {
      bounded_value_1 = 10;  // out-of-bounds: throw a run-time exception
      FAIL();
   } catch (const BoundedValueException<double> &e) {
      // std::cerr << e.what() << '\n';
   }

   const double updated_value(1);
   try {
      BoundedValue<double, 0, 1> bounded_value_2(updated_value);
      bounded_value_1 = bounded_value_2;
      EXPECT_DOUBLE_EQ(updated_value, bounded_value_1);
   } catch (const BoundedValueException<double> &e) {
      FAIL();
   }
}