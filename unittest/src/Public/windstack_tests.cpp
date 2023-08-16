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
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <gtest/gtest.h>
#include <iostream>
#include "loader/DecodedStream.h"
#include "public/WindStack.h"

using namespace aaesim::open_source;

namespace aaesim {
namespace test {
namespace open_source {
TEST(WindStack, operatorEqEq) {

   // 1.empty vs empty
   WindStack ws0;
   WindStack ws1;
   EXPECT_EQ(ws0, ws1);

   // 2.empty vs something
   WindStack ws2;
   ws2.SetBounds(0, 4);
   for (auto ix = 0; ix <= 4; ix++) {
      ws2.Insert(ix, Units::MetersLength((double)ix * 20.0), Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
   }
   if (ws0 == ws2) {
      std::string err = "WindStack::operator== empty vs something fails, expected false.";
      FAIL();
   }

   // 3.something vs same something
   WindStack ws3;
   ws3.SetBounds(0, 4);
   for (auto ix = 0; ix <= 4; ix++) {
      ws3.Insert(ix, Units::MetersLength((double)ix * 20.0), Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
   }
   if (!(ws2 == ws3)) {
      std::string err = "WindStack::operator== something vs same something fails, expected true.";
      FAIL();
   }

   // 4.something vs. something-different low row.
   WindStack ws4;
   ws4.SetBounds(1, 4);
   for (auto ix = 1; ix <= 4; ix++) {
      ws4.Insert(ix, Units::MetersLength((double)ix * 20.0), Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
   }
   if (ws2 == ws4) {
      std::string err = "WindStack::operator== something vs something different low row fails, expected false.";
      FAIL();
   }

   // 5.something vs. something-different high row.
   WindStack ws5;
   ws5.SetBounds(0, 5);
   for (auto ix = 0; ix <= 5; ix++) {
      ws5.Insert(ix, Units::MetersLength((double)ix * 20.0), Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
   }
   if (ws2 == ws5) {
      std::string err = "WindStack::operator== something vs something different high row fails, expected false.";
      FAIL();
   }

   // 6.something vs something with 1 different altitude.
   WindStack ws6;
   ws6.SetBounds(0, 4);
   for (auto ix = 0; ix <= 4; ix++) {
      if (ix != 2) {
         ws6.Insert(ix, Units::MetersLength((double)ix * 20.0),
                    Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
      } else {
         ws6.Insert(ix, Units::MetersLength(-15.0), Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
      }
   }
   if (ws2 == ws6) {
      std::string err = "WindStack::operator== something vs something with 1 different altitude fails, expected false.";
      FAIL();
   }

   // 7.something vs something with 1 different speed.
   WindStack ws7;
   ws7.SetBounds(0, 4);
   for (auto ix = 0; ix <= 4; ix++) {
      if (ix != 4) {
         ws7.Insert(ix, Units::MetersLength((double)ix * 20.0),
                    Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
      } else {
         ws7.Insert(ix, Units::MetersLength((double)ix * 20.0), Units::MetersPerSecondSpeed(-93.0));
      }
   }
   if (ws2 == ws7) {
      std::string err = "WindStack::operator== something vs something with 1 different speed fails, expected false.";
      FAIL();
   }
}

TEST(WindStack, operatorNotEq) {

   // 1.something vs. equals something
   WindStack ws0;
   ws0.SetBounds(0, 4);
   WindStack ws1;
   ws1.SetBounds(0, 4);
   for (auto ix = 0; ix <= 4; ix++) {
      ws0.Insert(ix, Units::MetersLength((double)ix * 20.0), Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
      ws1.Insert(ix, Units::MetersLength((double)ix * 20.0), Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
   }

   if (ws0 != ws1) {
      std::string err = "WindStack::operator!= something vs equals something fails, expected false.";
      FAIL();
   }

   // 2.something vs. not equals something
   WindStack ws2;
   ws2.SetBounds(0, 4);
   for (auto ix = 0; ix <= 4; ix++) {
      if (ix != 3) {
         ws2.Insert(ix, Units::MetersLength((double)ix * 20.0),
                    Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
      } else {
         ws2.Insert(ix, Units::MetersLength(-((double)ix * 20.0)),
                    Units::MetersPerSecondSpeed((double)(ix - 2) * 3 + 175.0));
      }
   }
   if (!(ws0 != ws2)) {
      std::string err = "WindStack::operator!= something vs not equals something fails, expected true.";
      FAIL();
   }
}

TEST(WindStack, sort_basic) {
   WindStack unsorted_stack(0, 2);
   unsorted_stack.Insert(0, Units::FeetLength(10), Units::KnotsSpeed(20));
   unsorted_stack.Insert(1, Units::ZERO_LENGTH, Units::ZERO_SPEED);
   unsorted_stack.Insert(2, Units::FeetLength(5), Units::ZERO_SPEED);

   WindStack sorted_stack = unsorted_stack;
   sorted_stack.SortAltitudesAscending();

   EXPECT_FALSE(unsorted_stack == sorted_stack);
   EXPECT_EQ(sorted_stack.GetAltitude(0), Units::ZERO_LENGTH);
   EXPECT_EQ(sorted_stack.GetSpeed(0), Units::ZERO_SPEED);
   EXPECT_EQ(sorted_stack.GetAltitude(1), Units::FeetLength(5));
   EXPECT_EQ(sorted_stack.GetSpeed(1), Units::ZERO_SPEED);
   EXPECT_EQ(sorted_stack.GetAltitude(2), Units::FeetLength(10));
   EXPECT_EQ(sorted_stack.GetSpeed(2), Units::KnotsSpeed(20));
}

TEST(WindStack, sort_shifted_min_max) {
   WindStack unsorted_stack(2, 4);
   unsorted_stack.Insert(2, Units::FeetLength(10), Units::KnotsSpeed(20));
   unsorted_stack.Insert(3, Units::ZERO_LENGTH, Units::ZERO_SPEED);
   unsorted_stack.Insert(4, Units::FeetLength(5), Units::ZERO_SPEED);

   WindStack sorted_stack = unsorted_stack;
   sorted_stack.SortAltitudesAscending();

   EXPECT_FALSE(unsorted_stack == sorted_stack);
   EXPECT_EQ(sorted_stack.GetAltitude(2), Units::ZERO_LENGTH);
   EXPECT_EQ(sorted_stack.GetSpeed(2), Units::ZERO_SPEED);
   EXPECT_EQ(sorted_stack.GetAltitude(3), Units::FeetLength(5));
   EXPECT_EQ(sorted_stack.GetSpeed(3), Units::ZERO_SPEED);
   EXPECT_EQ(sorted_stack.GetAltitude(4), Units::FeetLength(10));
   EXPECT_EQ(sorted_stack.GetSpeed(4), Units::KnotsSpeed(20));
}

}  // namespace open_source
}  // namespace test
}  // namespace aaesim