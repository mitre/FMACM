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

#include "loader/DecodedStream.h"

#include <limits>

using namespace std;

DecodedStream::DecodedStream(void) {}

//----------------------------------------------------------------------------------

DecodedStream::~DecodedStream(void) {}

//----------------------------------------------------------------------------------

bool find_num(string temp, int j) {
   if (!(('0' <= temp[0] && temp[0] <= '9') || temp[0] == '-')) {
      return false;
   }
   for (int i = 1; i <= int(temp.length() - 1); i++) {
      if (j == 0) {
         if (!(('0' <= temp[i] && temp[i] <= '9'))) {
            return false;
         }
      }
      if (j == 1) {
         if (!(('0' <= temp[i] && temp[i] <= '9') || temp[i] == '.')) {
            return false;
         }
      }
   }
   return true;
}

//----------------------------------------------------------------------------------

bool DecodedStream::get_datum(string &s) {
   s = get_next().get_Data();

   if (!DecodedStream::ok())  // if not ok return false
   {
      return false;
   }

   return !s.empty();
}

//----------------------------------------------------------------------------------

bool DecodedStream::get_datum(short &s) {
   // WinDef fefines these macros which prevents us from these thinges elswhere which causes problems
   // ADM jan 24 2011
#undef max
#undef min

   string temp = get_next().get_Data();
   static const short big = numeric_limits<short>::max();
   static const short small = numeric_limits<short>::min();

   if (temp.empty() || !DecodedStream::ok())  // if temp is empty or stream not ok
   {
      return false;
   }

   if (!find_num(temp, 0)) {
      return false;
   }

   if (atoi(temp.c_str()) == big)  // too big to be short
   {
      return false;
   }
   if (atoi(temp.c_str()) == small)  // too small to be short
   {
      return false;
   }
   if (temp != "0") {
      if (atoi(temp.c_str()) == 0) {
         return false;
      }
   }
   s = atoi(temp.c_str());

   return true;
}

//----------------------------------------------------------------------------------

bool DecodedStream::get_datum(unsigned short &s) {
   // WinDef fefines these macros which prevents us from these thinges elswhere which causes problems
   // ADM jan 24 2011
#undef max
#undef min

   string temp = get_next().get_Data();
   static const unsigned short big = numeric_limits<unsigned short>::max();
   static const unsigned short small = numeric_limits<unsigned short>::min();

   if (temp.empty() || !DecodedStream::ok())  // if temp is empty or stream not ok
   {
      return false;
   }

   if (!find_num(temp, 0)) {
      return false;
   }

   if (atoi(temp.c_str()) == big)  // too big to be unsigned short
   {
      return false;
   }
   if (atoi(temp.c_str()) == small)  // too small to be unsigned short
   {
      return false;
   }
   if (temp != "0") {
      if (atoi(temp.c_str()) == 0) {
         return false;
      }
   }
   s = atoi(temp.c_str());

   return true;
}

//----------------------------------------------------------------------------------

bool DecodedStream::get_datum(int &s) {
   // WinDef fefines these macros which prevents us from these thinges elswhere which causes problems
   // ADM jan 24 2011
#undef max
#undef min

   string temp = get_next().get_Data();
   static const int big = numeric_limits<int>::max();
   static const int small = numeric_limits<int>::min();

   if (temp.empty() || !DecodedStream::ok())  // if temp is empty or stream not ok
   {
      return false;
   }

   if (!find_num(temp, 0)) {
      return false;
   }

   if (atoi(temp.c_str()) == big)  // too big to be int
   {
      return false;
   }
   if (atoi(temp.c_str()) == small)  // too small to be int
   {
      return false;
   }
   if (temp != "0") {
      if (atoi(temp.c_str()) == 0) {
         return false;
      }
   }
   s = atoi(temp.c_str());

   return true;
}

//----------------------------------------------------------------------------------

bool DecodedStream::get_datum(unsigned int &s) {
   // WinDef fefines these macros which prevents us from these thinges elswhere which causes problems
   // ADM jan 24 2011
#undef max
#undef min

   string temp = get_next().get_Data();
   static const unsigned int big = numeric_limits<unsigned int>::max();
   static const unsigned int small = numeric_limits<unsigned int>::min();

   if (temp.empty() || !DecodedStream::ok())  // if temp is empty or stream not ok
   {
      return false;
   }

   if (!find_num(temp, 0)) {
      return false;
   }

   if (atoi(temp.c_str()) == big)  // too big to be unsigned int
   {
      return false;
   }
   if (atoi(temp.c_str()) == small)  // too small to be unsigned int
   {
      return false;
   }
   if (temp != "0") {
      if (atoi(temp.c_str()) == 0) {
         return false;
      }
   }
   s = atoi(temp.c_str());

   return true;
}

//----------------------------------------------------------------------------------

bool DecodedStream::get_datum(double &s) {
   string temp = get_next().get_Data();

   // wont be necessary- double goes out to 308 digits precision
   // double big = numeric_limits<double>::max();
   // double small = numeric_limits<double>::min();

   if (temp.empty() || !DecodedStream::ok())  // if temp is empty or stream not ok
   {
      return false;
   }

   if (!find_num(temp, 1)) {
      return false;
   }

   if (temp != "0" && temp != "0.0" && temp != "-0" && temp != "-0.0") {
      if (atof(temp.c_str()) == 0) {
         return false;
      }
   }
   s = atof(temp.c_str());
   return true;
}

//----------------------------------------------------------------------------------

bool DecodedStream::get_datum(bool &s) {
   string temp = get_next().get_Data();
   vector<string> testbool;
   testbool.push_back("yes");
   testbool.push_back("no");
   testbool.push_back("true");
   testbool.push_back("false");
   testbool.push_back("0");
   testbool.push_back("1");

   if (temp.empty() || !DecodedStream::ok())  // if temp is empty or stream not ok
   {
      return false;
   }

   for (int i = 0; i <= int(testbool.size() - 1); i++) {
      if (testbool[i] == temp) {
         if (temp == "true" || temp == "yes" || temp == "1") {
            s = true;
         }
         if (temp == "false" || temp == "no" || temp == "0") {
            s = false;
         }
         break;
      }
      if (i == testbool.size() - 1) {
         return false;
      }
   }
   return true;
}

//----------------------------------------------------------------------------------

bool DecodedStream::get_datum(long &s) {
   string temp = get_next().get_Data();

   if (temp.empty() || !DecodedStream::ok()) {
      return false;
   }

   if (!find_num(temp, 0)) {
      return false;
   }

   try {
      size_t idx = 0;
      long value = std::stol(temp, &idx, 10);
      if (idx != temp.size()) {
         // Not all characters were consumed, so not a valid number
         return false;
      }
      s = value;
      return true;
   } catch (const std::invalid_argument &) {
      return false;
   } catch (const std::out_of_range &) {
      return false;
   }
}

bool DecodedStream::get_datum(float &s) {
   string temp = get_next().get_Data();
   static const float big = numeric_limits<float>::max();
   static const float small = numeric_limits<float>::min();

   if (temp.empty() || !DecodedStream::ok())  // if temp is empty or stream not ok
   {
      return false;
   }

   if (!find_num(temp, 0)) {
      return false;
   }

   if (atoi(temp.c_str()) == big)  // too big to be int
   {
      return false;
   }
   if (atoi(temp.c_str()) == small)  // too small to be int
   {
      return false;
   }
   if (temp != "0") {
      if (atoi(temp.c_str()) == 0) {
         return false;
      }
   }
   s = atoi(temp.c_str());

   return true;
}

bool DecodedStream::get_datum(Units::MetersLength &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::MetersLength(d);
   return true;
}

bool DecodedStream::get_datum(Units::NauticalMilesLength &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::NauticalMilesLength(d);
   return true;
}

bool DecodedStream::get_datum(Units::FeetLength &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::FeetLength(d);
   return true;
}

bool DecodedStream::get_datum(Units::KilometersLength &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::KilometersLength(d);
   return true;
}

bool DecodedStream::get_datum(Units::MetersPerSecondSpeed &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::MetersPerSecondSpeed(d);
   return true;
}

bool DecodedStream::get_datum(Units::KnotsSpeed &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::KnotsSpeed(d);
   return true;
}

bool DecodedStream::get_datum(Units::SecondsTime &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::SecondsTime(d);
   return true;
}

bool DecodedStream::get_datum(Units::DegreesAngle &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::DegreesAngle(d);
   return true;
}

bool DecodedStream::get_datum(Units::RadiansAngle &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::RadiansAngle(d);
   return true;
}

bool DecodedStream::get_datum(Units::KilogramsMass &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::KilogramsMass(d);
   return true;
}

bool DecodedStream::get_datum(Units::PoundsMass &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::PoundsMass(d);
   return true;
}

bool DecodedStream::get_datum(Units::KilogramsPerHourMassFlowRate &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::KilogramsPerHourMassFlowRate(d);
   return true;
}

bool DecodedStream::get_datum(Units::PoundsPerHourMassFlowRate &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::PoundsPerHourMassFlowRate(d);
   return true;
}

bool DecodedStream::get_datum(Units::FeetArea &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::FeetArea(d);
   return true;
}

bool DecodedStream::get_datum(Units::MetersArea &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::MetersArea(d);
   return true;
}

bool DecodedStream::get_datum(Units::FeetPerMinuteSpeed &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::FeetPerMinuteSpeed(d);
   return true;
}

bool DecodedStream::get_datum(Units::SecondsPerNauticalMileInvertedSpeed &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::SecondsPerNauticalMileInvertedSpeed(d);
   return true;
}

bool DecodedStream::get_datum(Units::HertzFrequency &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::HertzFrequency(d);
   return true;
}

bool DecodedStream::get_datum(Units::MetersPerSecondSquaredLengthGain &s) {
   double d = 0;
   bool getDouble = get_datum(d);
   if (!getDouble) {
      return false;
   }
   s = Units::MetersPerSecondSquaredLengthGain(d);
   return true;
}
