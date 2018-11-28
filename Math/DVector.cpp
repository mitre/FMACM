// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "math/DVector.h"
#include <iostream>
#include "math/InvalidIndexException.h"

using std::cout;
using std::endl;

// default constructor
DVector::DVector(void) {
   minIndex = 0;
   maxIndex = -1;
   vector = NULL;
}

// constructor that takes the bouonds of the vector as arguments
// NOTE: max index is inclusive
DVector::DVector(int min,
                 int max) {
   // gets the size of the vector
   int size = max - min + 1;

   // set the min/max index
   minIndex = min;
   maxIndex = max;

   // allocate the array
   vector = new double[size];
}

// copy constructor to copy the values of a given DVector
DVector::DVector(const DVector &in) {
   int size = in.maxIndex - in.minIndex + 1;
   minIndex = in.minIndex;
   maxIndex = in.maxIndex;
   vector = new double[size];

   //loop to copy the values of the given DVector
   for (int loop = 0; loop < size; loop++) {
      vector[loop] = in.vector[loop];
   }
}

// deconstructor to free dynamicly allocated array
DVector::~DVector(void) {
   delete[] vector;
   vector = NULL;
}

// method to get the double at a given index
double DVector::get(int index) {
   if (inRange(index)) {
      return vector[index - minIndex];
   }

   throw InvalidIndexException(index, minIndex, maxIndex);
}

// method to set the value at a given index
void DVector::set(int index,
                  double value) {
   if (inRange(index)) {
      vector[index - minIndex] = value;
   } else {
      throw InvalidIndexException(index, minIndex, maxIndex);
   }
}

// private method to check if the given index is in the valid range of the DVector
// NOTE: the max index is inclusive
bool DVector::inRange(int index) const {
   bool result = false;

   if (index >= minIndex && index <= maxIndex) {
      result = true;
   }

   return result;
}

// method to set the bounds of the vector and dynamicly allocate the proper size
// NOTE: max index is inclusive
void DVector::setBounds(int min,
                        int max) {
   int size0 = this->maxIndex - this->minIndex + 1;
   minIndex = min;
   maxIndex = max;

   int size = max - min + 1;

   if (size != size0) {
      delete[] vector;
      vector = new double[size];
   }
}

// method to overload the [] indexing operator to make it act like a double array
double &DVector::operator[](int index) {
   // if index is in valid range get the value
   if (inRange(index)) {
      return vector[index - minIndex];
   }

   throw InvalidIndexException(index, minIndex, maxIndex);
}

// method to overload the [] indexing operator to make it act like a double array
const double &DVector::operator[](int index) const {
   // if index is in valid range get the value
   if (inRange(index)) {
      return vector[index - minIndex];
   }

   throw InvalidIndexException(index, minIndex, maxIndex);
}

// overloads the equals operator
DVector &DVector::operator=(const DVector &in) {
   if (this != &in) {
      setBounds(in.minIndex, in.maxIndex);
      int size = maxIndex - minIndex + 1;
      //loop to copy the values of the given DVector
      for (int loop = 0; loop < size; loop++) {
         vector[loop] = in.vector[loop];
      }
   }

   return *this;
}

// overloads less than operator
bool DVector::operator<(const DVector &other) const {
   return vector[0] < other.vector[0];
}

// getter methods
int DVector::get_min() {
   return minIndex;
}

int DVector::get_max() {
   return maxIndex;
}

