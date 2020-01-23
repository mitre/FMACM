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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "math/InvalidIndexException.h"

template<class T>
class UVector
{
private:
   int minIndex;
   int maxIndex;
   T *vector;

public:
   // constructor/destructors
   UVector(void) {
      minIndex = 0;
      maxIndex = -1;
      vector = NULL;
   }

   UVector(const int min,
           const int max) {
      // gets the size of the vector
      int size = max - min + 1;

      // set the min/max index
      minIndex = min;
      maxIndex = max;

      // allocate the array
      vector = new T[size];
   }

   UVector(const UVector<T> &in) {
      int size = in.maxIndex - in.minIndex + 1;
      minIndex = in.minIndex;
      maxIndex = in.maxIndex;
      vector = new T[size];

      //loop to copy the values of the given DVector
      for (int loop = 0; loop < size; loop++) {
         vector[loop] = in.vector[loop];
      }
   }

   ~UVector(void) {
      delete[] vector;
      vector = NULL;
   }

   // get/set methods
   // method to get the value at a given index
   T get(const int index) const {
      if (inRange(index)) {
         return vector[index - minIndex];
      }

      throw InvalidIndexException(index, minIndex, maxIndex);
   }

   // method to set the value at a given index
   void set(const int index,
            const T value) {
      if (inRange(index)) {
         vector[index - minIndex] = value;
      } else {
         throw InvalidIndexException(index, minIndex, maxIndex);
      }
   }

   // method to set the bounds of the Vector
   void setBounds(const int min,
                  const int max) {
      int size0 = this->maxIndex - this->minIndex + 1;
      minIndex = min;
      maxIndex = max;

      int size = max - min + 1;

      if (size != size0) {
         delete[] vector;
         vector = new T[size];
      }
   }

   // method to check if a given index is in the array
   bool inRange(const int index) const {
      bool result = false;

      if (index >= minIndex && index <= maxIndex) {
         result = true;
      }

      return result;
   }

   // getter methods
   const int get_min() const {
      return minIndex;
   }

   const int get_max() const {
      return maxIndex;
   }

   // overloads the array index operator
   T &operator[](const int index) {
      // if index is in valid range get the value
      if (inRange(index)) {
         return vector[index - minIndex];
      }

      throw InvalidIndexException(index, minIndex, maxIndex);
   }

   const T &operator[](const int index) const {
      // if index is in valid range get the value
      if (inRange(index)) {
         return vector[index - minIndex];
      }

      throw InvalidIndexException(index, minIndex, maxIndex);
   }

   // overloads the equals operator
   UVector<T> &operator=(const UVector<T> &in) {
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

   // overloads less than opertor for sort
   bool operator<(const UVector<T> &other) const {
      return vector[0] < other.vector[0];
   }
};

