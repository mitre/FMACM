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

/*
 * UMatrix.h
 *
 *  Created on: Nov 12, 2015
 *      Author: klewis
 */

#pragma once

template<class T>
class UMatrix
{
private:
   static char *MULTIPLICATION_DIMENSIONS_MESSAGE;
   UVector <T> *rows;
   int minRow;
   int maxRow;

public:
   class IncompatibleDimensionsException : public std::exception
   {
   public:
      IncompatibleDimensionsException(char *explanation);

      virtual ~IncompatibleDimensionsException() throw();

      virtual const char *what() const throw();

   private:
      const char *explanation;
   };

   // default Constructor/Destructors
   UMatrix(void) {
      minRow = 0;
      maxRow = -1;
      rows = NULL;
   }

   ~UMatrix(void) {
      delete[] rows;
      rows = NULL;
   }

   // Copy Constructor
   UMatrix(const UMatrix<T> &in) {
      // calculates size of Matrix
      int size = in.maxRow - in.minRow + 1;

      // sets row min/max
      maxRow = in.maxRow;
      minRow = in.minRow;

      // allocates the new UMatrix (array of UVector<T>s)
      rows = new UVector<T>[size];

      // loop to copy the values of the given UMatrix
      for (int loop = 0; loop < size; loop++) {
         rows[loop] = in.rows[loop];
      }
   }

   // Constructor that takes row and column min/max values
   UMatrix(int inRowMin,
           int inRowMax,
           int inColMin,
           int inColMax) {
      // calculates the size of the UMatrix
      int size = inRowMax - inRowMin + 1;

      // sets row min/max
      maxRow = inRowMax;
      minRow = inRowMin;

      // allocates the new UMatrix (array of UVector<T>s)
      rows = new UVector<T>[size];

      // loop to set the column size (UVector<T> min/max)
      for (int loop = 0; loop < size; loop++) {
         rows[loop].setBounds(inColMin, inColMax);
      }
   }

   // Constructor that takes a 2D array
   UMatrix(T **array_in,
           int inRowMin,
           int inRowMax,
           int inColMin,
           int inColMax) {
      // calculates the size of the UMatrix
      int size = inRowMax - inRowMin + 1;

      // sets row min/max
      maxRow = inRowMax;
      minRow = inRowMin;

      // allocates the new UMatrix (array of UVector<T>s)
      rows = new UVector<T>[size];

      // loop to set the column size (UVector<T> min/max)
      for (int loop = 0; loop < size; loop++) {
         rows[loop].setBounds(inColMin, inColMax);
      }

      // get row and column sizes
      //int column_size = inColMax-inColMin;
      //int row_size = inRowMax-inRowMin;

      // NOTE this is dangerous code, the array MUST be the same size as the given row/column information
      // failure to match the bounds given will cause dangerous memory access!!!
      // this only works because 2d arrays are stored in sequential memory addresses
      // if the array is dynamically allocated this WILL cause dangerous memory access!!!
      T *memory_pointer; // pointer to point to the memory address of the 2d array
      memory_pointer = (T *) array_in; // accesses the the memory address of the first element
      for (int outer = 0; outer < size; outer++) {
         for (int inner = inColMin; inner <= inColMax; inner++) {
            rows[outer].set(inner, (*memory_pointer));
            memory_pointer++; // iterates the memory address
         }
      }
   }

   // get/set methods
   T get(int row,
         int column) const {
      // check if in valid range of UMatrix
      if (inRange(row)) {
         // uses the UVector<T>s overloaded array operator to get the value
         return rows[row - minRow][column];
      }

      // if not in valid range throw Invalid Index Exception
      throw InvalidIndexException();
   }

   void set(int row,
            int column,
            T value) {
      // check if in valid range of UMatrix
      if (inRange(row)) {
         // uses the UVector<T> overloaded array operator to set the value
         rows[row - minRow][column] = value;
      } else {
         // else not in range throw Invalid Index Exception
         throw InvalidIndexException();
      }
   }

   // method to set the bounds of the Matrix
   void setBounds(int inRowMin,
                  int inRowMax,
                  int inColMin,
                  int inColMax) {
      // calculates the size of the UMatrix
      int size = inRowMax - inRowMin + 1;

      // sets row min/max
      maxRow = inRowMax;
      minRow = inRowMin;

      // allocates the new UMatrix (array of UVector<T>s)
      delete[] rows;
      rows = new UVector<T>[size];

      // loop to set the column size (UVector<T> min/max)
      for (int loop = 0; loop < size; loop++) {
         rows[loop].setBounds(inColMin, inColMax);
      }
   }

   // method to check if a given index is in the array
   bool inRange(int row,
                int column) const {
      // initializes results to false
      bool results = false;

      // check if row is in the range of [min,max] inclusive
      if (row >= minRow && row <= maxRow) {
         // if in rows range call the UVector<T> of that row and check range
         results = rows[row].inRange(column);
      }

      return results;
   }

   // method to check if a given index is in the array
   bool inRange(int row) const {
      // initializes results to false
      bool results = false;

      // check if row is in the range of [min,max] inclusive
      if (row >= minRow && row <= maxRow) {
         // if in rows range then results is true
         results = true;
      }

      return results;
   }

   // overloads the array index operator
   UVector <T> &operator[](int row) {
      // check if in valid range of UMatrix
      if (inRange(row)) {
         // returns the UVector<T> of the given row
         return rows[row - minRow];
      }

      // if not in valid range throw Invalid Index Exception
      throw InvalidIndexException();
   }

   const UVector <T> &operator[](int row) const {
      // check if in valid range of UMatrix
      if (inRange(row)) {
         // returns the UVector<T> of the given row
         return rows[row - minRow];
      }

      // if not in valid range throw Invalid Index Exception
      throw InvalidIndexException();
   }

   // overloads the equals operator
   UMatrix &operator=(const UMatrix &in) {
      if (this != &in) {
         // calculates size of Matrix
         int size = in.maxRow - in.minRow + 1;

         // sets row min/max
         maxRow = in.maxRow;
         minRow = in.minRow;

         delete[] rows;

         // allocates the new UMatrix (array of UVector<T>s)
         rows = new UVector<T>[size];

         // loop to copy the values of the given UMatrix
         for (int loop = 0; loop < size; loop++) {
            rows[loop] = in.rows[loop];
         }
      }

      return *this;
   }

   /**
    * Matrix multiplication
    */
/*	UMatrix& operator*(const UMatrix &that) const {
		int rowStart1 = get_min_row();
		int rowEnd1 = get_max_row();
		int colStart1 = get_min_column();
		int colEnd1 = get_max_column();
		int rowStart2 = that.get_min_row();
		int rowEnd2 = that.get_max_row();
		int colStart2 = that.get_min_column();
		int colEnd2 = that.get_max_column();
		if (colStart1 != rowStart2 || colEnd1 != rowEnd2) {
			// cannot be multiplied because inner dimensions don't match
			throw IncompatibleDimensionsException(
					MULTIPLICATION_DIMENSIONS_MESSAGE);
		}
		UMatrix *result = new UMatrix(rowStart1, rowEnd1, colStart2, colEnd2);
		for (int i = rowStart1; i <= rowEnd1; i++) {
			for (int j = colStart2; j <= colEnd2; j++) {
				T x = 0;
				for (int k = colStart1; k <= colEnd1; k++) {
					x += (*this)[i][k] * that[k][j];
				}
				result->set(i, j, x);
			}
		}
		return *result;
	}*/

   void ascendSort() {
      //std::sort(&rows[0],&rows[maxRow-minRow+1],&rowsComparator);
      std::sort(&rows[0], &rows[maxRow - minRow + 1]);
   }

   // getters
   int get_min_row() const {
      return minRow;
   }

   int get_max_row() const {
      return maxRow;
   }

   int get_min_column() const {
      return rows[0].get_min();
   }

   int get_max_column() const {
      return rows[0].get_max();
   }

};


