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

#include "math/DMatrix.h"
#include "math/InvalidIndexException.h"
#include <algorithm>

char *DMatrix::MULTIPLICATION_DIMENSIONS_MESSAGE = (char *)"Cannot multiply DMatrix unless inner dimensions match.";

DMatrix::DMatrix() {
   m_min_row = 0;
   m_max_row = -1;
   m_rows = NULL;
}

DMatrix::~DMatrix() {
   delete[] m_rows;
   m_rows = NULL;
}

DMatrix::DMatrix(const DMatrix &in) {
   int size = in.m_max_row - in.m_min_row + 1;

   m_max_row = in.m_max_row;
   m_min_row = in.m_min_row;

   m_rows = new DVector[size];

   for (int loop = 0; loop < size; loop++) {
      m_rows[loop] = in.m_rows[loop];
   }
}

DMatrix::DMatrix(int inRowMin, int inRowMax, int inColMin, int inColMax) {
   // calculates the size of the DMatrix
   int size = inRowMax - inRowMin + 1;

   // sets row min/max
   m_max_row = inRowMax;
   m_min_row = inRowMin;

   // allocates the new DMatrix (array of DVectors)
   m_rows = new DVector[size];

   // loop to set the colomn size (DVector min/max)
   for (int loop = 0; loop < size; loop++) {
      m_rows[loop].SetBounds(inColMin, inColMax);
   }
}

DMatrix::DMatrix(double **array_in, int inRowMin, int inRowMax, int inColMin, int inColMax) {
   // calculates the size of the DMatrix
   int size = inRowMax - inRowMin + 1;

   // sets row min/max
   m_max_row = inRowMax;
   m_min_row = inRowMin;

   // allocates the new DMatrix (array of DVectors)
   m_rows = new DVector[size];

   // loop to set the colomn size (DVector min/max)
   for (int loop = 0; loop < size; loop++) {
      m_rows[loop].SetBounds(inColMin, inColMax);
   }

   // NOTE this is dangerous code, the array MUST be the same size as the given row/colomn information
   // failure to match the bounds given will cause dangerous memory access!!!
   // this only works because 2d arrays are stored in sequential memory addresses
   // if the array is dynamically allocated this WILL cause dangerous memory access!!!
   double *memory_pointer;               // pointer to point to the memory address of the 2d array
   memory_pointer = (double *)array_in;  // accesses the the memory address of the first element
   for (int outer = 0; outer < size; outer++) {
      for (int inner = inColMin; inner <= inColMax; inner++) {
         m_rows[outer].Set(inner, (*memory_pointer));
         memory_pointer++;  // iterates the memory address
      }
   }
}

double DMatrix::Get(const int row, const int column) const {
   // check if in valid range of DMatrix
   if (InRange(row)) {
      // uses the DVectors overloaded array operator to get the value
      return m_rows[row - m_min_row][column];
   }

   // if not in valid range throw Invalid Index Exception
   throw InvalidIndexException(row, m_min_row, m_max_row);
}

void DMatrix::Set(const int row, const int column, const double value) {
   // check if in valid range of DMatrix
   if (InRange(row)) {
      // uses the DVector overloaded array operator to set the value
      m_rows[row - m_min_row][column] = value;
   } else {
      // else not in range throw Invalid Index Exception
      throw InvalidIndexException(row, m_min_row, m_max_row);
   }
}

void DMatrix::SetBounds(int row_min, int row_max, int column_min, int column_max) {
   // calculates the size of the DMatrix
   int size = row_max - row_min + 1;

   // sets row min/max
   m_max_row = row_max;
   m_min_row = row_min;

   // allocates the new DMatrix (array of DVectors)
   delete[] m_rows;
   m_rows = new DVector[size];

   // loop to set the colomn size (DVector min/max)
   for (int loop = 0; loop < size; loop++) {
      m_rows[loop].SetBounds(column_min, column_max);
   }
}

bool DMatrix::InRange(const int row, const int colomn) const {
   // initializes results to false
   bool results = false;

   // check if row is in the range of [min,max] inclusive
   if (row >= m_min_row && row <= m_max_row) {
      // if in rows range call the DVector of that row and check range
      results = m_rows[row].IsIndexInRange(colomn);
   }

   return results;
}

bool DMatrix::InRange(const int row) const {
   // initializes results to false
   bool results = false;

   // check if row is in the range of [min,max] inclusive
   if (row >= m_min_row && row <= m_max_row) {
      // if in rows range then results is true
      results = true;
   }

   return results;
}

DVector &DMatrix::operator[](int row) {
   // check if in valid range of DMatrix
   if (InRange(row)) {
      // returns the DVector of the given row
      return m_rows[row - m_min_row];
   }

   // if not in valid range throw Invalid Index Exception
   throw InvalidIndexException(row, m_min_row, m_max_row);
}

const DVector &DMatrix::operator[](int row) const {
   // check if in valid range of DMatrix
   if (InRange(row)) {
      // returns the DVector of the given row
      return m_rows[row - m_min_row];
   }

   // if not in valid range throw Invalid Index Exception
   throw InvalidIndexException(row, m_min_row, m_max_row);
}

DMatrix &DMatrix::operator=(const DMatrix &in) {
   if (this != &in) {
      // calculates size of Matrix
      int size = in.m_max_row - in.m_min_row + 1;

      // sets row min/max
      m_max_row = in.m_max_row;
      m_min_row = in.m_min_row;

      delete[] m_rows;

      // allocates the new DMatrix (array of DVectors)
      m_rows = new DVector[size];

      // loop to copy the values of the given DMatrix
      for (int loop = 0; loop < size; loop++) {
         m_rows[loop] = in.m_rows[loop];
      }
   }

   return *this;
}

DMatrix &DMatrix::operator*(const DMatrix &that) const {
   int rowStart1 = GetMinRow();
   int rowEnd1 = GetMaxRow();
   int colStart1 = GetMinColumn();
   int colEnd1 = GetMaxColumn();
   int rowStart2 = that.GetMinRow();
   int rowEnd2 = that.GetMaxRow();
   int colStart2 = that.GetMinColumn();
   int colEnd2 = that.GetMaxColumn();
   if (colStart1 != rowStart2 || colEnd1 != rowEnd2) {
      // cannot be multiplied because inner dimensions don't match
      throw IncompatibleDimensionsException(MULTIPLICATION_DIMENSIONS_MESSAGE);
   }
   DMatrix *result = new DMatrix(rowStart1, rowEnd1, colStart2, colEnd2);
   for (int i = rowStart1; i <= rowEnd1; i++) {
      for (int j = colStart2; j <= colEnd2; j++) {
         double x = 0;
         for (int k = colStart1; k <= colEnd1; k++) {
            x += (*this)[i][k] * that[k][j];
         }
         result->Set(i, j, x);
      }
   }
   return *result;
}

void DMatrix::AscendSort() { std::sort(&m_rows[0], &m_rows[m_max_row - m_min_row + 1]); }

int DMatrix::GetMinRow() const { return m_min_row; }

int DMatrix::GetMaxRow() const { return m_max_row; }

int DMatrix::GetMinColumn() const { return m_rows[0].GetMin(); }

int DMatrix::GetMaxColumn() const { return m_rows[0].GetMax(); }

DMatrix::IncompatibleDimensionsException::IncompatibleDimensionsException(char *explanation)
   : exception(), m_explanation(explanation) {}

DMatrix::IncompatibleDimensionsException::~IncompatibleDimensionsException() throw() {}

const char *DMatrix::IncompatibleDimensionsException::what() const throw() { return m_explanation; }
