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

#pragma once

#include "math/DVector.h"
#include <exception>

class DMatrix {
  public:
   class IncompatibleDimensionsException : public std::exception {
     public:
      IncompatibleDimensionsException(char *explanation);

      virtual ~IncompatibleDimensionsException() throw();

      virtual const char *what() const throw();

     private:
      const char *m_explanation;
   };

   DMatrix();

   virtual ~DMatrix();

   DMatrix(const DMatrix &in);

   DMatrix(double **array_in, int row_min, int row_max, int column_min, int column_max);

   DMatrix(int row_min, int row_max, int column_min, int column_max);

   double Get(const int row, const int column) const;

   void Set(const int row, const int column, const double value);

   void AscendSort();

   void SetBounds(int row_min, int row_max, int column_min, int column_max);

   bool InRange(const int row, const int colomn) const;

   bool InRange(const int row) const;

   DVector &operator[](const int);

   const DVector &operator[](const int) const;

   DMatrix &operator=(const DMatrix &in);

   DMatrix &operator*(const DMatrix &that) const;

   int GetMinRow() const;

   int GetMaxRow() const;

   int GetMinColumn() const;

   int GetMaxColumn() const;

  private:
   static char *MULTIPLICATION_DIMENSIONS_MESSAGE;
   DVector *m_rows;
   int m_min_row;
   int m_max_row;
};
