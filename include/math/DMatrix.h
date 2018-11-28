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

#pragma once

#include "math/DVector.h"
#include <exception>

class DMatrix
{
public:
   class IncompatibleDimensionsException : public std::exception
   {
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

   DMatrix(double **array_in,
           int row_min,
           int row_max,
           int column_min,
           int column_max);
   
   DMatrix(int row_min,
           int row_max,
           int column_min,
           int column_max);

   double Get(const int row,
              const int column) const;

   void Set(const int row,
            const int column,
            const double value);

   void AscendSort();
   
   void SetBounds(int row_min,
                  int row_max,
                  int column_min,
                  int column_max);
   
   bool InRange(const int row,
                const int colomn) const;

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
