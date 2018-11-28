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

#ifndef DVECTOR_H
#define DVECTOR_H

class DVector
{
public:
   DVector();

   DVector(int min,
           int max);

   DVector(const DVector &in);

   virtual ~DVector();
   
   double Get(int index);

   void Set(int index,
            double value);
   
   void SetBounds(int min,
                  int max);
   
   bool IsIndexInRange(int index) const;
   
   int GetMin();

   int GetMax();
   
   double &operator[](int);

   const double &operator[](int) const;
   
   DVector &operator=(const DVector &in);
   
   bool operator<(const DVector &other) const;


private:
   int m_min_index;
   int m_max_index;
   double *m_vector;
};
#endif
