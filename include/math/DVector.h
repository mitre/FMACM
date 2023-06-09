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

#ifndef DVECTOR_H
#define DVECTOR_H

class DVector {
  public:
   DVector();

   DVector(int min, int max);

   DVector(const DVector &in);

   virtual ~DVector();

   double Get(int index);

   void Set(int index, double value);

   void SetBounds(int min, int max);

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
