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

#include <iostream>
#include "public/DVector.h"
#include "public/InvalidIndexException.h"

using std::cout;
using std::endl;

DVector::DVector() {
   m_min_index = 0;
   m_max_index = -1;
   m_vector = NULL;
}

// NOTE: max index is inclusive
DVector::DVector(int min, int max) {

   int size = max - min + 1;

   m_min_index = min;
   m_max_index = max;

   m_vector = new double[size];
}

DVector::DVector(const DVector &in) {
   int size = in.m_max_index - in.m_min_index + 1;
   m_min_index = in.m_min_index;
   m_max_index = in.m_max_index;
   m_vector = new double[size];

   for (int loop = 0; loop < size; loop++) {
      m_vector[loop] = in.m_vector[loop];
   }
}

DVector::~DVector() {
   delete[] m_vector;
   m_vector = NULL;
}

double DVector::Get(int index) {
   if (IsIndexInRange(index)) {
      return m_vector[index - m_min_index];
   }

   throw InvalidIndexException(index, m_min_index, m_max_index);
}

void DVector::Set(int index, double value) {
   if (IsIndexInRange(index)) {
      m_vector[index - m_min_index] = value;
   } else {
      throw InvalidIndexException(index, m_min_index, m_max_index);
   }
}

// NOTE: the max index is inclusive
bool DVector::IsIndexInRange(int index) const {
   bool result = false;

   if (index >= m_min_index && index <= m_max_index) {
      result = true;
   }

   return result;
}

// NOTE: max index is inclusive
void DVector::SetBounds(int min, int max) {
   int size0 = m_max_index - m_min_index + 1;
   m_min_index = min;
   m_max_index = max;

   int size = max - min + 1;

   if (size != size0) {
      delete[] m_vector;
      m_vector = new double[size];
   }
}

double &DVector::operator[](int index) {
   if (IsIndexInRange(index)) {
      return m_vector[index - m_min_index];
   }
   throw InvalidIndexException(index, m_min_index, m_max_index);
}

const double &DVector::operator[](int index) const {
   if (IsIndexInRange(index)) {
      return m_vector[index - m_min_index];
   }
   throw InvalidIndexException(index, m_min_index, m_max_index);
}

DVector &DVector::operator=(const DVector &in) {
   if (this != &in) {
      SetBounds(in.m_min_index, in.m_max_index);
      int size = m_max_index - m_min_index + 1;

      for (int loop = 0; loop < size; loop++) {
         m_vector[loop] = in.m_vector[loop];
      }
   }

   return *this;
}

bool DVector::operator<(const DVector &other) const { return m_vector[0] < other.m_vector[0]; }

int DVector::GetMin() { return m_min_index; }

int DVector::GetMax() { return m_max_index; }
