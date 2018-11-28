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
#include "math/Statistics.h"
#include <math.h>
#include <algorithm>


Statistics::Statistics() {
   m_sum_of_samples = 0;
   m_max = 0.0;
   m_min = 0.0;
}

Statistics::~Statistics() {

}

void Statistics::Insert(double value) {
   if (m_samples.size() == 0) {
      m_max = value;
      m_min = value;
   } else {
      m_max = std::max(m_max, value);
      m_min = std::min(m_min, value);
   }

   m_samples.push_back(value);

   m_sum_of_samples += value;
}

double Statistics::ComputeStandardDeviation() const {
   double sDev = -1.0;

   if (m_samples.size() > 0) {

      double variance_sum = 0.0;
      for (int loop = 0; loop < (int) m_samples.size(); loop++) {
         if (loop != 0) {
            variance_sum += pow(m_samples[loop] - GetMean(), 2);
         } else {
            variance_sum = pow(m_samples[loop] - GetMean(), 2);
         }
      }
      sDev = sqrt(variance_sum / m_samples.size());
   }

   return sDev;
}

double Statistics::GetPercentile(double percentage) {
   int desired_sample_num;
   double result;
   vector<double> samples_sorted;

   for (unsigned int i = 0; i < m_samples.size(); i++) {
      double tmp = m_samples.at(i);
      samples_sorted.push_back(tmp);
   }

   stable_sort(samples_sorted.begin(), samples_sorted.end());

   desired_sample_num = (int) (percentage * m_samples.size());

   if (desired_sample_num == 0)
   {
      result = samples_sorted.at(0);
   } else {
      result = samples_sorted.at(desired_sample_num - 1);
   }

   return (result);

}

double Statistics::Get95thBounds() {
   double bound95 = -1.0;

   double n = (double) m_samples.size();
   double prob = 0.0;
   double delta = 0.1;

   while (prob < 0.95) {
      double size = 0.0;

      for (unsigned int ix = 0; ix < m_samples.size(); ix++) {
         if (fabs(m_samples[ix]) < delta) {
            size = size + 1.0;
         }
      }

      prob = size / n;

      bound95 = delta;
      delta = delta + 0.1;
   }

   return bound95;
}
