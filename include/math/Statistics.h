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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <vector>

using std::vector;

class Statistics
{
public:
   Statistics();

   virtual ~Statistics();

   virtual void Insert(double value);

   double ComputeStandardDeviation() const;

   inline double GetMean() const {
      return m_sum_of_samples / m_samples.size();
   }

   inline long GetNumberOfSamples() const {
      return m_samples.size();
   }

   inline double GetMax() const {
      return m_max;
   }

   inline double GetMin() const {
      return m_min;
   }

   double GetPercentile(double percentage);

   double Get95thBounds();

private:
   double m_sum_of_samples;
   vector<double> m_samples;
   double m_max;
   double m_min;
};
