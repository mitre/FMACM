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
