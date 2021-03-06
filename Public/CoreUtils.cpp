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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <stdexcept>

#include "public/CoreUtils.h"
#include "public/Scenario.h"
#include "public/AircraftCalculations.h"
#include "public/SimulationTime.h"

using namespace std;


log4cplus::Logger CoreUtils::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("CoreUtils"));

int CoreUtils::FindNearestIndex(const double &value_to_find,
                                const vector<double> &vector_to_search) {

   int idx;
   if (value_to_find > vector_to_search.back()) {
      // Special handling that upper_bound() won't accomplish
      idx = static_cast<int>(vector_to_search.size() - 1);
   } else {
      auto itr = std::upper_bound(vector_to_search.begin(), vector_to_search.end(), value_to_find);
      idx = static_cast<int>(itr - vector_to_search.begin());
   }

   return idx;
}

double CoreUtils::LinearlyInterpolate(int upper_index,
                                      double x_interpolation_value,
                                      const std::vector<double> &x_values,
                                      const std::vector<double> &y_values) {

   if (upper_index < 1 || upper_index >= x_values.size()) {
      char msg[200];
      sprintf(msg, "upperIx (%d) is not between 1 and %d", upper_index, static_cast<int>(x_values.size() - 1));
      LOG4CPLUS_FATAL(m_logger, msg);
      throw out_of_range(msg);
   }

   double v2 = x_values[upper_index];
   double v1 = x_values[upper_index - 1];
   double o2 = y_values[upper_index];
   double o1 = y_values[upper_index - 1];

   if ((x_interpolation_value - v1) * (x_interpolation_value - v2) > 0) {
      char msg[200];
      sprintf(msg, "v (%lf) is not between %lf and %lf.", x_interpolation_value, v1, v2);

      double ratio = (x_interpolation_value-v1) / (x_interpolation_value-v2);   // must be positive
      if (upper_index + 1 == x_values.size() && (ratio < .1 || ratio > 10)) {
         // within 10%, let him off with a warning
         LOG4CPLUS_WARN(m_logger, msg);
      }
      else {
         LOG4CPLUS_FATAL(m_logger, msg);
         throw domain_error(msg);
      }
   }

   return ((o2 - o1) / (v2 - v1)) * (x_interpolation_value - v1) + o1;
}

const Units::Length CoreUtils::CalculateEuclideanDistance(const std::pair<Units::Length, Units::Length> &xyLoc1,
                                                          const std::pair<Units::Length, Units::Length> &xyLoc2) {
   Units::Length xdiff = xyLoc1.first - xyLoc2.first;
   Units::Length ydiff = xyLoc1.second - xyLoc2.second;
   const Units::Length eucldist = sqrt((xdiff * xdiff) + (ydiff * ydiff));
   return eucldist;
}

const double CoreUtils::LimitOnInterval(double value,
                                        double low_limit,
                                        double high_limit) {
   return (value < low_limit ? low_limit : (value > high_limit ? high_limit : value));
}

const int CoreUtils::SignOfValue(double value) {
   return (((value) == (0)) ? 0 : (((value) > (0)) ? (1) : (-1)));
}
