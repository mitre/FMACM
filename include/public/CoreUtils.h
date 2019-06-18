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
#include <Time.h>
#include <Length.h>

#include "utility/Logging.h"
#include "public/AircraftState.h"
#include "public/HorizontalPath.h"

class CoreUtils
{
public:

   /**
    * Find the index of a value in a vector. Uses STL upper_bound(), but with one modified return.
    *
    * @param value_to_find: value to search for
    * @param vector_to_search: vector to be searched
    * @return upper index that bounds. Will never grow larger than size()-1.
    */
   static int FindNearestIndex(const double &value_to_find,
                               const std::vector<double> &vector_to_search);

   /**
    * Linearly interpolate between items in vector. Caller must have already calculated where the interpolation
    * should start from.
    *
    * @param upper_index upper index for start of interpolation, see CoreUtils::FindNearestIndex()
    * @param x_interpolation_value value to interpolate to
    * @param x_values
    * @param y_values
    * @throws if upper_index is not within the range of value_vector
    * @return a y-value the linearly corresponds to x_interpolation_value
    */
   static double LinearlyInterpolate(int upper_index,
                                     double x_interpolation_value,
                                     const std::vector<double> &x_values,
                                     const std::vector<double> &y_values);

   /**
    * @param xyLoc1: first x,y pair
    * @param xyLoc2  second x,y pair
    * @return The Euclidean straight line distance between the two points.
    */
   static const Units::Length CalculateEuclideanDistance(const std::pair<Units::Length, Units::Length> &xyLoc1,
                                                         const std::pair<Units::Length, Units::Length> &xyLoc2);

   /**
    * Limit value on the exclusive range of (low_limit, high_limit).
    *
    * @param value
    * @param low_limit default is double min
    * @param high_limit default is double max
    * @return the limited value
    */
   static const double LimitOnInterval(double value,
                                       double low_limit = std::numeric_limits<double>::min(),
                                       double high_limit = std::numeric_limits<double>::max());

   /**
    * @param value
    * @return 0 for zero input value, -1 for negative values, 1 for positive values
    */
   static const int SignOfValue(double value);

private:

   static log4cplus::Logger m_logger;
};


