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

#pragma once

#include <vector>
#include <scalar/Time.h>
#include <scalar/Length.h>

#include "public/Logging.h"
#include "public/AircraftState.h"
#include "public/HorizontalPath.h"
#include "public/AircraftIntent.h"
#include "public/LineOnEllipsoid.h"

class CoreUtils {
  public:
   static const std::string INTERMEDIATE_WAYPOINT_ROOT_NAME;

   /**
    * Find the index of a value in a vector. Uses STL upper_bound(), but with one modified return.
    *
    * @param value_to_find: value to search for
    * @param vector_to_search: vector to be searched
    * @return upper index that bounds. Will never grow larger than size()-1.
    */
   static int FindNearestIndex(const double &value_to_find, const std::vector<double> &vector_to_search);

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
   static double LinearlyInterpolate(int upper_index, double x_interpolation_value, const std::vector<double> &x_values,
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
   static const double LimitOnInterval(double value, double low_limit = std::numeric_limits<double>::min(),
                                       double high_limit = std::numeric_limits<double>::max());

   /**
    * @param value
    * @return 0 for zero input value, -1 for negative values, 1 for positive values
    */
   static const int SignOfValue(double value);

   /**
    *
    * @param ordered_waypoints
    * @param maximum_allowable_length, default is CoreUtils::MAXIMUM_ALLOWABLE_SINGLE_LEG_LENGTH
    * @see CoreUtils::MAXIMUM_ALLOWABLE_SINGLE_LEG_LENGTH
    * @return
    */
   static std::list<Waypoint> ShortenLongLegs(
         const std::list<Waypoint> &ordered_waypoints,
         Units::Length maximum_allowable_length = MAXIMUM_ALLOWABLE_SINGLE_LEG_LENGTH);

   /**
    * Visible for testing.
    *
    * Update the static parameter value.
    *
    * @param new_value
    */
   static void UpdateMaximumAllowableSingleLegLength(Units::Length new_value);

   /**
    * Visible for testing.
    */
   static void ResetMaximumAllowableSingleLegLength();

   /**
    * @brief Find out if ptr is of type typename.
    *
    * @tparam Base
    * @tparam T
    * @param ptr
    * @return true
    * @return false
    */
   template <typename Base, typename T>
   inline static bool InstanceOf(const T *ptr) {
      return dynamic_cast<const Base *>(ptr) != nullptr;
   }

  private:
   static log4cplus::Logger m_logger;
   static Units::NauticalMilesLength MAXIMUM_ALLOWABLE_SINGLE_LEG_LENGTH;

   static std::list<Waypoint> GetIntermediateWaypointsForLongLeg(const aaesim::LineOnEllipsoid &line_on_ellipsoid,
                                                                 Units::Length maximum_allowable_single_leg_distance);
};

inline void CoreUtils::UpdateMaximumAllowableSingleLegLength(Units::Length new_value) {
   MAXIMUM_ALLOWABLE_SINGLE_LEG_LENGTH = new_value;
}

inline void CoreUtils::ResetMaximumAllowableSingleLegLength() {
   MAXIMUM_ALLOWABLE_SINGLE_LEG_LENGTH = Units::NauticalMilesLength(10);
}