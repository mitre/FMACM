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
#include "public/HorizontalPath.h"
#include "public/Guidance.h"
#include "public/AircraftState.h"
#include "public/AircraftIntent.h"
#include "public/PrecalcWaypoint.h"
#include "loader/Loadable.h"
#include <scalar/Angle.h>
#include <scalar/Length.h>
#include "public/AlongPathDistanceCalculator.h"
#include "public/PositionCalculator.h"
#include "public/GuidanceCalculator.h"
#include "utility/BoundedValue.h"

namespace fmacm {
class GuidanceFromStaticData final : aaesim::open_source::GuidanceCalculator {
  public:
   struct VerticalData {
      VerticalData()
         : m_time_to_go_sec(),
           m_distance_to_go_meters(),
           m_altitude_meters(),
           m_ias_mps(),
           m_vertical_speed_mps(),
           m_ground_speed_mps() {}

      std::vector<double> m_time_to_go_sec;
      std::vector<double> m_distance_to_go_meters;
      std::vector<double> m_altitude_meters;
      std::vector<double> m_ias_mps;
      std::vector<double> m_vertical_speed_mps;
      std::vector<double> m_ground_speed_mps;
   };
   struct PlannedDescentParameters {
      PlannedDescentParameters()
         : planned_transition_altitude(Units::Infinity()),
           planned_transition_ias(Units::NegInfinity()),
           planned_cruise_mach(0) {}
      Units::Length planned_transition_altitude;
      Units::Speed planned_transition_ias;
      double planned_cruise_mach;
   };

   GuidanceFromStaticData();

   GuidanceFromStaticData(const std::vector<HorizontalPath> &horizontal_path, const VerticalData &vertical_path,
                          const PlannedDescentParameters &planned_descent_parameters);

   aaesim::open_source::Guidance Update(const aaesim::open_source::AircraftState &state) override;

   const VerticalData &GetVerticalData() const;

   const Units::MetersLength GetEstimatedDistanceAlongPath() const;

   const std::vector<HorizontalPath> &GetHorizontalTrajectory() const;

  private:
   static log4cplus::Logger m_logger;
   enum TurnDirection { LEFT, RIGHT };

   TurnDirection GetTurnDirection(const Units::Angle course_change);
   aaesim::open_source::Guidance CalculateVerticalGuidance(const aaesim::open_source::AircraftState &state,
                                                           const Units::MetersLength &estimated_distance_to_go,
                                                           const Units::UnsignedAngle &estimated_course);
   aaesim::open_source::Guidance CalculateHorizontalGuidance(const aaesim::open_source::AircraftState &state,
                                                             const Units::MetersLength &estimated_distance_to_go,
                                                             const Units::UnsignedAngle &estimated_course);

   VerticalData m_vertical_data;
   std::vector<HorizontalPath> m_horizontal_trajectory;
   AlongPathDistanceCalculator m_decrementing_distance_calculator;
   PositionCalculator m_decrementing_position_calculator;
   Units::Length m_estimated_distance_to_go;
   aaesim::open_source::Guidance m_previous_guidance;
   PlannedDescentParameters m_planned_descent_parameters;
};

inline const Units::MetersLength GuidanceFromStaticData::GetEstimatedDistanceAlongPath() const {
   return m_estimated_distance_to_go;
}

inline const std::vector<HorizontalPath> &GuidanceFromStaticData::GetHorizontalTrajectory() const {
   return m_horizontal_trajectory;
}

inline const GuidanceFromStaticData::VerticalData &GuidanceFromStaticData::GetVerticalData() const {
   return m_vertical_data;
}

inline GuidanceFromStaticData::TurnDirection GuidanceFromStaticData::GetTurnDirection(
      const Units::Angle course_change) {
   if (course_change > Units::ZERO_ANGLE) {
      return LEFT;
   } else {
      return RIGHT;
   }
}
}  // namespace fmacm