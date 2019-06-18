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
#include "public/HorizontalPath.h"
#include "public/Guidance.h"
#include "public/AircraftState.h"
#include "public/AircraftIntent.h"
#include "public/PrecalcWaypoint.h"
#include "loader/Loadable.h"
#include <Angle.h>
#include <Length.h>
#include <public/AlongPathDistanceCalculator.h>
#include <public/PositionCalculator.h>


class TrajectoryFromFile : Loadable
{

public:

   struct VerticalData
   {
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
   } m_vertical_data;

   TrajectoryFromFile();

   virtual ~TrajectoryFromFile();

   bool load(DecodedStream *input);

   VerticalData GetVerticalData();

   Guidance Update(const AircraftState &state,
                   const Guidance &guidance_in);

   void CalculateWaypoints(AircraftIntent &intent);


   const Units::MetersLength GetEstimatedDistanceAlongPath() const;

   AlongPathDistanceCalculator &GetDecrementingDistanceCalculator();

   const std::vector<HorizontalPath> &GetHorizontalTrajectory() const;

   double GetMassPercentile() const;

   const std::vector<PrecalcWaypoint> &GetPrecalcWaypoint() const;

   const bool IsLoaded() const;

private:
   enum VerticalFields
   {
      TIME_TO_GO_SEC = 0,
      DISTANCE_TO_GO_VERT_M,
      ALTITUDE_M,
      IAS_MPS,
      DOT_ALTITUDE_MPS,
      GS_MPS,
      NUM_VERTICAL_TRAJ_FIELDS
   };

   enum HorizontalFields
   {
      IX = 0,
      X_M,
      Y_M,
      DISTANCE_TO_GO_HORZ_M,
      SEGMENT_TYPE,
      COURSE_R,
      TURN_CENTER_X_M,
      TURN_CENTER_Y_M,
      ANGLE_AT_TURN_START_R,
      ANGLE_AT_TURN_END_R,
      TURN_RADIUS_M,
      GROUND_SPEED_MPS,
      BANK_ANGLE_DEG,
      LAT_D,
      LON_D,
      TURN_CENTER_LAT_D,
      TURN_CENTER_LON_D,
      NUM_HORIZONTAL_TRAJ_FIELDS
   };

   enum TurnDirection {
      LEFT,
      RIGHT
   };

   TurnDirection GetTurnDirection(const Units::Angle course_change);

   void ReadVerticalTrajectoryFile();

   void ReadHorizontalTrajectoryFile();

   std::vector<HorizontalPath> m_horizontal_trajectory;
   std::vector<PrecalcWaypoint> m_precalc_waypoints;

   AlongPathDistanceCalculator m_decrementing_distance_calculator;
   PositionCalculator m_decrementing_position_calculator;

   Units::MetersLength estimated_distance_to_go;

   std::string m_vertical_trajectory_file;
   std::string m_horizontal_trajectory_file;

   double m_mass_percentile;

   bool m_loaded;
};

inline const Units::MetersLength TrajectoryFromFile::GetEstimatedDistanceAlongPath() const {
   return estimated_distance_to_go;
}

inline AlongPathDistanceCalculator &TrajectoryFromFile::GetDecrementingDistanceCalculator() {
   return m_decrementing_distance_calculator;
}

inline const std::vector<HorizontalPath> &TrajectoryFromFile::GetHorizontalTrajectory() const {
   return m_horizontal_trajectory;
}

inline double TrajectoryFromFile::GetMassPercentile() const {
   return m_mass_percentile;
}

inline const std::vector<PrecalcWaypoint> &TrajectoryFromFile::GetPrecalcWaypoint() const {
   return m_precalc_waypoints;
}

inline const bool TrajectoryFromFile::IsLoaded() const {
   return m_loaded;
}

inline TrajectoryFromFile::VerticalData TrajectoryFromFile::GetVerticalData() {
   return m_vertical_data;
}

inline TrajectoryFromFile::TurnDirection TrajectoryFromFile::GetTurnDirection(const Units::Angle course_change) {
   if (course_change > Units::SignedRadiansAngle(0.0)) {
      return LEFT;
   } else {
      return RIGHT;
   }
}