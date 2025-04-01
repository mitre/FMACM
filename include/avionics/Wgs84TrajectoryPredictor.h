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
#include <scalar/Angle.h>
#include <nlohmann/json.hpp>
#include "public/PositionCalculator.h"
#include "public/AlongPathDistanceCalculator.h"
#include "public/AircraftIntent.h"
#include "public/VerticalPath.h"
#include "public/Wgs84PrecalcWaypoint.h"
#include "public/WeatherPrediction.h"
#include "avionics/Wgs84HorizontalPathSegment.h"

namespace aaesim {

struct Wgs84TrajectoryPredictor {

   enum TrajectoryPassOption { FIRST_PASS, SECOND_PASS };

   Wgs84TrajectoryPredictor() = default;

   ~Wgs84TrajectoryPredictor() = default;

   virtual const AircraftIntent &GetAircraftIntent() const = 0;

   virtual const std::vector<Wgs84HorizontalPathSegment> &GetHorizontalPath() const = 0;

   virtual const std::vector<Wgs84PrecalcWaypoint> &GetPrecalcWaypoints() const = 0;

   virtual const VerticalPath GetVerticalPath() const = 0;

   virtual void CalculateWaypoints(const AircraftIntent &aircraft_intent,
                                   const aaesim::open_source::WeatherPrediction &weather_prediction) = 0;

   virtual void BuildTrajectoryPrediction(const std::string &aircraft_peformance_name,
                                          aaesim::open_source::WeatherPrediction &weather,
                                          Units::Length start_altitude) = 0;

   void DoHorizontalPathLogging(log4cplus::Logger &logger, TrajectoryPassOption current_option) const {
      using json = nlohmann::json;
      if (logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
         auto idx = 0;
         for (Wgs84HorizontalPathSegment segment : GetHorizontalPath()) {
            json j;
            j["segment_index"] = idx;
            j["trajectory_option"] = GetTrajectoryOptionAsString(current_option);
            j["segment_type"] = segment.GetSegmentTypeAsString();
            j["cumulative_path_length_nm"] =
                  Units::NauticalMilesLength(segment.GetCumulativePathLengthIncludingThisShape()).value();
            j["start_lat_deg"] =
                  Units::DegreesAngle(segment.GetShapeOnEllipsoid()->GetStartPoint().GetLatitude()).value();
            j["start_lon_deg"] =
                  Units::DegreesAngle(segment.GetShapeOnEllipsoid()->GetStartPoint().GetLongitude()).value();
            j["end_lat_deg"] = Units::DegreesAngle(segment.GetShapeOnEllipsoid()->GetEndPoint().GetLatitude()).value();
            j["end_lon_deg"] = Units::DegreesAngle(segment.GetShapeOnEllipsoid()->GetEndPoint().GetLongitude()).value();
            LOG4CPLUS_TRACE(logger, j.dump());
            ++idx;
         }
      }
   }

   void DoVerticalPathLogging(log4cplus::Logger &logger, TrajectoryPassOption current_option) const {
      using json = nlohmann::json;
      VerticalPath vertical_path = GetVerticalPath();
      const bool too_short(vertical_path.along_path_distance_m.size() <= 4);
      if (too_short || logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
         json j;
         j["trajectory_option"] = GetTrajectoryOptionAsString(current_option);
         j["time_to_go_sec"] = vertical_path.time_to_go_sec;
         j["along_path_distance_m"] = vertical_path.along_path_distance_m;
         j["altitude_m"] = vertical_path.altitude_m;
         j["cas_mps"] = vertical_path.cas_mps;
         j["mach_number"] = vertical_path.mach;
         j["altitude_rate_mps"] = vertical_path.altitude_rate_mps;
         j["tas_rate_mps"] = vertical_path.tas_rate_mps;
         j["theta_radians"] = vertical_path.theta_radians;
         j["gs_mps"] = vertical_path.gs_mps;
         j["algorithm_enum"] = vertical_path.algorithm_type;
         if (too_short) {
            LOG4CPLUS_ERROR(logger, "Vertical path does not contain enough elements.");
            LOG4CPLUS_ERROR(logger, j.dump());  // log contents at ERROR level
         } else {
            LOG4CPLUS_TRACE(logger, j.dump());
         }
      }
   }

   void DoWaypointDataLogging(log4cplus::Logger &logger) const {
      using json = nlohmann::json;

      if (logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
         for (const auto &waypoint : GetPrecalcWaypoints()) {
            json j;
            j["name"] = waypoint.m_name;
            j["leg_type"] = static_cast<int>(waypoint.m_leg_type);
            j["leg_length"] = Units::MetersLength(waypoint.m_leg_length).value();
            j["enu_course_out_angle"] = waypoint.m_enu_course_out_angle.value();
            j["enu_course_in_angle"] = waypoint.m_enu_course_in_angle.value();
            j["latitude_deg"] = Units::DegreesAngle(waypoint.m_position.GetLatitude()).value();
            j["longitude_deg"] = Units::DegreesAngle(waypoint.m_position.GetLongitude()).value();
            j["rf_leg_center_latitude_deg"] = Units::DegreesAngle(waypoint.m_rf_leg_center.GetLatitude()).value();
            j["rf_leg_center_longitude_deg"] = Units::DegreesAngle(waypoint.m_rf_leg_center.GetLongitude()).value();
            j["radius_rf_leg"] = waypoint.m_radius_rf_leg.value();
            j["bank_angle"] = waypoint.m_bank_angle.value();
            j["ground_speed"] = Units::MetersPerSecondSpeed(waypoint.m_ground_speed).value();
            j["constraint_along_path_distance"] =
                  Units::MetersLength(waypoint.m_precalc_constraints.constraint_along_path_distance).value();
            j["constraint_altHi"] = Units::MetersLength(waypoint.m_precalc_constraints.constraint_altHi).value();
            j["constraint_altLow"] = Units::MetersLength(waypoint.m_precalc_constraints.constraint_altLow).value();
            j["constraint_speedHi"] =
                  Units::MetersPerSecondSpeed(waypoint.m_precalc_constraints.constraint_speedHi).value();
            j["constraint_speedLow"] =
                  Units::MetersPerSecondSpeed(waypoint.m_precalc_constraints.constraint_speedLow).value();
            j["index"] = waypoint.m_precalc_constraints.index;
            j["active_flag"] = static_cast<int>(waypoint.m_precalc_constraints.active_flag);
            j["violation_flag"] = waypoint.m_precalc_constraints.violation_flag;

            LOG4CPLUS_TRACE(logger, j.dump());
         }
      }
   }

   void DoTightTurnLogging(log4cplus::Logger &logger, const std::string &msg,
                           const TrajectoryPassOption &current_option, const Wgs84PrecalcWaypoint &incoming_waypoint,
                           const Wgs84PrecalcWaypoint &outgoing_waypoint, const aaesim::LineOnEllipsoid &first_bisector,
                           const aaesim::LineOnEllipsoid &second_bisector,
                           const LatitudeLongitudePoint &intersection) const {
      using json = nlohmann::json;
      if (logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
         json j;
         j["logger_msg"] = msg;
         j["trajectory_option"] = GetTrajectoryOptionAsString(current_option);
         j["first_waypoint"] = incoming_waypoint.m_name;
         j["first_waypoint_lat_deg"] = Units::DegreesAngle(incoming_waypoint.m_position.GetLatitude()).value();
         j["first_waypoint_lon_deg"] = Units::DegreesAngle(incoming_waypoint.m_position.GetLongitude()).value();
         j["second_waypoint"] = outgoing_waypoint.m_name;
         j["second_waypoint_lat_deg"] = Units::DegreesAngle(outgoing_waypoint.m_position.GetLatitude()).value();
         j["second_waypoint_lon_deg"] = Units::DegreesAngle(outgoing_waypoint.m_position.GetLongitude()).value();
         j["bisector1.start_lat_deg"] = Units::DegreesAngle(first_bisector.GetStartPoint().GetLatitude()).value();
         j["bisector1.start_lon_deg"] = Units::DegreesAngle(first_bisector.GetStartPoint().GetLongitude()).value();
         j["bisector1.end_lat_deg"] = Units::DegreesAngle(first_bisector.GetEndPoint().GetLatitude()).value();
         j["bisector1.end_lon_deg"] = Units::DegreesAngle(first_bisector.GetEndPoint().GetLongitude()).value();
         j["bisector2.start_lat_deg"] = Units::DegreesAngle(second_bisector.GetStartPoint().GetLatitude()).value();
         j["bisector2.start_lon_deg"] = Units::DegreesAngle(second_bisector.GetStartPoint().GetLongitude()).value();
         j["bisector2.end_lat_deg"] = Units::DegreesAngle(second_bisector.GetEndPoint().GetLatitude()).value();
         j["bisector2.end_lon_deg"] = Units::DegreesAngle(second_bisector.GetEndPoint().GetLongitude()).value();
         j["intersection.lat_deg"] = Units::DegreesAngle(intersection.GetLatitude()).value();
         j["intersection.lon_deg"] = Units::DegreesAngle(intersection.GetLongitude()).value();
         LOG4CPLUS_TRACE(logger, j.dump());
      }
   }

   std::string GetTrajectoryOptionAsString(TrajectoryPassOption option) const {
      std::string option_as_string = "FIRST_PASS";
      if (option == TrajectoryPassOption::SECOND_PASS) {
         option_as_string = "SECOND_PASS";
      }
      return option_as_string;
   }
};
}  // namespace aaesim
