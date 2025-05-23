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
#include "public/WeatherPrediction.h"
#include "public/VerticalPredictor.h"
#include "public/HorizontalPath.h"
#include "public/PrecalcWaypoint.h"
#include "public/AircraftIntent.h"
#include "public/PositionCalculator.h"
#include "public/AlongPathDistanceCalculator.h"
#include "public/TurnAnticipation.h"
#include "public/EuclideanTightTurnResolver.h"

namespace aaesim {
namespace open_source {

class EuclideanTrajectoryPredictor {
  public:
   EuclideanTrajectoryPredictor();

   virtual ~EuclideanTrajectoryPredictor() = default;

   virtual void CalculateWaypoints(const AircraftIntent &aircraft_intent,
                                   const aaesim::open_source::WeatherPrediction &weather_prediction);

   const std::vector<HorizontalPath> EstimateHorizontalTrajectory(
         const aaesim::open_source::WeatherPrediction &weather_prediction);

   virtual void BuildTrajectoryPrediction(aaesim::open_source::WeatherPrediction &weather,
                                          const std::shared_ptr<TangentPlaneSequence> &position_converter,
                                          Units::Length start_altitude);

   virtual void BuildTrajectoryPrediction(aaesim::open_source::WeatherPrediction &weather,
                                          const std::shared_ptr<TangentPlaneSequence> &position_converter,
                                          Units::Length start_altitude, Units::Length aircraft_distance_to_go);

   aaesim::open_source::Guidance Update(const aaesim::open_source::AircraftState &state,
                                        const aaesim::open_source::Guidance &current_guidance);

   const AircraftIntent &GetAircraftIntent() const;

   const std::vector<HorizontalPath> &GetHorizontalPath() const;

   const std::shared_ptr<Atmosphere> GetAtmosphere() const;

   const std::vector<PrecalcWaypoint> &GetPrecalcWaypoints() const;

   const std::shared_ptr<VerticalPredictor> &GetVerticalPredictor() const;

   void SetBankAngle(Units::Angle bank_angle);

   Units::Angle GetBankAngle() const;

   Units::Length GetAltitudeAtFinalWaypoint() const;

   static double CounterClockwise(const double ax, const double ay, const double bx, const double by, const double cx,
                                  const double cy);

   static double CounterClockwise(const double ax, const double ay, const double bx, const double by);

   static bool SamePoint(const PrecalcWaypoint &wp, const HorizontalPath &hp);

   static double FindCenterPoint(double fromX, double fromY, double toX, double toY, double gcpX, double gcpY,
                                 double radius, double &cpX, double &cpY);

   static double HalfTurn(HorizontalTurnPath turn);

  protected:
   void UpdateWeatherPrediction(aaesim::open_source::WeatherPrediction &weather,
                                const std::shared_ptr<TangentPlaneSequence> &position_converter) const;

   /**
    * Forces altitude constraints to form a monotonic space.
    */
   virtual void AdjustConstraints(const Units::Speed start_speed);

   std::vector<PrecalcWaypoint> m_waypoint_vector{};
   std::vector<HorizontalPath> m_horizontal_path{};
   std::shared_ptr<VerticalPredictor> m_vertical_predictor{};
   Units::Angle m_bank_angle{Units::DUMMY_DEGREES_ANGLE};
   Units::Length m_altitude_at_final_waypoint{Units::FeetLength(-50.0)};
   Units::Length m_aircraft_distance_to_go{Units::infinity()};
   AircraftIntent m_aircraft_intent{};
   AlongPathDistanceCalculator m_distance_calculator{};
   PositionCalculator m_position_calculator{};
   std::shared_ptr<Atmosphere> m_atmosphere{};
   std::shared_ptr<aaesim::open_source::EuclideanTightTurnResolver> m_tight_turn_resolver{};

  private:
   static log4cplus::Logger m_logger;
   enum HorizontalTrajOption { FIRST_PASS, SECOND_PASS };

   std::string GetTrajectoryOptionAsString(HorizontalTrajOption option) const {
      std::string option_as_string = "FIRST_PASS";
      if (option == HorizontalTrajOption::SECOND_PASS) {
         option_as_string = "SECOND_PASS";
      }
      return option_as_string;
   }

   void DoHorizontalPathLogging(log4cplus::Logger &logger, HorizontalTrajOption current_option) const {
      using json = nlohmann::json;
      if (logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
         auto idx = 0;
         for (HorizontalPath segment : GetHorizontalPath()) {
            json j;
            j["segment_index"] = idx;
            j["trajectory_option"] = GetTrajectoryOptionAsString(current_option);
            j["segment_type"] = segment.GetSegmentTypeAsString();
            j["cumulative_path_length_nm"] =
                  Units::NauticalMilesLength(Units::MetersLength(segment.m_path_length_cumulative_meters)).value();
            j["path_course_deg"] = Units::DegreesAngle(Units::RadiansAngle(segment.m_path_course)).value();
            j["x_position_m"] = segment.GetXPositionMeters();
            j["y_position_m"] = segment.GetYPositionMeters();
            LOG4CPLUS_TRACE(logger, j.dump());
            ++idx;
         }
      }
   }

   void DoVerticalPathLogging(log4cplus::Logger &logger, HorizontalTrajOption current_option) const {
      using json = nlohmann::json;
      if (logger.getLogLevel() == log4cplus::TRACE_LOG_LEVEL) {
         VerticalPath vertical_path = GetVerticalPredictor()->GetVerticalPath();
         json j;
         j["trajectory_option"] = GetTrajectoryOptionAsString(current_option);
         j["time_to_go_sec"] = vertical_path.time_to_go_sec;
         j["along_path_distance_m"] = vertical_path.along_path_distance_m;
         j["altitude_m"] = vertical_path.altitude_m;
         j["cas_mps"] = vertical_path.cas_mps;
         j["altitude_rate_mps"] = vertical_path.altitude_rate_mps;
         j["tas_rate_mps"] = vertical_path.tas_rate_mps;
         j["theta_radians"] = vertical_path.theta_radians;
         j["gs_mps"] = vertical_path.gs_mps;
         j["algorithm_enum"] = vertical_path.algorithm_type;
         LOG4CPLUS_TRACE(logger, j.dump());
      }
   }

   void SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere);

   void DefineRoute();

   void CalculateHorizontalTrajectory(const HorizontalTrajOption option);

   std::vector<aaesim::open_source::TurnAnticipation> CalculateTurnAnticipation(const HorizontalTrajOption option);
};

}  // namespace open_source
}  // namespace aaesim

inline const std::shared_ptr<Atmosphere> aaesim::open_source::EuclideanTrajectoryPredictor::GetAtmosphere() const {
   return m_atmosphere;
}

inline const std::vector<PrecalcWaypoint> &aaesim::open_source::EuclideanTrajectoryPredictor::GetPrecalcWaypoints()
      const {
   return m_waypoint_vector;
}

inline const std::shared_ptr<VerticalPredictor> &
      aaesim::open_source::EuclideanTrajectoryPredictor::GetVerticalPredictor() const {
   return m_vertical_predictor;
}

inline void aaesim::open_source::EuclideanTrajectoryPredictor::SetBankAngle(Units::Angle bank_angle) {
   m_bank_angle = bank_angle;
}

inline Units::Angle aaesim::open_source::EuclideanTrajectoryPredictor::GetBankAngle() const { return m_bank_angle; }

inline Units::Length aaesim::open_source::EuclideanTrajectoryPredictor::GetAltitudeAtFinalWaypoint() const {
   return Units::FeetLength(m_altitude_at_final_waypoint);
}
