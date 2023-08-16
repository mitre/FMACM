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

#include "public/VerticalPredictor.h"

namespace aaesim {
namespace open_source {
class KinematicDescent4DPredictor : public VerticalPredictor {
  public:
   enum KinematicDescentType { CONSTRAINED };

   KinematicDescent4DPredictor();

   virtual ~KinematicDescent4DPredictor();

   void BuildVerticalPrediction(std::vector<HorizontalPath> &horizontal_path,
                                std::vector<PrecalcWaypoint> &precalc_waypoints,
                                const WeatherPrediction &weather_prediction, const Units::Length &start_altitude,
                                const Units::Length &aircraft_distance_to_go);

   void SetMembers(const double &mach_descent, const Units::Speed ias_descent, const Units::Length cruise_altitude,
                   const Units::Length transition_altitude);

   void SetConditionsAtEndOfRoute(const Units::Length altitude_at_end_of_route, const Units::Speed ias_at_end_of_route);

   KinematicDescentType GetDescentType() const;

   virtual const Units::Length GetAltitudeAtEndOfRoute() const;

   const double GetDecelerationRateFPA() const;

  private:
   void ConstrainedVerticalPath(std::vector<HorizontalPath> &horizontal_path,
                                std::vector<PrecalcWaypoint> &precalc_waypoints, double deceleration,
                                double const_gamma_cas_term, double const_gamma_cas_er, double const_gamma_mach,
                                const WeatherPrediction &weather_prediction,
                                const Units::Length &aircraft_distance_to_go);

   VerticalPath ConstantCasVerticalPath(VerticalPath vertical_path, double altitude_at_end,
                                        std::vector<HorizontalPath> &horizontal_path,
                                        std::vector<PrecalcWaypoint> &precalc_waypoints, double gamma,
                                        const WeatherPrediction &weather_prediction,
                                        const Units::Length &aircraft_distance_to_go);

   VerticalPath ConstantMachVerticalPath(VerticalPath vertical_path, double altitude_at_end,
                                         std::vector<HorizontalPath> &horizontal_path,
                                         std::vector<PrecalcWaypoint> &precalc_waypoints, double gamma,
                                         const WeatherPrediction &weather_prediction,
                                         const Units::Length &aircraft_distance_to_go);

   VerticalPath ConstantGeometricFpaVerticalPath(VerticalPath vertical_path, double altitude_at_end,
                                                 double flight_path_angle, std::vector<HorizontalPath> &horizontal_path,
                                                 std::vector<PrecalcWaypoint> &precalc_waypoints,
                                                 const WeatherPrediction &weather_prediction,
                                                 const Units::Length &aircraft_distance_to_go);

   VerticalPath ConstantFpaDecelerationVerticalPath(VerticalPath vertical_path, double altitude_at_end,
                                                    double deceleration, double velocity_cas_end,
                                                    double flight_path_angle,
                                                    std::vector<HorizontalPath> &horizontal_path,
                                                    std::vector<PrecalcWaypoint> &precalc_waypoints,
                                                    const WeatherPrediction &weather_prediction,
                                                    const Units::Length &aircraft_distance_to_go);

   VerticalPath LevelVerticalPath(VerticalPath vertical_path, double x_end,
                                  std::vector<HorizontalPath> &horizontal_path,
                                  const WeatherPrediction &weather_prediction,
                                  const Units::Length &aircraft_distance_to_go);

   VerticalPath ConstantDecelerationVerticalPath(VerticalPath vertical_path, Units::Length distance_to_go,
                                                 Units::Length altitude_high, double deceleration,
                                                 double velocity_cas_end, std::vector<HorizontalPath> &horizontal_path,
                                                 const WeatherPrediction &weather_prediction,
                                                 const Units::Length &aircraft_distance_to_go);

   VerticalPath LevelDecelerationVerticalPath(VerticalPath vertical_path, double deceleration, double velocity_cas_end,
                                              std::vector<HorizontalPath> &horizontal_path,
                                              const WeatherPrediction &weather_prediction,
                                              const Units::Length &aircraft_distance_to_go);

   VerticalPath LevelDecelerationVerticalPath(VerticalPath vertical_path, Units::Length distance_to_go,
                                              double deceleration, double velocity_cas_end,
                                              std::vector<HorizontalPath> &horizontal_path,
                                              const WeatherPrediction &weather_prediction,
                                              const Units::Length &aircraft_distance_to_go);

   VerticalPath ConstantFpaToCurrentPositionVerticalPath(VerticalPath vertical_path,
                                                         std::vector<HorizontalPath> &horizontal_path,
                                                         std::vector<PrecalcWaypoint> &precalc_waypoints,
                                                         double const_gamma_mach,
                                                         const WeatherPrediction &weather_prediction,
                                                         const Units::Length &aircraft_distance_to_go);

   void ComputeWindCoefficients(Units::Length altitude, Units::Angle course,
                                const WeatherPrediction &weather_prediction, Units::Speed &parallel_wind_velocity,
                                Units::Speed &perpendicular_wind_velocity, Units::Speed &wind_velocity_x,
                                Units::Speed &wind_velocity_y);

   void TrimVerticalPath(VerticalPath &vertical_path, int path_index);

   KinematicDescentType m_kinematic_descent_type;

   Units::Length m_altitude_at_end_of_route;

   double m_deceleration_mps;
   double m_deceleration_level_mps;
   double m_deceleration_fpa_mps;

   double m_const_gamma_cas_term_rad;
   double m_const_gamma_cas_er_rad;
   double m_const_gamma_mach_rad;

   std::vector<int> m_vertical_path_waypoint_index;
   static const Units::Length m_vertical_tolerance_distance;

   bool m_prediction_too_low;
   bool m_prediction_too_high;

   static log4cplus::Logger m_logger;
};
}  // namespace open_source
}  // namespace aaesim

inline aaesim::open_source::KinematicDescent4DPredictor::KinematicDescentType
      aaesim::open_source::KinematicDescent4DPredictor::GetDescentType() const {
   return m_kinematic_descent_type;
}

inline const Units::Length aaesim::open_source::KinematicDescent4DPredictor::GetAltitudeAtEndOfRoute() const {
   return m_altitude_at_end_of_route;
}

inline void aaesim::open_source::KinematicDescent4DPredictor::SetConditionsAtEndOfRoute(
      const Units::Length altitude_at_end_of_route, const Units::Speed ias_at_end_of_route) {
   m_altitude_at_end_of_route = altitude_at_end_of_route;
   m_ias_at_end_of_route = ias_at_end_of_route;
}

inline const double aaesim::open_source::KinematicDescent4DPredictor::GetDecelerationRateFPA() const {
   return m_deceleration_fpa_mps;
}