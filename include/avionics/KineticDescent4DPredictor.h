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
#include "public/LoggingLoadable.h"

#include <string>
#include <vector>
#include <scalar/Speed.h>
#include <scalar/Length.h>
#include <scalar/Time.h>

#include "public/VerticalPath.h"
#include "public/AircraftState.h"
#include "public/Guidance.h"
#include "public/PrecalcConstraint.h"
#include "public/PrecalcWaypoint.h"
#include "public/HorizontalPath.h"
#include "public/DMatrix.h"
#include "public/WindStack.h"

class KineticDescent4DPredictor : public VerticalPredictor, public LoggingLoadable {
  public:
   enum KineticDescentType { GEOMETRIC = 0, DRIVE_AND_DIVE, DIVE_AND_DRIVE };

   KineticDescent4DPredictor();

   KineticDescent4DPredictor(const std::shared_ptr<VerticalPredictor> &obj);

   virtual ~KineticDescent4DPredictor() = default;

   bool load(DecodedStream *input);

   void Initialize(const std::string &aircraft_type, const Units::Length altitude_at_final_waypoint,
                   const aaesim::open_source::WeatherPrediction &weather_prediction);

   virtual void BuildVerticalPrediction(std::vector<aaesim::open_source::HorizontalPath> &horizontal_path,
                                        std::vector<PrecalcWaypoint> &precalc_waypoints,
                                        const aaesim::open_source::WeatherPrediction &weather_prediction,
                                        const Units::Length &start_altitude,
                                        const Units::Length &aircraft_distance_to_go) = 0;

   void SetMembers(const KineticDescent4DPredictor &kinetic_descent_4d_predictor);

   KineticDescent4DPredictor &operator=(const KineticDescent4DPredictor &obj);

   virtual KineticDescent4DPredictor *GetDeepCopy() const = 0;

   bool operator==(const KineticDescent4DPredictor &obj) const;

   bool operator!=(const KineticDescent4DPredictor &obj) const;

   bool IsModelLoaded();

   const Units::Length GetAltitudeAtEndOfRoute() const;

   void SetAltitudeAtEndOfRoute(Units::Length altitude_at_end_of_route);

   void SetIasAtEndOfRoute(Units::Speed ias_at_end_of_route);

   void SetKineticDescentType(KineticDescentType type_in);

   KineticDescentType GetKineticDescentType() const;

   Units::Length m_altitude_at_final_waypoint;
   std::string m_aircraft_type;
   double m_energy_share_factor;
   double m_mass_percentile;
   double m_idle_thrust_factor;

   bool m_model_loaded;

  protected:
   VerticalPath IdleVerticalPath(double esf, VerticalPath vertical_path, double ias_in_tracon_mps,
                                 double altitude_in_tracon_meters,
                                 const aaesim::open_source::WeatherPrediction &weather_prediction);

   // method to calculate a decelerating idle verical path that terminates at end of segment.
   VerticalPath IdleVerticalPath(double esf, VerticalPath vertical_path, double ias_in_tracon_mps,
                                 double altitude_in_tracon_meters, double distance_to_end_of_segment_meters,
                                 const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath ConstantCASVerticalPath(VerticalPath vertical_path, double altitude_at_end_of_segment_meters,
                                        std::vector<aaesim::open_source::HorizontalPath> &horizontal_path,
                                        std::vector<PrecalcWaypoint> &precalc_waypoints,
                                        const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath ConstantMachVerticalPath(VerticalPath vertical_path, double altitude_at_end_of_segment_meters,
                                         std::vector<aaesim::open_source::HorizontalPath> &horizontal_path,
                                         std::vector<PrecalcWaypoint> &precalc_waypoints,
                                         const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath LevelVerticalPath(VerticalPath vertical_path, double distance_to_end_of_segment_meters,
                                  std::vector<aaesim::open_source::HorizontalPath> &horizontal_path,
                                  const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath ConstantFlightPathAngleVerticalPath(VerticalPath vertical_path,
                                                    double altitude_at_end_of_segment_meters, double flight_path_angle,
                                                    std::vector<aaesim::open_source::HorizontalPath> &horizontal_path,
                                                    std::vector<PrecalcWaypoint> &precalc_waypoints,
                                                    const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath ConstantFlightPathAngleDecelVerticalPath(
         VerticalPath vertical_path, double altitude_at_end_of_segment_meters, double ias_at_end_of_segment_mps,
         double flight_path_angle, std::vector<aaesim::open_source::HorizontalPath> &horizontal_path,
         std::vector<PrecalcWaypoint> &precalc_waypoints,
         const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath LevelDecelVerticalPath(VerticalPath vertical_path, double ias_at_end_of_segment_mps,
                                       std::vector<aaesim::open_source::HorizontalPath> &horizontal_path,
                                       const aaesim::open_source::WeatherPrediction &weather_prediction);

   // method to calculate a level deceleration vertical path that terminates at end of segment.
   VerticalPath LevelDecelVerticalPath(VerticalPath vertical_path, double ias_at_end_of_segment_mps,
                                       double distance_to_end_of_segment_meters,
                                       std::vector<aaesim::open_source::HorizontalPath> &horizontal_path,
                                       const aaesim::open_source::WeatherPrediction &weather_prediction);

   const VerticalPath InitializeVerticalPath(const aaesim::open_source::WeatherPrediction &weather_prediction);

   void GetWindSpeedXYComponents(const aaesim::open_source::WeatherPrediction &weather_prediction, Units::Speed &Vwx,
                                 Units::Speed &Vwy);

   void CalculateWindVelocityRelativeToTrack(const Units::MetersPerSecondSpeed Vwx,
                                             const Units::MetersPerSecondSpeed Vwy, const Units::Angle course,
                                             Units::MetersPerSecondSpeed &wind_velocity_perpendicular_to_track,
                                             Units::MetersPerSecondSpeed &wind_velocity_parallel_to_track);

   const Units::MetersPerSecondSpeed CalculateInitialGroundSpeed(
         const aaesim::open_source::WeatherPrediction &weather_prediction,
         const Units::MetersPerSecondSpeed wind_velocity_perpendicular_to_track,
         const Units::MetersPerSecondSpeed wind_velocity_parallel_to_track);

   KineticDescentType m_kinetic_descent_type;
   Units::Length m_altitude_at_end_of_route;

   static log4cplus::Logger logger;
};

inline void KineticDescent4DPredictor::SetIasAtEndOfRoute(Units::Speed ias_at_end_of_route) {
   m_ias_at_end_of_route = ias_at_end_of_route;
}

inline void KineticDescent4DPredictor::SetAltitudeAtEndOfRoute(Units::Length altitude_at_end_of_route) {
   m_altitude_at_end_of_route = altitude_at_end_of_route;
}

inline void KineticDescent4DPredictor::SetKineticDescentType(KineticDescentType type_in) {
   m_kinetic_descent_type = type_in;
}

inline KineticDescent4DPredictor::KineticDescentType KineticDescent4DPredictor::GetKineticDescentType() const {
   return m_kinetic_descent_type;
}

inline const Units::Length KineticDescent4DPredictor::GetAltitudeAtEndOfRoute() const {
   return m_altitude_at_end_of_route;
}

inline bool KineticDescent4DPredictor::IsModelLoaded() { return m_model_loaded; }
