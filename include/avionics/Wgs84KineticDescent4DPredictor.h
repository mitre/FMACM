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

#include "avionics/Wgs84VerticalPredictor.h"
#include "avionics/Wgs84HorizontalPathSegment.h"
#include "public/VerticalPath.h"
#include "public/AircraftState.h"
#include "public/Guidance.h"
#include "public/PrecalcConstraint.h"
#include "public/Wgs84PrecalcWaypoint.h"
#include "public/DMatrix.h"
#include "public/WindStack.h"

#include <string>
#include <vector>
#include <scalar/Speed.h>
#include <scalar/Length.h>
#include <scalar/Time.h>

class Wgs84KineticDescent4DPredictor : public aaesim::Wgs84VerticalPredictor {
  public:
   enum Wgs84KineticDescentType { GEOMETRIC = 0, DRIVE_AND_DIVE, DIVE_AND_DRIVE };

   Wgs84KineticDescent4DPredictor();

   Wgs84KineticDescent4DPredictor(const std::shared_ptr<aaesim::Wgs84VerticalPredictor> &obj);

   virtual ~Wgs84KineticDescent4DPredictor() = default;

   void Initialize(const std::string &aircraft_type, const Units::Length cruise_altitude_msl, const double cruise_mach,
                   const Units::Length altitude_at_final_waypoint,
                   const aaesim::open_source::WeatherPrediction &weather_prediction);

   virtual void BuildVerticalPrediction(const std::vector<aaesim::Wgs84HorizontalPathSegment> &horizontal_path,
                                        const std::vector<aaesim::Wgs84PrecalcWaypoint> &precalc_waypoints,
                                        const aaesim::open_source::WeatherPrediction &weather_prediction,
                                        const Units::Length &start_altitude,
                                        const Units::Length &aircraft_distance_to_go) = 0;

   void SetMembers(const Wgs84KineticDescent4DPredictor &kinetic_descent_4d_predictor);

   Wgs84KineticDescent4DPredictor &operator=(const Wgs84KineticDescent4DPredictor &obj);

   virtual Wgs84KineticDescent4DPredictor *GetDeepCopy() const = 0;

   bool operator==(const Wgs84KineticDescent4DPredictor &obj) const;

   bool operator!=(const Wgs84KineticDescent4DPredictor &obj) const;

   const Units::Length GetAltitudeAtEndOfRoute() const override;

   void SetAltitudeAtEndOfRoute(Units::Length altitude_at_end_of_route);

   void SetIasAtEndOfRoute(Units::Speed ias_at_end_of_route);

   void SetKineticDescentType(Wgs84KineticDescentType type_in);

   Wgs84KineticDescentType GetKineticDescentType() const;

   void ConfigureParameters(double mass_percentile, Units::Length altitude_at_end_of_route,
                            Units::Speed ias_at_end_of_route, Units::Speed transition_ias, double idle_thrust_factor);

  protected:
   VerticalPath IdleVerticalPath(double esf, const VerticalPath &vertical_path, double ias_in_tracon_mps,
                                 double altitude_in_tracon_meters, double distance_to_end_of_segment_meters,
                                 const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath ConstantCASVerticalPath(const VerticalPath &vertical_path, double altitude_at_end_of_segment_meters,
                                        const std::vector<aaesim::Wgs84HorizontalPathSegment> &horizontal_path,
                                        const std::vector<aaesim::Wgs84PrecalcWaypoint> &precalc_waypoints,
                                        const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath ConstantMachVerticalPath(const VerticalPath &vertical_path, double altitude_at_end_of_segment_meters,
                                         const std::vector<aaesim::Wgs84HorizontalPathSegment> &horizontal_path,
                                         const std::vector<aaesim::Wgs84PrecalcWaypoint> &precalc_waypoints,
                                         const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath LevelVerticalPath(const VerticalPath &vertical_path, double distance_to_end_of_segment_meters,
                                  const std::vector<aaesim::Wgs84HorizontalPathSegment> &horizontal_path,
                                  const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath ConstantFlightPathAngleVerticalPath(
         const VerticalPath &vertical_path, double altitude_at_end_of_segment_meters, double flight_path_angle,
         const std::vector<aaesim::Wgs84HorizontalPathSegment> &horizontal_path,
         const std::vector<aaesim::Wgs84PrecalcWaypoint> &precalc_waypoints,
         const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath ConstantFlightPathAngleDecelVerticalPath(
         const VerticalPath &vertical_path, double altitude_at_end_of_segment_meters, double ias_at_end_of_segment_mps,
         const double flight_path_angle, const std::vector<aaesim::Wgs84HorizontalPathSegment> &horizontal_path,
         const std::vector<aaesim::Wgs84PrecalcWaypoint> &precalc_waypoints,
         const aaesim::open_source::WeatherPrediction &weather_prediction);

   VerticalPath LevelDecelVerticalPath(const VerticalPath &vertical_path, double ias_at_end_of_segment_mps,
                                       const std::vector<aaesim::Wgs84HorizontalPathSegment> &horizontal_path,
                                       const aaesim::open_source::WeatherPrediction &weather_prediction);

   // method to calculate a level deceleration vertical path that terminates at end of segment.
   VerticalPath LevelDecelVerticalPath(const VerticalPath &vertical_path, double ias_at_end_of_segment_mps,
                                       double distance_to_end_of_segment_meters,
                                       const std::vector<aaesim::Wgs84HorizontalPathSegment> &horizontal_path,
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

   PrecalcConstraint FindActiveConstraint(const Units::Length &along_path_distance_to_go,
                                          const Units::Length &altitude_msl, const Units::Speed &calibrated_airspeed,
                                          const Units::Length &transition_altitude,
                                          const std::vector<aaesim::Wgs84PrecalcWaypoint> &precalc_waypoints);

   double CalculateEsfUsingConstantCAS(const double true_airspeed_mps, const double altitude_msl_meter,
                                       const Units::Temperature temperature);

   PrecalcConstraint m_precalculated_constraints;
   Wgs84KineticDescentType m_kinetic_descent_type;
   Units::Length m_altitude_at_end_of_route;
   double m_energy_share_factor;
   std::string m_aircraft_type;
   double m_mass_percentile;
   double m_idle_thrust_factor;

  private:
   static const Units::KnotsSpeed AIRSPACE_IAS_250KTS;
   static const Units::FeetLength AIRSPACE_ALTITUDE_10K_MSL;

   PrecalcConstraint CheckActiveConstraint(const Units::Length &along_path_distance,
                                           const Units::Length &altitude_msl_meter,
                                           const Units::Speed &calibrated_airspeed,
                                           const PrecalcConstraint &precalc_constraint,
                                           const Units::Length &transition_altitude);

   static log4cplus::Logger logger;
};

inline void Wgs84KineticDescent4DPredictor::SetIasAtEndOfRoute(Units::Speed ias_at_end_of_route) {
   m_ias_at_end_of_route = ias_at_end_of_route;
}

inline void Wgs84KineticDescent4DPredictor::SetAltitudeAtEndOfRoute(Units::Length altitude_at_end_of_route) {
   m_altitude_at_end_of_route = altitude_at_end_of_route;
}

inline void Wgs84KineticDescent4DPredictor::SetKineticDescentType(Wgs84KineticDescentType type_in) {
   m_kinetic_descent_type = type_in;
}

inline Wgs84KineticDescent4DPredictor::Wgs84KineticDescentType Wgs84KineticDescent4DPredictor::GetKineticDescentType()
      const {
   return m_kinetic_descent_type;
}

inline const Units::Length Wgs84KineticDescent4DPredictor::GetAltitudeAtEndOfRoute() const {
   return m_altitude_at_end_of_route;
}
