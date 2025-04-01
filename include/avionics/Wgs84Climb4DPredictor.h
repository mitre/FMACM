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

#include "public/LoggingLoadable.h"
#include "public/VerticalPath.h"
#include "public/AircraftState.h"
#include "public/Guidance.h"
#include "avionics/ClimbPrecalcConstraint.h"
#include "public/Wgs84PrecalcWaypoint.h"
#include "avionics/Wgs84HorizontalPathSegment.h"
#include "avionics/Wgs84VerticalPredictor.h"
#include "public/DMatrix.h"
#include "public/WindStack.h"
#include "avionics/Wgs84DirectionOfFlightCourseCalculator.h"
#include "public/CalcWindGradControl.h"
#include "public/WeatherPrediction.h"

#include <string>
#include <vector>
#include <scalar/Speed.h>
#include <scalar/Length.h>
#include <scalar/Time.h>

namespace aaesim {
class Wgs84Climb4DPredictor : public Wgs84VerticalPredictor {
  public:
   static const Units::KnotsSpeed MINIMUM_INITIAL_IAS, AIRSPACE_IAS_250KTS;
   static const Units::FeetLength AIRSPACE_ALTITUDE_10K_MSL, AIRSPACE_CONSTRAINT_ALTITUDE_MSL;

   Wgs84Climb4DPredictor(double mass_percentile, Units::Speed mach_transition_cas);

   ~Wgs84Climb4DPredictor() = default;

   void BuildVerticalPrediction(const std::vector<Wgs84HorizontalPathSegment> &horizontal_path,
                                const std::vector<Wgs84PrecalcWaypoint> &precalc_waypoints,
                                const aaesim::open_source::WeatherPrediction &weather_prediction,
                                const Units::Length &start_altitude,
                                const Units::Length &aircraft_distance_to_go = Units::ZERO_LENGTH) override;

   void Initialize(const std::string &aircraft_type, const aaesim::open_source::WeatherPrediction &weather_prediction,
                   const Units::Length cruise_altitude_msl, const double cruise_mach);

   Units::Length GetTransitionAltitude() const;

   VerticalPath GetVerticalPath() const;

   const Units::Length GetAltitudeAtEndOfRoute() const override;

  protected:
   const VerticalPath GroundAccel(Units::Acceleration accel_rate, Units::Speed rotate_speed, Units::Length elevation,
                                  const aaesim::open_source::WeatherPrediction &weather_prediction);

   ClimbPrecalcConstraint FindActiveClimbConstraint(const Units::Length &along_path_distance,
                                                    const std::vector<Wgs84PrecalcWaypoint> &precalculated_waypoints);

   ClimbActiveFlagType GetClimbActiveFlag(const Units::Length &along_path_distance,
                                          const Units::Length &altitude_msl_meter,
                                          const Units::Speed &calibrated_airspeed, ClimbPrecalcConstraint &constraints);

   void ConstantCasClimb(const Units::Length &altitude_at_end_of_segment, const Units::Length &distance_at_segment_end,
                         const double thrust_factor, const aaesim::open_source::WeatherPrediction &weather_prediction);

   void ConstantMachClimb(const Units::Length &altitude_at_end_of_segment_meters, const Units::Length &distance,
                          const double thrust_factor, const aaesim::open_source::WeatherPrediction &weather_prediction);

   void LevelFlightConstantSpeed(const Units::Length &distance, const double thrust_factor,
                                 const aaesim::open_source::WeatherPrediction &weather_prediction);

   void EsfClimb(const Units::Length &altitude_at_end_of_segment_meters, const Units::Speed &climb_cas,
                 const Units::Length &distance, const double thrust_factor,
                 const aaesim::open_source::WeatherPrediction &weather_prediction);

   void LevelAccel(const Units::Length &altitude_at_end_of_segment_meters, const Units::Speed &climb_cas,
                   const Units::Length &distance, const double thrust_factor,
                   const aaesim::open_source::WeatherPrediction &weather_prediction);

   void CalculateWindVelocityRelativeToTrack(const Units::MetersPerSecondSpeed Vwx,
                                             const Units::MetersPerSecondSpeed Vwy, const Units::Angle course,
                                             Units::MetersPerSecondSpeed &wind_velocity_perpendicular_to_track,
                                             Units::MetersPerSecondSpeed &wind_velocity_parallel_to_track);

   ClimbPrecalcConstraint m_precalculated_constraints;

  private:
   static log4cplus::Logger logger;
   double m_initial_thrust_factor;
   double m_energy_share_factor;
   double m_mass_percentile;
};
}  // namespace aaesim

inline Units::Length aaesim::Wgs84Climb4DPredictor::GetTransitionAltitude() const { return m_transition_altitude_msl; }

inline VerticalPath aaesim::Wgs84Climb4DPredictor::GetVerticalPath() const { return m_vertical_path; }

inline const Units::Length aaesim::Wgs84Climb4DPredictor::GetAltitudeAtEndOfRoute() const {
   return m_cruise_altitude_msl;
}
