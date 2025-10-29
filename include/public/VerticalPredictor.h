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

#include <memory>
#include <string>
#include <vector>

#include "public/AircraftState.h"
#include "public/CalcWindGradControl.h"
#include "public/DirectionOfFlightCourseCalculator.h"
#include "public/FixedMassAircraftPerformance.h"
#include "public/Guidance.h"
#include "public/HorizontalPath.h"
#include "public/PrecalcConstraint.h"
#include "public/PrecalcWaypoint.h"
#include "public/VerticalPath.h"
#include "public/WeatherPrediction.h"
#include "scalar/Length.h"
#include "scalar/Speed.h"
#include "scalar/Time.h"

class VerticalPredictor {
  public:
   VerticalPredictor();

   virtual ~VerticalPredictor(void) = default;

   VerticalPredictor &operator=(const VerticalPredictor &obj);

   bool operator==(const VerticalPredictor &obj) const;

   bool operator!=(const VerticalPredictor &obj) const;

   virtual void BuildVerticalPrediction(std::vector<aaesim::open_source::HorizontalPath> &horizontal_path,
                                        std::vector<PrecalcWaypoint> &precalc_waypoints,
                                        const aaesim::open_source::WeatherPrediction &weather,
                                        const Units::Length &start_altitude,
                                        const Units::Length &aircraft_distance_to_go) = 0;

   void SetMembers(const VerticalPredictor &vertical_predictor);

   virtual const Units::Length GetAltitudeAtEndOfRoute() const = 0;

   aaesim::open_source::Guidance Update(const aaesim::open_source::AircraftState &current_state,
                                        const aaesim::open_source::Guidance &current_guidance,
                                        const Units::Length distance_to_go);

   const VerticalPath &GetVerticalPath() const;

   void SetCruiseAltitude(Units::Length cruise_altitude_msl);

   Units::Length GetCruiseAltitude() const;

   const double GetTransitionMach() const;

   Units::Speed GetTransitionIas() const;

   void SetTransitionAltitude(Units::Length transition_altitude_msl);

   Units::Length GetTransitionAltitude() const;

   Units::KnotsSpeed GetIasAtEndOfRoute();

   std::shared_ptr<Atmosphere> GetAtmosphere() const;
   void SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere);

  protected:
   double CalculateEsfUsingConstantCAS(const double true_airspeed_mps, const double altitude_msl_meter,
                                       const Units::Temperature temperature);

   double CalculateEsfUsingConstantMach(const double true_airspeed_mps, const double altitude_msl_meter,
                                        const Units::Temperature temperature);

   PrecalcConstraint CheckActiveConstraint(double along_path_distance_to_go_meter, double altitude_msl_meter,
                                           double calibrated_airspeed_mps, const PrecalcConstraint &constraints,
                                           double transition_altitude_meter);

   aaesim::open_source::Guidance CalculateGuidanceCommands(const aaesim::open_source::AircraftState &state,
                                                           const Units::Length distance_to_go,
                                                           const aaesim::open_source::Guidance &current_guidance);

   void TrimDuplicatesFromVerticalPath();

   PrecalcConstraint FindActiveConstraint(const double &along_path_distance_to_go_meters,
                                          const std::vector<PrecalcWaypoint> &precalculated_waypoints);

   const bool IsCruiseMachValid() const;

   inline static const double TIME_STEP_SECONDS{0.5};
   inline static const Units::MetersPerSecondSpeed SPEED_DIFFERENCE_THRESHOLD{-0.1};
   inline static const Units::KnotsSpeed HIGH_SPEED_CONSTRAINT_THRESHOLD{1000};
   inline static const Units::FeetLength ALT_DIFFERENCE_THRESHOLD{100};

   const Units::KnotsSpeed LOW_GROUNDSPEED_WARNING;
   const Units::KnotsSpeed LOW_GROUNDSPEED_FATAL;
   const Units::DegreesAngle DESCENT_ANGLE_MAX;
   const Units::DegreesAngle DESCENT_ANGLE_WARNING;
   int m_current_trajectory_index;
   Units::Length m_cruise_altitude_msl;
   Units::Time m_descent_start_time;
   Units::Speed m_transition_ias;
   Units::Length m_transition_altitude_msl;
   double m_cruise_mach;
   double m_transition_mach;
   PrecalcConstraint m_precalculated_constraints;
   aaesim::open_source::CalcWindGradControl m_wind_calculator;
   Units::MetersLength m_start_altitude_msl;
   VerticalPath m_vertical_path;
   aaesim::open_source::DirectionOfFlightCourseCalculator m_course_calculator;
   Units::Speed m_ias_at_end_of_route;
   std::shared_ptr<Atmosphere> m_atmosphere;
   std::shared_ptr<const aaesim::open_source::FixedMassAircraftPerformance> m_bada_calculator;
};

inline const VerticalPath &VerticalPredictor::GetVerticalPath() const { return m_vertical_path; }

inline void VerticalPredictor::SetCruiseAltitude(Units::Length cruise_altitude_msl) {
   m_cruise_altitude_msl = cruise_altitude_msl;
}

inline Units::Length VerticalPredictor::GetCruiseAltitude() const { return m_cruise_altitude_msl; }

inline const double VerticalPredictor::GetTransitionMach() const { return m_transition_mach; }

inline Units::Speed VerticalPredictor::GetTransitionIas() const { return m_transition_ias; }

inline void VerticalPredictor::SetTransitionAltitude(Units::Length transition_altitude_msl) {
   m_transition_altitude_msl = transition_altitude_msl;
}

inline Units::Length VerticalPredictor::GetTransitionAltitude() const { return m_transition_altitude_msl; }

inline std::shared_ptr<Atmosphere> VerticalPredictor::GetAtmosphere() const { return m_atmosphere; }

inline void VerticalPredictor::SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere) { m_atmosphere = atmosphere; }

inline const bool VerticalPredictor::IsCruiseMachValid() const { return m_transition_mach > 0; }

inline Units::KnotsSpeed VerticalPredictor::GetIasAtEndOfRoute() { return m_ias_at_end_of_route; }
