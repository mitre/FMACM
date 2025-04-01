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
#include <vector>
#include <string>
#include "public/FixedMassAircraftPerformance.h"
#include "public/Wgs84PrecalcWaypoint.h"
#include "public/PrecalcConstraint.h"
#include "public/VerticalPath.h"
#include "avionics/Wgs84HorizontalPathSegment.h"
#include "public/WeatherPrediction.h"
#include "public/AircraftState.h"
#include "public/Guidance.h"
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Time.h>
#include "public/CalcWindGradControl.h"
#include "avionics/Wgs84DirectionOfFlightCourseCalculator.h"

namespace aaesim {

class Wgs84VerticalPredictor {
  public:
   Wgs84VerticalPredictor();

   virtual ~Wgs84VerticalPredictor(void) = default;

   Wgs84VerticalPredictor &operator=(const Wgs84VerticalPredictor &obj);

   bool operator==(const Wgs84VerticalPredictor &obj) const;

   bool operator!=(const Wgs84VerticalPredictor &obj) const;

   virtual void BuildVerticalPrediction(const std::vector<Wgs84HorizontalPathSegment> &horizontal_path,
                                        const std::vector<Wgs84PrecalcWaypoint> &precalc_waypoints,
                                        const aaesim::open_source::WeatherPrediction &weather,
                                        const Units::Length &start_altitude,
                                        const Units::Length &aircraft_distance_to_go) = 0;

   void SetMembers(const Wgs84VerticalPredictor &Wgs84VerticalPredictor);

   virtual const Units::Length GetAltitudeAtEndOfRoute() const = 0;

   std::shared_ptr<const aaesim::open_source::FixedMassAircraftPerformance> GetAircraftPerformance() const;

   const VerticalPath &GetVerticalPath() const;

   void SetCruiseAltitude(Units::Length cruise_altitude_msl);

   Units::Length GetCruiseAltitude() const;

   const double GetTransitionMach() const;

   Units::Speed GetTransitionIas() const;

   void SetTransitionIas(Units::Speed ias_transition);

   void SetTransitionAltitude(Units::Length transition_altitude_msl);

   Units::Length GetTransitionAltitude() const;

   Units::KnotsSpeed GetIasAtEndOfRoute();

   std::shared_ptr<Atmosphere> GetAtmosphere() const;

   void SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere);

   const bool IsCruiseMachValid() const;

  protected:
   constexpr static auto TIME_STEP_SECONDS{0.5};
   inline static const Units::MetersPerSecondSpeed SPEED_DIFFERENCE_THRESHOLD{-0.1};
   inline static const Units::KnotsSpeed HIGH_SPEED_CONSTRAINT_THRESHOLD{1000};
   inline static const Units::FeetLength ALT_DIFFERENCE_THRESHOLD{100};

   double CalculateEsfUsingConstantMach(const double true_airspeed_mps, const double altitude_msl_meter,
                                        const Units::Temperature temperature);

   void TrimDuplicatesFromVerticalPath();

   const Units::KnotsSpeed m_low_groundspeed_warning;
   const Units::KnotsSpeed m_low_groundspeed_fatal;
   const Units::DegreesAngle m_descent_angle_max;
   const Units::DegreesAngle m_descent_angle_warning;
   int m_current_trajectory_index;
   Units::Length m_cruise_altitude_msl;
   Units::Speed m_transition_ias;
   Units::Length m_transition_altitude_msl;
   double m_cruise_mach;
   aaesim::open_source::CalcWindGradControl m_wind_calculator;
   Units::MetersLength m_start_altitude_msl;
   VerticalPath m_vertical_path;
   Wgs84DirectionOfFlightCourseCalculator m_course_calculator;
   Units::Speed m_ias_at_end_of_route;
   std::shared_ptr<Atmosphere> m_atmosphere;
   std::shared_ptr<const aaesim::open_source::FixedMassAircraftPerformance> m_bada_calculator;
};

inline const VerticalPath &Wgs84VerticalPredictor::GetVerticalPath() const { return m_vertical_path; }

inline void Wgs84VerticalPredictor::SetCruiseAltitude(Units::Length cruise_altitude_msl) {
   m_cruise_altitude_msl = cruise_altitude_msl;
}

inline Units::Length Wgs84VerticalPredictor::GetCruiseAltitude() const { return m_cruise_altitude_msl; }

inline const double Wgs84VerticalPredictor::GetTransitionMach() const { return m_cruise_mach; }

inline Units::Speed Wgs84VerticalPredictor::GetTransitionIas() const { return m_transition_ias; }

inline void Wgs84VerticalPredictor::SetTransitionIas(Units::Speed ias_transition) { m_transition_ias = ias_transition; }

inline void Wgs84VerticalPredictor::SetTransitionAltitude(Units::Length transition_altitude_msl) {
   m_transition_altitude_msl = transition_altitude_msl;
}

inline Units::Length Wgs84VerticalPredictor::GetTransitionAltitude() const { return m_transition_altitude_msl; }

inline std::shared_ptr<Atmosphere> Wgs84VerticalPredictor::GetAtmosphere() const { return m_atmosphere; }

inline void Wgs84VerticalPredictor::SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere) { m_atmosphere = atmosphere; }

inline const bool Wgs84VerticalPredictor::IsCruiseMachValid() const { return m_cruise_mach > 0 && m_cruise_mach < 1; }

inline Units::KnotsSpeed Wgs84VerticalPredictor::GetIasAtEndOfRoute() { return m_ias_at_end_of_route; }

inline std::shared_ptr<const aaesim::open_source::FixedMassAircraftPerformance>
      Wgs84VerticalPredictor::GetAircraftPerformance() const {
   return m_bada_calculator;
}

}  // namespace aaesim