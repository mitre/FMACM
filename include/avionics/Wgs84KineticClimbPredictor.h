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

#include "avionics/Wgs84TrajectoryPredictor.h"

#include "avionics/Wgs84HorizontalPathSegment.h"
#include "public/AircraftIntent.h"
#include "public/Guidance.h"
#include "public/AircraftState.h"
#include "public/Logging.h"
#include <vector>
#include "scalar/Angle.h"
#include "avionics/Wgs84PositionCalculator.h"
#include "avionics/Wgs84AlongPathDistanceCalculator.h"
#include "avionics/Wgs84VerticalPredictor.h"
#include "avionics/Wgs84Climb4DPredictor.h"
#include "public/EuclideanTrajectoryPredictor.h"
#include "public/TurnAnticipation.h"

namespace aaesim {

class Wgs84KineticClimbPredictor : public Wgs84TrajectoryPredictor {
  public:
   Wgs84KineticClimbPredictor();

   Wgs84KineticClimbPredictor(double mass_percentile, Units::Speed mach_transition_cas);

   Wgs84KineticClimbPredictor(const Wgs84KineticClimbPredictor &obj);

   virtual ~Wgs84KineticClimbPredictor() = default;

   bool operator==(const Wgs84KineticClimbPredictor &obj) const;

   bool operator!=(const Wgs84KineticClimbPredictor &obj) const;

   Wgs84KineticClimbPredictor &operator=(const Wgs84KineticClimbPredictor &obj) = default;

   void BuildTrajectoryPrediction(const std::string &aircraft_peformance_name,
                                  aaesim::open_source::WeatherPrediction &weather_prediction,
                                  Units::Length start_altitude) override;

   void CalculateWaypoints(const AircraftIntent &aircraft_intent,
                           const aaesim::open_source::WeatherPrediction &weather_prediction) override;

   std::shared_ptr<Wgs84Climb4DPredictor> GetVertPredictor() const;

   const std::string &GetAircraftType() const;

   const AircraftIntent &GetAircraftIntent() const override;

   const std::vector<Wgs84HorizontalPathSegment> &GetHorizontalPath() const override;

   const std::shared_ptr<Atmosphere> GetAtmosphere() const;

   const std::vector<Wgs84PrecalcWaypoint> &GetPrecalcWaypoints() const override;

   Units::Angle GetBankAngle() const;

   void SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere);

   static geolib_idealab::ArcDirection GetTurnDirection(const Units::Angle in_direction,
                                                        const Units::Angle out_direction);

   const VerticalPath GetVerticalPath() const override;

  protected:
   void AdjustConstraints(const Units::Speed start_speed);

   void UpdateWeatherPrediction(aaesim::open_source::WeatherPrediction &weather) const;

   void Copy(const Wgs84KineticClimbPredictor &obj);

   std::vector<Wgs84HorizontalPathSegment> m_horizontal_path;
   std::vector<Wgs84PrecalcWaypoint> m_processed_waypoints;
   Units::Angle m_bank_angle;
   AircraftIntent m_aircraft_intent;
   Wgs84AlongPathDistanceCalculator m_distance_calculator;
   Wgs84PositionCalculator m_position_calculator;
   std::shared_ptr<Atmosphere> m_atmosphere;

  private:
   // Allowable course change, in degrees, to be considered straight, and does not need turn anticipation
   // Also the allowable course change between RF-Leg course and previous or next course to consider aligned.
   static Units::Angle CONSIDER_TO_BE_STRAIGHT;

   static log4cplus::Logger m_logger;

   void CalculateHorizontalTrajectory(const TrajectoryPassOption option);

   std::vector<aaesim::open_source::TurnAnticipation> CalculateTurnAnticipation(const TrajectoryPassOption option);

   std::string m_aircraft_type;

   std::shared_ptr<Wgs84Climb4DPredictor> m_vertical_predictor;

   Units::Speed m_mach_transition_cas;
};

inline std::shared_ptr<Wgs84Climb4DPredictor> Wgs84KineticClimbPredictor::GetVertPredictor() const {
   return std::shared_ptr<Wgs84Climb4DPredictor>(std::static_pointer_cast<Wgs84Climb4DPredictor>(m_vertical_predictor));
}

inline const std::string &Wgs84KineticClimbPredictor::GetAircraftType() const { return m_aircraft_type; }

inline const std::shared_ptr<Atmosphere> Wgs84KineticClimbPredictor::GetAtmosphere() const { return m_atmosphere; }

inline const std::vector<Wgs84PrecalcWaypoint> &Wgs84KineticClimbPredictor::GetPrecalcWaypoints() const {
   return m_processed_waypoints;
}

inline Units::Angle Wgs84KineticClimbPredictor::GetBankAngle() const { return m_bank_angle; }

inline const std::vector<Wgs84HorizontalPathSegment> &Wgs84KineticClimbPredictor::GetHorizontalPath() const {
   return m_horizontal_path;
}

inline const AircraftIntent &Wgs84KineticClimbPredictor::GetAircraftIntent() const { return m_aircraft_intent; }

inline void Wgs84KineticClimbPredictor::SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere) {
   m_atmosphere = atmosphere;
}

inline const VerticalPath Wgs84KineticClimbPredictor::GetVerticalPath() const {
   return m_vertical_predictor->GetVerticalPath();
}
}  // namespace aaesim