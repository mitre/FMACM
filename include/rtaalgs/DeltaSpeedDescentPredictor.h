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

#include "avionics/Wgs84KineticDescentPredictor.h"
#include "rtaalgs/AircraftIntentMutator.h"
namespace required_time_of_arrival {
class DeltaSpeedDescentPredictor : public aaesim::Wgs84KineticDescentPredictor {
  public:
   DeltaSpeedDescentPredictor(std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> predictor,
                              const Units::Speed delta_ias);

   DeltaSpeedDescentPredictor(std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> predictor,
                              const BoundedValue<double, -1, 1> &delta_mach);

   void CalculateWaypoints(const AircraftIntent &aircraft_intent,
                           const aaesim::open_source::WeatherPrediction &weather_prediction);

   void BuildTrajectoryPrediction(const std::string &acPerfName, aaesim::open_source::WeatherPrediction &weather,
                                  Units::Length start_altitude);

   const std::vector<aaesim::Wgs84HorizontalPathSegment> EstimateHorizontalTrajectory(
         aaesim::open_source::WeatherPrediction weather_prediction);

   std::shared_ptr<Wgs84KineticDescent4DPredictor> GetVertPredictor() const;

   const std::string &GetAircraftType() const;

   const AircraftIntent &GetAircraftIntent() const override;

   const std::vector<aaesim::Wgs84HorizontalPathSegment> &GetHorizontalPath() const override;

   const std::shared_ptr<Atmosphere> GetAtmosphere() const;

   const std::vector<aaesim::Wgs84PrecalcWaypoint> &GetPrecalcWaypoints() const override;

   Units::Angle GetBankAngle() const;

   Units::Length GetAltitudeAtFinalWaypoint() const;

   void SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere);

   const VerticalPath GetVerticalPath() const override;

  private:
   std::shared_ptr<aaesim::Wgs84KineticDescentPredictor> m_delegate;
   std::shared_ptr<AircraftIntentMutator> m_aircraft_intent_function;
};
}  // namespace required_time_of_arrival