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

#include "aaesim/AircraftControlLoader.h"
#include "aaesim/NavigationSensorLoader.h"
#include "public/LoggingLoadable.h"
#include "avionics/FlightManagementSystem.h"
#include "aaesim/FmsApplicationLoader.h"
#include "public/SpeedOnPitchControl.h"
#include "public/SpeedOnThrustControl.h"
#include "aaesim/ClimbPredictorLoader.h"
#include "aaesim/DescentPredictorLoader.h"

namespace aaesim {
namespace loaders {
class FmsLoader final : public LoggingLoadable {

  public:
   FmsLoader();

   bool load(DecodedStream *input);

   std::shared_ptr<FlightManagementSystem> BuildFms();

   bool const IsLoaded() const;
   void InitializeFmsApplication(std::shared_ptr<FlightManagementSystem> fms);

  private:
   static log4cplus::Logger m_logger;

   std::shared_ptr<aaesim::open_source::PredictedWindEvaluator> m_predicted_wind_evaluator;
   bool m_allow_trajectory_updates;
   bool m_model_loaded;

   aaesim::loaders::AircraftControlLoader m_aircraft_control_loader;
   aaesim::loaders::NavigationSensorLoader m_navigation_sensor_loader;
   aaesim::loaders::ClimbPredictorLoader m_kinetic_climb_loader;
   aaesim::loaders::DescentPredictorLoader m_kinetic_descent_loader;
   aaesim::loaders::FmsApplicationLoader m_fms_application_loader;
};
}  // namespace loaders
}  // namespace aaesim
