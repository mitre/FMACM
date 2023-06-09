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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/Aircraft.h"
#include "framework/TestFrameworkApplication.h"
#include "framework/WeatherTruthByTime.h"
#include "public/AircraftControl.h"
#include "public/AircraftState.h"
#include "public/SimulationTime.h"
#include "public/EuclideanThreeDofDynamics.h"
#include "framework/TrajectoryFromFile.h"
#include "public/WeatherPrediction.h"
#include "framework/AircraftIntentFromFile.h"
#include "framework/TestFrameworkFMS.h"
#include "math/DMatrix.h"
#include "loader/DecodedStream.h"
#include <queue>
#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include <scalar/Speed.h>
#include <scalar/Length.h>

class TestFrameworkAircraft : public aaesim::open_source::Aircraft {
  public:
   TestFrameworkAircraft();

   virtual ~TestFrameworkAircraft();

   /**
    * Override the base class loader
    */
   bool load(DecodedStream *input);

   bool Update(const SimulationTime &time);

   void PostLoad(Units::Time simulation_time_step, int predicted_wind_opt, bool blend_wind, WeatherTruth weather_truth);

   void Initialize(Units::Length adsbReceptionRangeThreshold, const WeatherTruth &weather);

   aaesim::open_source::DynamicsState GetCurrentDynamicsState() const;

   int m_start_time;
   int m_id;
   aaesim::open_source::AircraftState m_truth_state_vector_old;
   AircraftIntentFromFile m_aircraft_intent;
   aaesim::open_source::EuclideanThreeDofDynamics m_dynamics;
   TrajectoryFromFile m_precalc_traj;
   TestFrameworkApplication m_airborne_app;
   TestFrameworkFMS m_fms;
   std::shared_ptr<aaesim::open_source::AircraftControl> m_aircraft_control;
   std::string m_ac_type;
   std::shared_ptr<aaesim::BadaPerformanceCalculator> m_bada_calculator;

  private:
   void InitTruthStateVectorOld();

   double CalculateEndTime(aaesim::open_source::AircraftState state_in);

   Units::FeetLength m_initial_altitude;
   Units::KnotsSpeed m_initial_ias;
   double m_initial_mach;

   std::shared_ptr<WeatherTruthByTime> m_weather_truth;

   static log4cplus::Logger m_logger;
};

inline aaesim::open_source::DynamicsState TestFrameworkAircraft::GetCurrentDynamicsState() const {
   return m_dynamics.GetDynamicsState();
}