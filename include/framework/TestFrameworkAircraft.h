// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/Aircraft.h"
#include "framework/TestFrameworkApplication.h"
#include "framework/WeatherTruthByTime.h"
#include "public/AircraftControl.h"
#include "public/AircraftState.h"
#include "public/SimulationTime.h"
#include "public/ThreeDOFDynamics.h"
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
#include "Speed.h"
#include "Length.h"

class TestFrameworkAircraft : public Aircraft
{
public:
   TestFrameworkAircraft();

   virtual ~TestFrameworkAircraft();

   /**
    * Override the base class loader
    */
   bool load(DecodedStream *input);

   bool Update(const SimulationTime &time);

   void PostLoad(Units::Time simulation_time_step,
                 int predicted_wind_opt,
                 bool blend_wind,
                 WeatherTruth weather_truth);

   void Initialize(Units::Length adsbReceptionRangeThreshold,
                   const WeatherTruth &weather);

   int m_start_time;
   int m_id;
   AircraftState m_truth_state_vector_old;
   AircraftIntentFromFile m_aircraft_intent;
   ThreeDOFDynamics m_dynamics;
   TrajectoryFromFile m_precalc_traj;
   TestFrameworkApplication m_airborne_app;
   TestFrameworkFMS m_fms;
   std::shared_ptr<AircraftControl> m_aircraft_control;
   std::string m_ac_type;
   BadaWithCalc m_bada_calculator;

private:
   void InitTruthStateVectorOld();

   double CalculateEndTime(AircraftState state_in);

   Units::FeetLength m_initial_altitude;
   Units::KnotsSpeed m_initial_ias;
   double m_initial_mach;

   std::shared_ptr<WeatherTruthByTime> m_weather_truth;

   static log4cplus::Logger m_logger;
};

