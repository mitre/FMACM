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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "public/Aircraft.h"
#include "TestFrameworkApplication.h"
#include "public/AircraftControl.h"
#include "public/AircraftState.h"
#include "public/SimulationTime.h"
#include "TestFrameworkDynamics.h"
#include "TrajectoryFromFile.h"
#include "public/WeatherPrediction.h"
#include "AircraftIntentFromFile.h"
#include "TestFrameworkFMS.h"
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

   TestFrameworkAircraft(const TestFrameworkAircraft &in);

   TestFrameworkAircraft &operator=(const TestFrameworkAircraft &in);

   /**
    * Override the base class loader
    */
   bool load(DecodedStream *input);

   bool Update(const SimulationTime &time);

   void PostLoad(Units::Time simulation_time_step,
                  int predicted_wind_opt,
                  bool blend_wind);

   void Initialize(Units::Length adsbReceptionRangeThreshold,
		   const WeatherTruth &weather);

   //Other Member Data:
   int m_start_time;
   int m_id;
   AircraftState m_truth_state_vector_old;
   AircraftIntentFromFile m_aircraft_intent;
   TestFrameworkDynamics m_dynamics;
   AircraftIntent m_target_intent_not_used; // NOT USED, but some signatures require it still
   TrajectoryFromFile m_precalc_traj;
   TestFrameworkApplication m_airborne_app;
   TestFrameworkFMS m_fms;
   AircraftControl m_aircraft_control;   // for FMS

private:
   void InitTruthStateVectorOld();

   // helper method to calculate the end time of the aircraft
   double CalculateEndTime(AircraftState state_in);

   void Copy(const TestFrameworkAircraft &in);

   // Data Members
   bool m_is_finished;

   // Wind data, setup as 5x2 matrices.
   // 1st Column:altitudes in feet.
   // 2nd Column:wind speed in feet/sec.
   //	DMatrix mPredictedWindX;
   //	DMatrix mPredictedWindY;
   // WeatherPrediction mPredictedWind;

   // Initialization loadables
   Units::FeetLength m_initial_altitude;
   Units::KnotsSpeed m_initial_ias;
   double m_initial_mach; // default 0.78

   // std::shared_ptr<AircraftControl> mAircraftControl;
   //	static FILE *mGuidanceOut;
};

