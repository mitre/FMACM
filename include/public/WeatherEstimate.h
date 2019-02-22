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

#include <memory>
#include "public/WeatherEstimate.h"
#include "public/Atmosphere.h"
#include "public/WindStack.h"

class Wind;

class WeatherEstimate
{
   friend class Wind_populate_predicted_wind_matrices_Test;

   friend class TrajectoryPredictor_updateWeatherPrediction_Test;

   friend class TrajectoryPredictor_startAltitudeInDescentAltList_Test;

   friend class TrajectoryPredictor_startAndEndAltitudeInDescentAltList_Test;

public:
   WindStack east_west;
   WindStack north_south;

   std::shared_ptr<Wind> getWind() const;

   std::shared_ptr<Atmosphere> getAtmosphere() const;

protected:
   // Constructors are protected to prevent bare instantiation.
   // Callers should use WeatherPrediction or WeatherTruth.
   WeatherEstimate();

   WeatherEstimate(std::shared_ptr<Wind> wind,
                   std::shared_ptr<Atmosphere> atmosphere);

   virtual ~WeatherEstimate();

   std::shared_ptr<Wind> m_wind;
   std::shared_ptr<Atmosphere> m_atmosphere;
};

