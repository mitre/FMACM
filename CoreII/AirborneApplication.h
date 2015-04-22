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
// Copyright 2015 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once
#include "AircraftState.h"
#include "AircraftIntent.h"
#include "IMSpeedCommandFile.h"
#include "Loadable.h"
#include "Guidance.h"
#include "ThreeDOFDynamics.h"
#include <string>
#include <vector>
#include <map>

class AirborneApplication : public Loadable
{
 public:
  AirborneApplication (void);
  ~AirborneApplication (void);

  Guidance update(ThreeDOFDynamics *dynamics, AircraftState state_in,
	  // ADSBEther ads_b_in,
	  Guidance guidance_in, DMatrix &wind_x,
	  DMatrix &wind_y, double &seed);

  bool load(DecodedStream *input);

  // Conditionals
  bool is_loaded();

private:

  // Interval Management classes
  IMSpeedCommandFile mIMSpeedCommandFile;

  //Input Data
  std::string application_type;

  bool loaded;

};
