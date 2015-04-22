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

#include "AirborneApplication.h"
#include "constants.h"
#include "AircraftCalculations.h"

using namespace std;

AirborneApplication::AirborneApplication(void) 
{
	loaded = false;
	application_type = "speed_commands_from_file";
}

AirborneApplication::~AirborneApplication(void)
{
}

Guidance AirborneApplication::update(ThreeDOFDynamics *dynamics,AircraftState state_in,
	//ADSBEther ads_b_in,
	Guidance guidance_in, DMatrix &wind_x, DMatrix &wind_y,
	double &seed)
{
	// Main update method computing guidance based on particular airborne application from runfile.
	// The Interval Management models have checks to ensure they are loaded properly and that
	// they have the correct trajectory data.
	//
	// dynamics:aircraft dynamimcs model.
	// state_in:current state of IM aircraft.
	// ads_b_in:received ADS B reports list.
	// own_intent:intent of IM aircraft.
	// target_intent:intent of target aircraft.
	// guidance_in:input guidance of IM aircraft.
	// wind_x, wind_y:wind data (altitude in feet, wind speed in knots).
	// seed:current seed for use in functions needing a random number.
	// output:data store and output for metrics collected on each IM algorithm update call.
	// 
	// returns updated guidance from Interval Management update method, (mainly ias in FPS).
	
	Guidance guidance_out = guidance_in; // initialize output guidance to given guidance
	Guidance imGuidance;

	double time = state_in.time; // sets the current time to the give state time

//	Retrieve the IM speed from a test vector file.
	imGuidance = mIMSpeedCommandFile.update(Units::SecondsTime(time));
	guidance_out.indicated_airspeed = imGuidance.indicated_airspeed;
	guidance_out.setValid(imGuidance.is_valid());

	return guidance_out;
}

bool AirborneApplication::load(DecodedStream *input)
{
  set_stream(input); 


  //register variable for loading:
  register_var("application_type", &application_type, true);
  register_loadable_with_brackets("IM_speed_commands_from_file", &mIMSpeedCommandFile, true);

  //do the actual reading:
  loaded = complete();

  if (application_type != "speed_commands_from_file")
  {
	  cout << "invalid application_type value: " << application_type << ". Must be speed_commands_from_file." << endl;
  }
  return loaded;
}

bool AirborneApplication::is_loaded()
{
	return loaded;
}
