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

#include "framework/TestFrameworkApplication.h"
#include "utility/constants.h"
#include "public/AircraftCalculations.h"

using namespace std;

TestFrameworkApplication::TestFrameworkApplication() {
   m_loaded = false;
   m_application_type = "speed_commands_from_file";
}

TestFrameworkApplication::~TestFrameworkApplication() {
}

Guidance TestFrameworkApplication::Update(const SimulationTime &simTime,
                                          ThreeDOFDynamics &dynamics,
                                          const AircraftState &state_in,
                                          const Guidance &guidance_in) {
   Guidance guidance_out = guidance_in;
   Guidance im_guidance;

   double time = state_in.m_time;

   im_guidance = m_im_speed_command_file.Update(Units::SecondsTime(time));
   guidance_out.m_ias_command = im_guidance.m_ias_command;
   guidance_out.SetValid(im_guidance.IsValid());

   return guidance_out;
}

bool TestFrameworkApplication::load(DecodedStream *input) {
   set_stream(input);


   //register variable for loading:
   register_var("application_type", &m_application_type, false);
   register_loadable_with_brackets("IM_speed_commands_from_file", &m_im_speed_command_file, true);

   //do the actual reading:
   m_loaded = complete();

   if (m_application_type != "speed_commands_from_file") {
      cout << "invalid application_type value: " << m_application_type << ". Must be speed_commands_from_file." << endl;
   }
   return m_loaded;
}

bool TestFrameworkApplication::IsLoaded() {
   return m_loaded;
}
