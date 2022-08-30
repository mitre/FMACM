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

aaesim::open_source::Guidance TestFrameworkApplication::Update(const SimulationTime &simTime,
                                          aaesim::open_source::ThreeDOFDynamics &dynamics,
                                          const aaesim::open_source::AircraftState &state_in,
                                          const aaesim::open_source::Guidance &guidance_in) {
   aaesim::open_source::Guidance guidance_out = guidance_in;
   aaesim::open_source::Guidance im_guidance;

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
