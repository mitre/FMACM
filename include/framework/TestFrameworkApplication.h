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

#include "public/AircraftState.h"
#include "public/AircraftIntent.h"
#include "framework/IMSpeedCommandFile.h"
#include "loader/Loadable.h"
#include "public/Guidance.h"
#include "public/SimulationTime.h"
#include "public/ThreeDOFDynamics.h"
#include <string>
#include <vector>
#include <map>

class TestFrameworkApplication : public Loadable {
  public:
   TestFrameworkApplication();

   ~TestFrameworkApplication();

   aaesim::open_source::Guidance Update(const SimulationTime &simTime, aaesim::open_source::ThreeDOFDynamics &dynamics,
                                        const aaesim::open_source::AircraftState &state_in,
                                        const aaesim::open_source::Guidance &guidance_in);

   bool load(DecodedStream *input);

   // Conditionals
   bool IsLoaded();

  private:
   // Interval Management classes
   IMSpeedCommandFile m_im_speed_command_file;

   // Input Data
   std::string m_application_type;

   bool m_loaded;
};
