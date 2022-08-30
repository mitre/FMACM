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

#include <stdexcept>

namespace aaesim {
   namespace open_source {
      namespace bada_utils {

         enum FlapConfiguration {
            UNDEFINED = -1,
            TAKEOFF = 0,
            INITIAL_CLIMB = 1,
            CRUISE = 2,
            APPROACH = 3,
            LANDING = 4,
            GEAR_DOWN = 5
         };

         static std::string GetFlapConfigurationAsString(FlapConfiguration flap_configuration) {
            switch (flap_configuration) {
               case UNDEFINED:
                  return "UNDEFINED";
               case TAKEOFF:
                  return "TAKEOFF";
               case INITIAL_CLIMB:
                  return "INITIAL_CLIMB";
               case CRUISE:
                  return "CRUISE";
               case APPROACH:
                  return "APPROACH";
               case LANDING:
                  return "LANDING";
               case GEAR_DOWN:
                  return "GEAR_DOWN";
               default:
                  throw std::runtime_error("Invalid flap_configuration encountered: " + flap_configuration);
            }
         }

         enum EngineThrustMode {
            MAXIMUM_CLIMB = 0,
            MAXIMUM_CRUISE,
            DESCENT
         };

      }
   }
}