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
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <scalar/Length.h>
#include <scalar/Speed.h>

class NIC {
  public:
   NIC() = default;
   ~NIC() = default;

   static Units::FeetLength nicp_to_contain_horizontal(int nicp) {
      // TODO: put table here

      Units::FeetLength contain(0);
      return contain;
   };

   static Units::FeetLength nicp_to_contain_vertical(int nicp) {
      // TODO: put table here

      Units::FeetLength contain(0);
      return contain;
   };

   static Units::FeetLength nicp_to_sigma_horizontal(int nicp) {
      // TODO: put table here

      Units::FeetLength sigma(0);
      return sigma;
   };

   static Units::FeetPerSecondSpeed nicv_to_contain_horizontal(int nicv) {
      // TODO: put table here

      Units::FeetPerSecondSpeed contain(0);
      return contain;
   };

   static Units::FeetPerSecondSpeed nicv_to_contain_vertical(int nicv) {
      // TODO: put table here

      Units::FeetPerSecondSpeed contain(0);
      return contain;
   };
};
