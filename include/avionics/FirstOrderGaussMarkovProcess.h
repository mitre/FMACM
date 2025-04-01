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

/* stat_vector.h		Initial code from Survsim 2.00R1  11/2/99

      REVISION HISTORY:
        DATE				INITIALS				REASON



*/

#pragma once

#include "public/Logging.h"

class FirstOrderGaussMarkovProcess {
  public:
   FirstOrderGaussMarkovProcess();

   ~FirstOrderGaussMarkovProcess(void);

   double update(double a, double sigma_u);

   void init();

  private:
   static log4cplus::Logger logger;

   bool stationary;
   double a;
   double sigma_u;
   double previous;
   int counter;
};
