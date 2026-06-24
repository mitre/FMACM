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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

/* OldCustomMath.h		Initial code from Survsim 2.00R1  11/2/99*/

#pragma once

#include <log4cplus/logger.h>
#include <scalar/Angle.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Time.h>

#include "public/DMatrix.h"
#include "public/DVector.h"

/**
 * This contains the version of Math/CustomMath before Math/RandomGenerator
 * was implemented.  The sampling functions that were in CustomMath are
 * replaced by similar sampling code in RandomGenerator and removed from
 * CustomMath.  The CustomMath functions are all wrapped inside the class
 * here on 19 Nov 15.  The sampling methods saved here are used in some
 * of the tests.
 **/
namespace aaesim {
namespace test {
namespace utils {
class OldCustomMath {
  public:
   static log4cplus::Logger logger;

   double atan3(double x,
                double y);        // arc tangent from 0 - 2pi
   double uniform(double &seed);  // seed should not be 0.0
   double gauss(double mean, double sigma, double &seed);

   double trunc_gauss(double mean, double sigma, double max_std_dev, double &seed);

   double Rayleigh(double mean, double sigma, double &seed);

   Units::Time Rayleigh(Units::Time mean, Units::Time sigma, double &seed);

   double laplace(double lambda,
                  double &seed);  // returns laplacian rv given lambda
   double quantize(double value,
                   double lsb);  // quantizes value to lsb
   Units::Length quantize(Units::Length value, Units::Length lsb);

   Units::Speed quantize(Units::Speed value, Units::Speed lsb);

   bool hit(double probability, double &seed);

   double MachToCas_MITRE(double mach, double alt);

   double subtract_headings(double hd1, double hd2);

   double MachToTas(double mach, double altitude);

   // for stereographic convertion:

   bool inverse(DMatrix &in, int n, DMatrix &inverse);

   void matrix_times_vector(DMatrix &matrix_in, DVector &vector_in, int n, DVector &vector_out);

   DMatrix &createRotationMatrix(double l, double m, double n, const Units::Angle theta);
};
}  // namespace utils
}  // namespace test
}  // namespace aaesim
