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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

/* CustomMath.h		Initial code from Survsim 2.00R1  11/2/99*/


#pragma once

#include <Angle.h>
#include <Length.h>
#include <Speed.h>
#include <Time.h>
#include "math/DMatrix.h"
#include "math/DVector.h"

// TODO move global C-style math functions into the class
class CustomMath
{
public:

};

#ifndef _LINUX_
int roundToInt(double d); /* rounds double to int-away from 0 for 0.5 values */
#endif

double atan3(double x,
             double y); // arc tangent from 0 - 2pi
double quantize(double value,
                double lsb);  // quantizes value to lsb

Units::Length quantize(Units::Length value,
                       Units::Length lsb);

Units::Speed quantize(Units::Speed value,
                      Units::Speed lsb);

Units::Time quantize(Units::Time value,
                     Units::Time lsb);

double MachToCas_MITRE(double mach,
                       double alt);

double subtract_headings(double hd1,
                         double hd2);

double MachToTas(double mach,
                 double altitude);

//for stereographic convertion:

bool inverse(DMatrix &in,
             int n,
             DMatrix &inverse);

void matrix_times_vector(DMatrix &matrix_in,
                         DVector &vector_in,
                         int n,
                         DVector &vector_out);

DMatrix &createRotationMatrix(double l,
                              double m,
                              double n,
                              const Units::Angle theta);
