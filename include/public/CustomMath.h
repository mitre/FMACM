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

#include <scalar/Angle.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Time.h>
#include "public/DMatrix.h"
#include "public/DVector.h"

double atan3(double x,
             double y);  // arc tangent from 0 - 2pi
double quantize(double value,
                double lsb);  // quantizes value to lsb

Units::Length quantize(Units::Length value, Units::Length lsb);

Units::Speed quantize(Units::Speed value, Units::Speed lsb);

Units::Time quantize(Units::Time value, Units::Time lsb);

double subtract_headings(double hd1, double hd2);

bool inverse(DMatrix &in, int n, DMatrix &inverse);

void matrix_times_vector(DMatrix &matrix_in, DVector &vector_in, int n, DVector &vector_out);

DMatrix &createRotationMatrix(double l, double m, double n, const Units::Angle theta);
