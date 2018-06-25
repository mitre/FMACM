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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <Length.h>


// Class used to compute the closest point metric, which is used to determine
// closest point between the IM and target aircrafts.

class ClosestPointMetric
{
public:
  ClosestPointMetric(void);
  ~ClosestPointMetric(void);

  // Computes distance for input position and updates minimum
  // position if less than minimum distance.
  void update(double imx,double imy,double targx,double targy);

  Units::Length getMinDist();


private:

  // Minimum distance between IM and target aircraft.
  Units::Length mMinDist;

};
