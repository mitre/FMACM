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

class IMCommandObserver
{
public:
   IMCommandObserver(void);

   ~IMCommandObserver(void);

   // operator < for sort algorithm
   bool operator<(const IMCommandObserver &im_in) const;

   int iteration;
   double id;
   double time;
   double distance_to_go;
   double state_altitude;
   double state_TAS;
   double state_groundspeed;
   double IAS_command;
   double unmodified_IAS;
   double TAS_command;
   double reference_velocity;
   double reference_distance;
   double predictedDistance;
   double distance_difference;
   double trueDistance;
};
