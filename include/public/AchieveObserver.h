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

#include <string>
#include <Length.h>
#include <Time.h>

// Holds data used to produce time to go metrics for the achieve by algorithms.

class AchieveObserver
{
public:
   AchieveObserver();

   AchieveObserver(const int iter,
                   const int aircraft_id,
                   const double tm,
                   const double target_ttg_to_ach,
                   const double own_ttg_to_ach,
                   const double curr_distance,
                   const double reference_distance);

   ~AchieveObserver();

   const std::string Hdr();

   std::string ToString();

private:
   int m_iteration;
   int m_id; // aircraft id
   Units::Time m_time;
   Units::Time m_targ_ttg_to_ach;
   Units::Time m_own_ttg_to_ach;
   Units::Length m_curr_dist;
   Units::Length m_ref_dist;
};
