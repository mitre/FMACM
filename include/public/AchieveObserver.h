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

#include <string>
#include <scalar/Length.h>
#include <scalar/Time.h>

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
   Units::SecondsTime m_time;
   Units::Time m_targ_ttg_to_ach;
   Units::Time m_own_ttg_to_ach;
   Units::Length m_curr_dist;
   Units::Length m_ref_dist;
};
