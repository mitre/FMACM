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

#include <cstdio>
#include "public/AchieveObserver.h"

using namespace std;

AchieveObserver::AchieveObserver()
      :
      m_iteration(-1),
      m_id(-1) {
   m_time = Units::SecondsTime(-99999.0);
   m_targ_ttg_to_ach = Units::SecondsTime(-99999.0);
   m_own_ttg_to_ach = Units::SecondsTime(-99999.0);
   m_curr_dist = Units::MetersLength(-99999.0);
   m_ref_dist = Units::MetersLength(-99999.0);
}


AchieveObserver::AchieveObserver(const int iter,
                                 const int aircraft_id,
                                 const double tm,
                                 const double target_ttg_to_ach,
                                 const double own_ttg_to_ach,
                                 const double curr_distance,
                                 const double reference_distance)
      :
      m_iteration(iter),
      m_id(aircraft_id),
      m_time(tm) {
   m_targ_ttg_to_ach = Units::SecondsTime(target_ttg_to_ach);
   m_own_ttg_to_ach = Units::SecondsTime(own_ttg_to_ach);
   m_curr_dist = Units::MetersLength(curr_distance);
   m_ref_dist = Units::MetersLength(reference_distance);
}


AchieveObserver::~AchieveObserver() {
   // Destructor.
}


const std::string AchieveObserver::Hdr() {
   // Creates output header for csv file output.

   const string str = "Iteration,AircrafId,Time(s),Targ_TTG_to_Ach(s),Own_TTG_to_Ach(s),CurrDistance(m),RefDistance(m)";

   return str;
}


string AchieveObserver::ToString() {
   // Creates string of object for csv file output.

   string str;

   char *txt = new char[301];

   sprintf(txt, "%d,%d,%lf,%lf,%lf,%lf,%lf",
           m_iteration, m_id,
           Units::SecondsTime(m_time).value(),
           Units::SecondsTime(m_targ_ttg_to_ach).value(),
           Units::SecondsTime(m_own_ttg_to_ach).value(),
           Units::MetersLength(m_curr_dist).value(),
           Units::MetersLength(m_ref_dist).value());

   str = txt;

   delete[] txt;

   return str;
}
