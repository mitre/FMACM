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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <vector>
#include "public/NMObserverEntry.h"
#include "math/Statistics.h"

class NMObserver
{
public:
   NMObserver(void);

   ~NMObserver(void);

   // adds a new output entry to the Nautical Mile Observer
   void output_NM_values(double predictedDistance,
                         double trueDistance,
                         double time,
                         double currIAS,
                         double currGS,
                         double targetGS,
                         double minIAS,
                         double maxIAS,
                         double minTAS,
                         double maxTAS);

   std::vector<NMObserverEntry> entry_list;

   std::vector<double> predictedDistance;
   std::vector<double> trueDistance;
   std::vector<double> time;
   std::vector<Statistics> ac_IAS_stats;
   std::vector<Statistics> ac_GS_stats;
   std::vector<Statistics> target_GS_stats;
   std::vector<Statistics> min_IAS_stats;
   std::vector<Statistics> max_IAS_stats;

   int curr_NM;

   void initialize_stats();
};

