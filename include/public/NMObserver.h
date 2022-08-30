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

