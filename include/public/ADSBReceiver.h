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

#include <map>
#include <vector>
#include "scalar/Length.h"
#include "public/SimulationTime.h"
#include "public/ADSBSVReport.h"

namespace aaesim {
namespace open_source {
struct ADSBReceiver {
   virtual Sensor::ADSB::ADSBSVReport GetCurrentADSBReport(int id) const = 0;
   virtual Sensor::ADSB::ADSBSVReport GetADSBReportBefore(int id, double time) const = 0;
   virtual std::vector<Sensor::ADSB::ADSBSVReport> GetReportsReceivedByTime(const SimulationTime &time) const = 0;
};
}  // namespace open_source
}  // namespace aaesim
