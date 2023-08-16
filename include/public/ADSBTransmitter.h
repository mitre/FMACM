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

#include <vector>

#include "public/ADSBSVReport.h"
#include "public/SimulationTime.h"
#include "public/AircraftState.h"

namespace aaesim {
namespace open_source {
struct ADSBTransmitter {
   virtual const std::vector<Sensor::ADSB::ADSBSVReport> &GetAllTransmissions() const = 0;
   virtual void Initialize() = 0;
   virtual void Transmit(const aaesim::open_source::SimulationTime &simulation_time,
                         const aaesim::open_source::AircraftState &nav_measurement) = 0;
};
}  // namespace open_source
}  // namespace aaesim
