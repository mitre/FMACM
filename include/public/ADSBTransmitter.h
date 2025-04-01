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
#include "public/Waypoint.h"

namespace aaesim {
namespace open_source {

static const Units::FeetLength ADS_B_HOR_POS_QUANT(7.83);
static const Units::FeetPerSecondSpeed ADS_B_HOR_VEL_QUANT(1.68);
static const Units::FeetLength ADS_B_VER_POS_QUANT(25.);
static const Units::FeetPerSecondSpeed ADS_B_VER_VEL_QUANT(3.5);
static const Units::FeetPerSecondSpeed CPR_SPD_QUANT(1.68);
static const Units::FeetLength CPR_LONG_QUANT(16.69);
static const Units::FeetLength CPR_LAT_QUANT(16.69);
static const Units::FeetPerSecondSpeed CPR_Z_RATE_QUANT(0.490);
static const Units::FeetLength CPR_ALT_QUANT(25.0);

struct ADSBTransmitter {

   virtual void Initialize(const std::list<Waypoint> &waypoints_along_route) = 0;
   virtual void Transmit(const aaesim::open_source::SimulationTime &simulation_time,
                         const aaesim::open_source::AircraftState &nav_measurement) = 0;
   virtual const std::vector<aaesim::open_source::ADSBSVReport> &GetAllTransmissions() const = 0;
};
}  // namespace open_source
}  // namespace aaesim
