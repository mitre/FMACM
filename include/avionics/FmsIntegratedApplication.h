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

#include "public/FlightDeckApplication.h"
#include "avionics/Wgs84KineticDescentPredictor.h"

namespace aaesim {
namespace avionics {

struct FmsApplicationInitializer {
   virtual ~FmsApplicationInitializer() = default;
   aaesim::open_source::OwnshipFmsPredictionParameters fms_prediction_parameters;
   aaesim::open_source::OwnshipPerformanceParameters performance_parameters;
};

struct FmsIntegratedApplication {
   virtual void Initialize(FmsApplicationInitializer &initializer_visitor) = 0;
   virtual aaesim::open_source::Guidance Update(const aaesim::open_source::SimulationTime &sim_time,
                                                const aaesim::open_source::Guidance &prev_guidance,
                                                const aaesim::open_source::AircraftState &aircraft_state) = 0;
};
}  // namespace avionics
}  // namespace aaesim
