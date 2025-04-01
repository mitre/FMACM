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

#include "utility/BoundedValue.h"
#include "utility/UtilityConstants.h"
#include "scalar/Time.h"
#include "scalar/Speed.h"
#include "scalar/Length.h"
#include "public/AircraftState.h"
#include "public/SimulationTime.h"
#include "public/VerticalPath.h"
#include "avionics/Wgs84HorizontalPathSegment.h"

namespace required_time_of_arrival {
struct EtaMinimizerOutputData {
   Units::Speed delta_ias{Units::negInfinity()};
   Units::Time recomputed_eta_error{Units::negInfinity()};
   bool eta_minimization_is_valid{false};
   Units::Speed unlimited_toac_ias_command{Units::negInfinity()};
   Units::Speed limited_toac_ias_command{Units::negInfinity()};
   bool is_mach_solution{false};
   BoundedValue<double, 0, 2> unlimited_toac_mach_command{0};
   BoundedValue<double, 0, 2> limited_toac_mach_command{0};
   BoundedValue<double, -1, 1> delta_mach{0};
   VerticalPath active_vertical_guidance{};
   std::vector<aaesim::Wgs84HorizontalPathSegment> active_horizontal_guidance{};
};

struct RtaGoalSolver {
   virtual EtaMinimizerOutputData Solve(const aaesim::open_source::SimulationTime &simtime,
                                        const aaesim::open_source::AircraftState &aircraftstate,
                                        Units::Length distance_to_rta_fix) = 0;

   virtual bool EvaluateIsAble(const Units::Length &distance_to_rta_fix,
                               const aaesim::open_source::SimulationTime &sim_time,
                               const aaesim::open_source::AircraftState &current_state) = 0;
};
}  // namespace required_time_of_arrival