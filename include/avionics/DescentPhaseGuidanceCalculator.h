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

#include "public/GuidanceCalculator.h"

#include "avionics/Wgs84HorizontalPathSegment.h"
#include "public/VerticalPath.h"
#include "public/Wgs84PrecalcWaypoint.h"
#include "avionics/Wgs84AlongPathDistanceCalculator.h"
#include "avionics/Wgs84PositionCalculator.h"
#include "avionics/Wgs84AlongPathDistanceMonitor.h"

namespace aaesim {

class DescentPhaseGuidanceCalculator : public aaesim::open_source::GuidanceCalculator {

  public:
   DescentPhaseGuidanceCalculator(
         std::tuple<std::vector<Wgs84HorizontalPathSegment>, VerticalPath, std::vector<Wgs84PrecalcWaypoint>>
               guidance_path,
         Units::Length mach_cas_transition_altitude_msl, double mach);
   ~DescentPhaseGuidanceCalculator() = default;
   open_source::Guidance Update(const open_source::AircraftState &current_state) override;

  private:
   static log4cplus::Logger m_logger;
   open_source::Guidance ComputeHorizontalGuidance(
         const open_source::AircraftState &current_state,
         const aaesim::avionics::Wgs84AlongPathDistanceMonitor::ResultData &result_data);
   open_source::Guidance ComputeVerticalGuidance(
         const open_source::AircraftState &current_state,
         const aaesim::avionics::Wgs84AlongPathDistanceMonitor::ResultData &result_data) const;

   std::vector<Wgs84HorizontalPathSegment> m_horizontal_guidance_path;
   VerticalPath m_vertical_guidance_path;
   Units::Length m_mach_cas_transition_altitude_msl;
   double m_mach_at_transition;
   std::shared_ptr<aaesim::avionics::Wgs84AlongPathDistanceMonitor> m_distance_calculator{};
   aaesim::Wgs84PositionCalculator m_position_calculator;
   aaesim::open_source::Guidance m_previous_guidance{};
};
}  // namespace aaesim