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

#include "scalar/Time.h"

#include "public/Logging.h"
#include "public/BadaUtils.h"
#include "bada/BadaPerformanceData.h"

namespace aaesim {
namespace bada {
class FlapState final {
  public:
   FlapState(std::shared_ptr<const BadaPerformanceData> bada_data,
             const aaesim::open_source::bada_utils::FlapConfiguration initial_flap_configuration);
   void UpdateTargetFlapConfiguration(
         const aaesim::open_source::bada_utils::FlapConfiguration target_flap_configuration);
   void Update();
   void GetDragCoefficients(double &cd0, double &cd2) const;

   aaesim::open_source::bada_utils::FlapConfiguration GetTargetFlapConfiguration() const {
      return target_flap_configuration_;
   }

  private:
   static log4cplus::Logger logger_;
   static const Units::SecondsTime kDefaultFlapTravelTime;

   const Units::SecondsTime full_flap_travel_time_;
   std::shared_ptr<const BadaPerformanceData> bada_data_;
   aaesim::open_source::bada_utils::FlapConfiguration target_flap_configuration_;
   double target_flap_position_;
   double current_flap_position_;
   double positions_by_configuration_[aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN + 1];
   std::map<double, double> descent_drag_map_;  // map of cd0 to cd2 during descent
   std::map<double, double> climb_drag_map_;    // map of cd0 to cd2 during climb

   static double Interpolate(const std::map<double, double> &x_to_y_map, const double x);
   void Initialize();
   void SetInitialFlapConfiguration(
         const aaesim::open_source::bada_utils::FlapConfiguration initial_flap_configuration);
};
}  // namespace bada
}  // namespace aaesim
