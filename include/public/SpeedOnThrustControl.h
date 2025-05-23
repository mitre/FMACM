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

#include "public/AircraftControl.h"

namespace aaesim {
namespace open_source {
class SpeedOnThrustControl : public AircraftControl {

  public:
   SpeedOnThrustControl(const Units::Angle max_bank_angle);

   void Initialize(std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> bada_calculator) override;

  private:
   static log4cplus::Logger m_logger;
   static Units::Frequency m_gain_altitude, m_gain_gamma, m_gain_phi;
   static double m_gain_speedbrake;
   Units::Frequency m_gain_velocity;
   unsigned int m_min_thrust_counter;
   unsigned int m_speedbrake_counter;
   bool m_is_speedbrake_on;

  protected:
   virtual void DoVerticalControl(const Guidance &guidance, const EquationsOfMotionState &equations_of_motion_state,
                                  Units::Force &thrust_command, Units::Angle &gamma_command, Units::Speed &tas_command,
                                  double &speed_brake_command,
                                  aaesim::open_source::bada_utils::FlapConfiguration &new_flap_configuration) override;
};
}  // namespace open_source
}  // namespace aaesim
