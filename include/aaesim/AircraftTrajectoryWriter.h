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

#include "aaesim/AircraftDataIterationWriter.h"

#include <string>
#include <vector>
#include <scalar/Time.h>
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Angle.h>
#include <scalar/Temperature.h>

#include "public/Logging.h"
#include "public/AircraftState.h"
#include "public/SimulationTime.h"
#include "aaesim/AircraftEntity.h"

namespace aaesim {
class AircraftTrajectoryWriter final : public AircraftDataIterationWriter {

  public:
   AircraftTrajectoryWriter();

   void Finish() override;

   void Gather(const int &iteration_number, const aaesim::AircraftEntity &aircraft) override;

  private:
   struct SimData {
      SimData()
         : iteration_number(INT32_MIN),
           simulation_time(Units::NegInfinity()),
           acid(""),
           dynamics_ias(Units::NegInfinity()),
           dynamics_tas(Units::NegInfinity()),
           euclidean_x(Units::NegInfinity()),
           euclidean_y(Units::NegInfinity()),
           altitude_msl(Units::NegInfinity()),
           enu_x_velocity(Units::NegInfinity()),
           enu_y_velocity(Units::NegInfinity()),
           enu_z_velocity(Units::NegInfinity()),
           dynamics_ground_speed(Units::NegInfinity()),
           true_wind_velocity_east(Units::NegInfinity()),
           true_wind_velocity_north(Units::NegInfinity()),
           ned_heading_true(Units::ZERO_ANGLE),
           wind_vx_dh(Units::NegInfinity()),
           wind_vy_dh(Units::NegInfinity()),
           ellipsoidal_latitude(Units::NegInfinity()),
           ellipsoidal_longitude(Units::NegInfinity()),
           fms_ias_command(Units::NegInfinity()),
           true_distance_to_go(Units::NegInfinity()),
           true_temperature(Units::AbsKelvinTemperature(0)),
           aircraft_selected_speed(-INFINITY),
           aircraft_selected_speed_type(UNSPECIFIED_SPEED),
           flap_setting(aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED),
           speed_brake(0),
           roll_angle(Units::NegInfinity()),
           mach_number(-INFINITY),
           mach_command(-INFINITY) {}

      int iteration_number;
      Units::Time simulation_time;
      std::string acid;
      Units::Speed dynamics_ias;
      Units::Speed dynamics_tas;
      Units::Length euclidean_x;
      Units::Length euclidean_y;
      Units::Length altitude_msl;
      Units::Speed enu_x_velocity;
      Units::Speed enu_y_velocity;
      Units::Speed enu_z_velocity;
      Units::Speed dynamics_ground_speed;
      Units::Speed true_wind_velocity_east;
      Units::Speed true_wind_velocity_north;
      Units::SignedDegreesAngle ned_heading_true;
      Units::Frequency wind_vx_dh;
      Units::Frequency wind_vy_dh;
      Units::Angle ellipsoidal_latitude;
      Units::Angle ellipsoidal_longitude;
      Units::Speed fms_ias_command;
      Units::Length true_distance_to_go;
      Units::AbsTemperature true_temperature;
      double aircraft_selected_speed;
      SpeedValueType aircraft_selected_speed_type;
      aaesim::open_source::bada_utils::FlapConfiguration flap_setting;
      double speed_brake;
      Units::Angle roll_angle;
      double mach_number{-INFINITY};
      double mach_command{-INFINITY};
   };

   std::vector<SimData> m_data_to_write;

   static log4cplus::Logger m_logger;
};
}  // namespace aaesim
