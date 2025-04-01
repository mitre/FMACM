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

#include <string>
#include <vector>

#include "aaesim/AircraftDataIterationWriter.h"
#include "scalar/Time.h"

namespace aaesim {
class AdsbTransmitterWriter final : public AircraftDataIterationWriter {
  public:
   AdsbTransmitterWriter();
   void Finish() override;
   void Gather(const int &iteration_number, const aaesim::AircraftEntity &aircraft) override;

  private:
   static std::vector<std::string> COLUMN_NAMES;
   struct SimData {
      SimData() {
         iteration_number = INT32_MIN;
         transmitter_acid = "";
         transmission_time = Units::NegInfinity();
         latitude = Units::NegInfinity();
         longitude = Units::NegInfinity();
         enu_position_x = Units::NegInfinity();
         enu_position_y = Units::NegInfinity();
         altitude_msl = Units::NegInfinity();
         enu_velocity_x = Units::NegInfinity();
         enu_velocity_y = Units::NegInfinity();
         nacp = INT32_MIN;
         nicp = INT32_MIN;
         nacv = INT32_MIN;
         vertical_speed = Units::NegInfinity();
      }
      int iteration_number;
      Units::Time transmission_time;
      std::string transmitter_acid;
      Units::Angle latitude;
      Units::Angle longitude;
      Units::Length enu_position_x;
      Units::Length enu_position_y;
      Units::Length altitude_msl;
      Units::Speed enu_velocity_x;
      Units::Speed enu_velocity_y;
      int nacp;
      int nicp;
      int nacv;
      Units::Speed vertical_speed;
   };

   std::vector<SimData> m_data_to_write;
};
}  // namespace aaesim
