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

#include "public/OutputHandler.h"
#include "public/AircraftState.h"

namespace fmacm {
class AircraftStateWriter final : public OutputHandler {
  public:
   AircraftStateWriter() : OutputHandler("", "_AcStates.csv"), m_data_to_write() {}
   void Finish() override;
   void Gather(std::vector<aaesim::open_source::AircraftState> aircraft_states);

  private:
   static std::vector<std::string> COLUMN_NAMES;
   struct DataToWrite {
      DataToWrite() {
         simulation_time = Units::NegInfinity();
         dynamics_ias = Units::NegInfinity();
         dynamics_tas = Units::NegInfinity();
         dynamics_altitude_rate = Units::NegInfinity();
         euclidean_x = Units::NegInfinity();
         euclidean_y = Units::NegInfinity();
         altitude_msl = Units::NegInfinity();
         dynamics_ground_speed = Units::NegInfinity();
         latitude = Units::DegreesAngle(0);
         longitude = Units::DegreesAngle(0);
      }
      ~DataToWrite() = default;

      Units::Time simulation_time;
      Units::Speed dynamics_ias;
      Units::Speed dynamics_tas;
      Units::Speed dynamics_altitude_rate;
      Units::Length euclidean_x;
      Units::Length euclidean_y;
      Units::Length altitude_msl;
      Units::Speed dynamics_ground_speed;
      Units::DegreesAngle latitude, longitude;
   };

   std::vector<DataToWrite> m_data_to_write;
};
}  // namespace fmacm
