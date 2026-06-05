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
// (c) 2026 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <string>
#include <vector>

#include "public/AircraftState.h"
#include "public/OutputHandler.h"

namespace fmacm {
class AircraftStateWriter final : public OutputHandler {
  public:
   AircraftStateWriter() : OutputHandler("", "_AcStates.csv"), m_data_to_write() {}
   void Finish() override;
   void Gather(std::vector<aaesim::open_source::AircraftState> aircraft_states);

  private:
   static std::vector<std::string> COLUMN_NAMES;
   struct DataToWrite {
      DataToWrite() = default;
      ~DataToWrite() = default;

      Units::Time simulation_time{Units::NegInfinity()};
      Units::Speed dynamics_ias{Units::NegInfinity()};
      Units::Speed dynamics_tas{Units::NegInfinity()};
      Units::Speed dynamics_altitude_rate{Units::NegInfinity()};
      Units::Length euclidean_x{Units::NegInfinity()};
      Units::Length euclidean_y{Units::NegInfinity()};
      Units::Length altitude_msl{Units::NegInfinity()};
      Units::Speed dynamics_ground_speed{Units::NegInfinity()};
      Units::DegreesAngle latitude{0}, longitude{0};
   };

   std::vector<DataToWrite> m_data_to_write;
};
}  // namespace fmacm
