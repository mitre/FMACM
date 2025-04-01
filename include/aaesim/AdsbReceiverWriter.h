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
class AdsbReceiverWriter final : public AircraftDataIterationWriter {
  public:
   AdsbReceiverWriter();
   void Finish() override;
   void Gather(const int &iteration_number, const aaesim::AircraftEntity &aircraft) override;

  private:
   static std::vector<std::string> COLUMN_NAMES;
   struct SimData {
      SimData()
         : iteration_number(INT32_MIN),
           simulation_time(Units::NegInfinity()),
           receiver_acid(""),
           transmitter_acid(""),
           latitude(Units::NegInfinity()),
           longitude(Units::NegInfinity()),
           altitude(Units::NegInfinity()),
           ew_magnitude(Units::NegInfinity()),
           ns_magnitude(Units::NegInfinity()),
           nacp(INT32_MAX),
           nic(INT32_MAX),
           nacv(INT32_MAX),
           sil(2),
           sda(2),
           vertical_rate(Units::NegInfinity()) {}
      int iteration_number;
      Units::Time simulation_time;
      std::string receiver_acid;
      std::string transmitter_acid;
      Units::Angle latitude;
      Units::Angle longitude;
      Units::Length altitude;
      Units::Speed ew_magnitude;
      Units::Speed ns_magnitude;
      int nacp;
      int nic;
      int nacv;
      int sil;
      int sda;
      Units::Speed vertical_rate;
   };

   std::vector<SimData> m_data_to_write;
};
}  // namespace aaesim
