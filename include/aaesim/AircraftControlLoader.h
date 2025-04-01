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

#include "public/LoggingLoadable.h"
#include "public/AircraftControl.h"

namespace aaesim {
namespace loaders {
class AircraftControlLoader final : public LoggingLoadable {
  public:
   AircraftControlLoader();
   ~AircraftControlLoader() = default;

   bool load(DecodedStream *input) override;

   bool IsLoaded() const;

   std::shared_ptr<aaesim::open_source::AircraftControl> BuildAircraftControlModel();

  private:
   enum SpeedControlType { THRUST, PITCH };

   Units::Angle m_max_bank_angle;
   Units::Length m_altitude_threshold;
   Units::Speed m_speed_threshold;

   std::map<std::string, SpeedControlType> m_speed_control_types{{"thrust", THRUST}, {"pitch", PITCH}};
   SpeedControlType m_speed_control_type;

   bool m_model_loaded;

   static log4cplus::Logger m_logger;
};

inline bool AircraftControlLoader::IsLoaded() const { return m_model_loaded; }
}  // namespace loaders
}  // namespace aaesim
