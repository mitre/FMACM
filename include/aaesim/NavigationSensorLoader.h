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
#include "avionics/NavigationSensor.h"

namespace aaesim {
namespace loaders {
class NavigationSensorLoader final : public LoggingLoadable {
  public:
   NavigationSensorLoader() = default;
   ~NavigationSensorLoader() = default;

   bool load(DecodedStream *input) override;

   bool IsLoaded() const;

   std::shared_ptr<aaesim::NavigationSensor> BuildNavigationSensor();

  private:
   static log4cplus::Logger m_logger;
   bool m_model_loaded;
   double m_nacp, m_nacv, m_nicp, m_nicv;
   bool m_use_position_bias, m_use_velocity_bias;
};

inline bool NavigationSensorLoader::IsLoaded() const { return m_model_loaded; }
}  // namespace loaders

}  // namespace aaesim
