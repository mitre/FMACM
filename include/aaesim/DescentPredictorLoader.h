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

#include "aaesim/DescentVerticalPredictorLoader.h"

#include "public/LoggingLoadable.h"
#include "avionics/Wgs84KineticDescentPredictor.h"
#include "scalar/Angle.h"

namespace aaesim {
namespace loaders {
class DescentPredictorLoader final : public LoggingLoadable {
  public:
   DescentPredictorLoader() = default;
   ~DescentPredictorLoader() = default;
   bool load(DecodedStream *input);
   aaesim::Wgs84KineticDescentPredictor BuildPredictor();
   bool IsLoaded() const { return m_is_loaded; }

  private:
   static log4cplus::Logger m_logger;
   Units::Angle m_max_bank_below_fl195;
   aaesim::loaders::DescentVerticalPredictorLoader m_kinetic_descent_loader;
   bool m_is_loaded;
};
}  // namespace loaders
}  // namespace aaesim