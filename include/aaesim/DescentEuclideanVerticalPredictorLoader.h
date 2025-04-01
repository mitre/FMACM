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

#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Time.h>

#include "public/LoggingLoadable.h"
#include "avionics/KineticDescent4DPredictor.h"

namespace aaesim {
namespace loaders {
class DescentEuclideanVerticalPredictorLoader final : public LoggingLoadable {

  public:
   DescentEuclideanVerticalPredictorLoader();

   ~DescentEuclideanVerticalPredictorLoader() = default;

   bool load(DecodedStream *input);

   bool const IsLoaded() const;

   void SetModelLoaded(const bool value);

   void SetDescentType(const std::string &descent_type);

   std::shared_ptr<KineticDescent4DPredictor> BuildVerticalPredictor();

  private:
   static log4cplus::Logger m_logger;
   static std::map<std::string, KineticDescent4DPredictor::KineticDescentType> m_descent_type_converter;
   std::string m_descent_type;
   Units::KnotsSpeed m_ias_at_end_of_route;
   Units::KnotsSpeed m_transition_ias;
   Units::Length m_altitude_at_end_of_route;
   double m_mass_percentile;
   double m_idle_thrust_factor;
   bool m_model_loaded;
};

inline void aaesim::loaders::DescentEuclideanVerticalPredictorLoader::SetModelLoaded(const bool value) {
   m_model_loaded = value;
}

inline void aaesim::loaders::DescentEuclideanVerticalPredictorLoader::SetDescentType(const std::string &descent_type) {
   m_descent_type.assign(descent_type);
}
}  // namespace loaders
}  // namespace aaesim
