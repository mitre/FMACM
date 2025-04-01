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

#include "loader/Loadable.h"
#include "public/LoggingLoadable.h"
#include "avionics/Wgs84KineticDescent4DPredictor.h"
#include <string>
#include <vector>
#include <scalar/Speed.h>
#include <scalar/Length.h>
#include <scalar/Time.h>

namespace aaesim {
namespace loaders {
class DescentVerticalPredictorLoader final : public LoggingLoadable {

  public:
   DescentVerticalPredictorLoader() = default;
   ~DescentVerticalPredictorLoader() = default;

   bool load(DecodedStream *input);

   bool const IsLoaded() const;

   void SetModelLoaded(const bool value);

   void SetDescentType(const std::string &descent_type);

   std::shared_ptr<Wgs84KineticDescent4DPredictor> BuildWgs84KineticDescentPredictor();

  private:
   inline static log4cplus::Logger m_logger{
         log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("DescentVerticalPredictorLoader"))};
   static std::map<std::string, Wgs84KineticDescent4DPredictor::Wgs84KineticDescentType> m_descent_type_converter;
   std::string m_descent_type{};
   Units::KnotsSpeed m_ias_at_end_of_route{Units::ZERO_SPEED};
   Units::KnotsSpeed m_transition_ias{Units::ZERO_SPEED};
   Units::Length m_altitude_at_end_of_route{Units::negInfinity()};
   double m_mass_percentile{0.3};
   double m_idle_thrust_factor{1.0};
   bool m_model_loaded{false};
};

inline void aaesim::loaders::DescentVerticalPredictorLoader::SetModelLoaded(const bool value) {
   m_model_loaded = value;
}

inline void aaesim::loaders::DescentVerticalPredictorLoader::SetDescentType(const std::string &descent_type) {
   m_descent_type.assign(descent_type);
}
}  // namespace loaders
}  // namespace aaesim
