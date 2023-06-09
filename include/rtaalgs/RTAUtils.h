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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <map>
#include "scalar/Length.h"
#include "scalar/Speed.h"
#include "utility/CustomUnits.h"

namespace required_time_of_arrival {

struct RTAUtils {
   struct PolynomialCoefficients {
      double c3;
      double c2;
      double c1;
      double c0;
   };

   struct EtaSpeedDerivatives {
      Units::SecondsPerKnot positive_speed_delta;
      Units::SecondsPerKnot negative_speed_delta;
   };

   static std::map<std::string, RTAUtils::PolynomialCoefficients> POSITIVE_COEFFICIENTS_BY_ACID;
   static std::map<std::string, RTAUtils::PolynomialCoefficients> NEGATIVE_COEFFICIENTS_BY_ACID;

   static EtaSpeedDerivatives CalculateEtaSpeedDerivatives(
         const RTAUtils::PolynomialCoefficients &positive_coefficients,
         const RTAUtils::PolynomialCoefficients &negative_coefficients, const Units::Length &distance_to_rta_fix);

   enum SpeedLimiterType { NONE = 0, FLIGHT_ENVELOPE = 1, FIM = 2 };
   static std::map<std::string, SpeedLimiterType> SPEED_LIMITERS;
};

inline RTAUtils::EtaSpeedDerivatives RTAUtils::CalculateEtaSpeedDerivatives(
      const RTAUtils::PolynomialCoefficients &positive_coefficients,
      const RTAUtils::PolynomialCoefficients &negative_coefficients, const Units::Length &distance_to_rta_fix) {

   auto evaluate = [](const PolynomialCoefficients &coefficients, const Units::NauticalMilesLength &dtg) {
      const auto dtg_cubed = dtg.value() * dtg.value() * dtg.value();
      const auto dtg_squared = dtg.value() * dtg.value();
      return coefficients.c3 * dtg_cubed + coefficients.c2 * dtg_squared + coefficients.c1 * dtg.value() +
             coefficients.c0;
   };

   Units::SecondsPerKnot positive(evaluate(positive_coefficients, distance_to_rta_fix));
   Units::SecondsPerKnot negative(evaluate(negative_coefficients, distance_to_rta_fix));
   return {positive, negative};
};
}  // namespace required_time_of_arrival