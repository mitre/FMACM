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

#include "avionics/FmsIntegratedApplication.h"
#include "public/PassThroughAssap.h"

namespace aaesim {
class DefaultFmsApplicationInitializer final : public aaesim::avionics::FmsApplicationInitializer {
  public:
   DefaultFmsApplicationInitializer() = default;

   class Builder {
     private:
      aaesim::open_source::OwnshipPerformanceParameters m_performance_parameters;
      aaesim::open_source::OwnshipFmsPredictionParameters m_prediction_parameters;

     public:
      Builder() : m_performance_parameters(), m_prediction_parameters(){};
      ~Builder() = default;
      const aaesim::DefaultFmsApplicationInitializer Build() const;
      Builder *AddOwnshipPerformanceParameters(
            const aaesim::open_source::OwnshipPerformanceParameters &performance_parameters);
      Builder *AddOwnshipFmsPredictionParameters(
            const aaesim::open_source::OwnshipFmsPredictionParameters &prediction_parameters);

      aaesim::open_source::OwnshipPerformanceParameters GetPerformanceParameters() const {
         return m_performance_parameters;
      };
      aaesim::open_source::OwnshipFmsPredictionParameters GetFmsPredictionParameters() const {
         return m_prediction_parameters;
      };
   };

  private:
   DefaultFmsApplicationInitializer(const DefaultFmsApplicationInitializer::Builder *builder);
};

}  // namespace aaesim
