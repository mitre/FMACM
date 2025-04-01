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

#include "public/FlightDeckApplication.h"
#include "public/PassThroughAssap.h"

namespace aaesim {
class DefaultAlgorithmInitializer final : public aaesim::open_source::FlightDeckApplicationInitializer {
  public:
   DefaultAlgorithmInitializer() = default;

   class Builder {
     private:
      aaesim::open_source::OwnshipPerformanceParameters m_performance_parameters;
      aaesim::open_source::OwnshipFmsPredictionParameters m_prediction_parameters;
      std::shared_ptr<const aaesim::open_source::ASSAP> m_surveillance_processor;

     public:
      Builder()
         : m_performance_parameters(),
           m_prediction_parameters(),
           m_surveillance_processor(std::make_shared<aaesim::open_source::PassThroughAssap>()){};
      ~Builder() = default;
      const aaesim::DefaultAlgorithmInitializer Build() const;
      Builder *AddOwnshipPerformanceParameters(
            aaesim::open_source::OwnshipPerformanceParameters performance_parameters);
      Builder *AddOwnshipFmsPredictionParameters(
            aaesim::open_source::OwnshipFmsPredictionParameters prediction_parameters);
      Builder *AddSurveillanceProcessor(std::shared_ptr<const aaesim::open_source::ASSAP> processor);

      aaesim::open_source::OwnshipPerformanceParameters GetPerformanceParameters() const {
         return m_performance_parameters;
      };
      aaesim::open_source::OwnshipFmsPredictionParameters GetFmsPredictionParameters() const {
         return m_prediction_parameters;
      };
      std::shared_ptr<const aaesim::open_source::ASSAP> GetSurveillanceProcessor() const {
         return m_surveillance_processor;
      };
   };

  private:
   DefaultAlgorithmInitializer(const DefaultAlgorithmInitializer::Builder *builder);
};

}  // namespace aaesim
