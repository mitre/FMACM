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

#include "avionics/Wgs84KineticDescentPredictor.h"
#include "im_internal/InternalAlgorithmAdapter.h"
#include "im_internal/TestVectorSpeedControl.h"
#include "public/FlightDeckApplication.h"

namespace aaesim {
class InternalAlgorithmInitializer : public aaesim::open_source::FlightDeckApplicationInitializer {
  public:
   InternalAlgorithmInitializer() = default;

   void Initialize(aaesim::InternalAlgorithmAdapter *algorithm);

   const interval_management::open_source::IMClearance GetImClearance() const;

   class Builder {
     private:
      aaesim::open_source::OwnshipPerformanceParameters m_performance_parameters;
      aaesim::open_source::OwnshipFmsPredictionParameters m_prediction_parameters;
      std::shared_ptr<const aaesim::open_source::ASSAP> m_surveillance_processor;
      interval_management::open_source::IMClearance m_clearance;
      aaesim::Wgs84KineticDescentPredictor m_fms_predictor;

     public:
      Builder() = default;
      ~Builder() = default;
      const aaesim::InternalAlgorithmInitializer Build() const;
      Builder *AddOwnshipPerformanceParameters(
            const aaesim::open_source::OwnshipPerformanceParameters &performance_parameters);
      Builder *AddOwnshipFmsPredictionParameters(
            const aaesim::open_source::OwnshipFmsPredictionParameters &prediction_parameters);
      Builder *AddSurveillanceProcessor(std::shared_ptr<const aaesim::open_source::ASSAP> processor);
      Builder *AddClearance(const interval_management::open_source::IMClearance &clearance);
      Builder *AddFmsKineticPredictor(const aaesim::Wgs84KineticDescentPredictor &fms_predictor);

      aaesim::open_source::OwnshipPerformanceParameters GetPerformanceParameters() const {
         return m_performance_parameters;
      };
      aaesim::open_source::OwnshipFmsPredictionParameters GetFmsPredictionParameters() const {
         return m_prediction_parameters;
      };
      std::shared_ptr<const aaesim::open_source::ASSAP> GetSurveillanceProcessor() const {
         return m_surveillance_processor;
      };
      aaesim::Wgs84KineticDescentPredictor GetFmsPredictor() const { return m_fms_predictor; }
      interval_management::open_source::IMClearance GetClearance() const { return m_clearance; };
   };

  private:
   InternalAlgorithmInitializer(const InternalAlgorithmInitializer::Builder *builder);
   void Initialize(interval_management::TestVectorSpeedControl *test_vector_algorithm);
   interval_management::open_source::IMAlgorithm::OwnshipPredictionParameters BuildOwnshipPredictionParameters() const;
   interval_management::open_source::IMClearance m_im_clearance;
   aaesim::Wgs84KineticDescentPredictor m_fms_descent_predictor;
};

inline const interval_management::open_source::IMClearance InternalAlgorithmInitializer::GetImClearance() const {
   return m_im_clearance;
}
}  // namespace aaesim
