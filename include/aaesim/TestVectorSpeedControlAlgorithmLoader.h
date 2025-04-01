#pragma once

#include "aaesim/AbstractFimAlgorithmLoader.h"

#include "im_internal/TestVectorSpeedControl.h"
#include "imalgs/IMUtils.h"
#include "im_internal/InternalAlgorithmAdapter.h"

namespace aaesim::loaders {
class TestVectorSpeedControlAlgorithmLoader final : public aaesim::loaders::AbstractFimAlgorithmLoader {
  public:
   inline static const std::string TOP_LEVEL_BRACKET_NAME{"IM_test_speed_control"};
   inline static const IMUtils::IMAlgorithmTypes FIM_ALGORITHM_TYPE{IMUtils::IMAlgorithmTypes::TESTSPEEDCONTROL};
   TestVectorSpeedControlAlgorithmLoader() : AbstractFimAlgorithmLoader(TOP_LEVEL_BRACKET_NAME){};

  protected:
   std::shared_ptr<interval_management::open_source::IMKinematicAchieve> BuildLoadableFimAlgorithm() override {
      return std::make_unique<interval_management::TestVectorSpeedControl>();
   }

   std::shared_ptr<aaesim::open_source::FlightDeckApplication> BuildAdaptedFimAlgorithm(
         std::shared_ptr<interval_management::open_source::IMKinematicAchieve> &fim_algorithm) override {
      return std::make_shared<aaesim::InternalAlgorithmAdapter>(fim_algorithm, FIM_ALGORITHM_TYPE);
   }
};

}  // namespace aaesim::loaders
