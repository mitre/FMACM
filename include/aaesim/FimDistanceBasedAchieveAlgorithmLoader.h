#pragma once

#include "aaesim/AbstractFimAlgorithmLoader.h"

#include "imalgs/IMDistBasedAchieve.h"
#include "imalgs/FIMAlgorithmAdapter.h"

namespace aaesim::loaders {
class FimDistanceBasedAchieveAlgorithmLoader final : public aaesim::loaders::AbstractFimAlgorithmLoader {
  public:
   inline static const std::string TOP_LEVEL_BRACKET_NAME{"IM_dist_based_achieve"};
   inline static const IMUtils::IMAlgorithmTypes FIM_ALGORITHM_TYPE{IMUtils::IMAlgorithmTypes::DISTANCEBASEDACHIEVE};
   FimDistanceBasedAchieveAlgorithmLoader() : AbstractFimAlgorithmLoader(TOP_LEVEL_BRACKET_NAME){};

  protected:
   std::shared_ptr<interval_management::open_source::IMKinematicAchieve> BuildLoadableFimAlgorithm() override {
      return std::make_unique<interval_management::open_source::IMDistBasedAchieve>();
   }

   std::shared_ptr<aaesim::open_source::FlightDeckApplication> BuildAdaptedFimAlgorithm(
         std::shared_ptr<interval_management::open_source::IMKinematicAchieve> &fim_algorithm) override {
      return std::make_shared<interval_management::open_source::FIMAlgorithmAdapter>(fim_algorithm, FIM_ALGORITHM_TYPE);
   }
};

}  // namespace aaesim::loaders
