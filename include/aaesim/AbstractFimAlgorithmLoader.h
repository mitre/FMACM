#pragma once

#include "public/FlightDeckApplicationLoader.h"

#include "imalgs/IMKinematicAchieve.h"
#include "imalgs/IMUtils.h"

namespace aaesim::loaders {
class AbstractFimAlgorithmLoader : public aaesim::open_source::FlightDeckApplicationLoader {
  public:
   AbstractFimAlgorithmLoader(const std::string &loadable_variable);
   void RegisterLoadableVariables() override;
   bool VariablesAreLoaded() const override;
   bool load(DecodedStream *input) override;
   std::string GetTopLevelTag() const override { return m_loadable_variable; }
   std::shared_ptr<aaesim::open_source::FlightDeckApplication> ConstructLoadedAlgorithm(
         aaesim::open_source::StatisticalPilotDelay &pilot_delay) override;

  protected:
   virtual std::shared_ptr<interval_management::open_source::IMKinematicAchieve> BuildLoadableFimAlgorithm() = 0;
   virtual std::shared_ptr<aaesim::open_source::FlightDeckApplication> BuildAdaptedFimAlgorithm(
         std::shared_ptr<interval_management::open_source::IMKinematicAchieve> &fim_algorithm) = 0;

  private:
   inline static log4cplus::Logger m_logger{
         log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("AbstractFimAlgorithmFactory"))};
   std::string m_loadable_variable{};
   std::shared_ptr<interval_management::open_source::IMKinematicAchieve> m_fim_algorithm{};
};

inline void AbstractFimAlgorithmLoader::RegisterLoadableVariables() {}

inline bool AbstractFimAlgorithmLoader::VariablesAreLoaded() const {
   if (!m_fim_algorithm) return false;
   return m_fim_algorithm->IsLoaded();
};

}  // namespace aaesim::loaders
