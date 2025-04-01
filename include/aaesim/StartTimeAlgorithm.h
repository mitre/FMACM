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
#include <map>
#include <vector>

#include "public/RandomGenerator.h"
#include "aaesim/AircraftLoader.h"

namespace aaesim {
struct StartTimeAlgorithm {
   virtual std::map<std::string, int> ComputeStartTimes(std::map<std::string, int> &loaded_values) = 0;
};

class NoVariationStartTimes : public StartTimeAlgorithm {
  public:
   NoVariationStartTimes() {}
   ~NoVariationStartTimes() = default;
   std::map<std::string, int> ComputeStartTimes(std::map<std::string, int> &loaded_values) override {
      return loaded_values;
   };
};

class IndependentStartTimes : public StartTimeAlgorithm {
  public:
   IndependentStartTimes(RandomGenerator generator, Units::Time standard_deviation_interdelivery_time)
      : m_start_time_latency_generator(generator),
        m_standard_deviation_interdelivery_time(standard_deviation_interdelivery_time) {}
   ~IndependentStartTimes() = default;
   std::map<std::string, int> ComputeStartTimes(std::map<std::string, int> &loaded_values) override {
      std::map<std::string, int> start_times;
      auto start_time_calculator = [this, &start_times](const std::pair<std::string, int> &loaded_pair) {
         int new_start_time = Units::SecondsTime(loaded_pair.second).value() +
                              static_cast<int>(m_start_time_latency_generator.TruncatedGaussianSample(
                                    0.0, Units::SecondsTime(m_standard_deviation_interdelivery_time).value(), 3.0));
         int locally_computed_start_time = std::max<int>(0, new_start_time);
         start_times.insert(std::make_pair(loaded_pair.first, locally_computed_start_time));
      };
      std::for_each(loaded_values.cbegin(), loaded_values.cend(), start_time_calculator);
      return start_times;
   };

  private:
   RandomGenerator m_start_time_latency_generator;
   Units::Time m_standard_deviation_interdelivery_time;
};

class StatisticallyDependentStartTimes : public StartTimeAlgorithm {
  public:
   StatisticallyDependentStartTimes(RandomGenerator generator, Units::Time mean_interdelivery_time,
                                    Units::Time standard_deviation_interdelivery_time)
      : m_start_time_latency_generator(generator),
        m_mean_interdelivery_time(mean_interdelivery_time),
        m_standard_deviation_interdelivery_time(standard_deviation_interdelivery_time) {}
   ~StatisticallyDependentStartTimes() = default;
   std::map<std::string, int> ComputeStartTimes(std::map<std::string, int> &loaded_values) override {
      std::map<std::string, int> start_times;
      int start_time = 0;
      auto start_time_calculator = [this, &start_time, &start_times](const std::pair<std::string, int> &loaded_pair) {
         start_times.insert(std::make_pair(loaded_pair.first, start_time));
         start_time += static_cast<int>(m_start_time_latency_generator.TruncatedGaussianSample(
               Units::SecondsTime(m_mean_interdelivery_time).value(),
               Units::SecondsTime(m_standard_deviation_interdelivery_time).value(), 3.0));
      };
      std::for_each(loaded_values.cbegin(), loaded_values.cend(), start_time_calculator);
      return start_times;
   };

  private:
   RandomGenerator m_start_time_latency_generator;
   Units::Time m_mean_interdelivery_time;
   Units::Time m_standard_deviation_interdelivery_time;
};
}  // namespace aaesim