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

#include <memory>
#include <cstring>
#include <cmath>
#include <filesystem>

#include "public/Atmosphere.h"
#include "public/FixedMassAircraftPerformance.h"
#include "scalar/Length.h"
#include "public/BadaUtils.h"
#include "bada/BadaPerformanceData.h"
#include "bada/BadaPerformanceInitialConditions.h"
#include "bada/NormalFlapProgression.h"
#include "bada/ReverseFlapProgression.h"
#include "bada/BadaFixedMassPerformanceCalculator.h"

namespace aaesim {
namespace bada {

class Bada3Factory final {
  public:
   static std::unique_ptr<Atmosphere> MakeAtmosphere(const Atmosphere::AtmosphereType atmosphere_type,
                                                     const Units::KelvinTemperature temperature,
                                                     const Units::Length altitude_msl);

   static std::shared_ptr<Atmosphere> MakeAtmosphereFromTemperatureOffset(
         const Atmosphere::AtmosphereType atmosphere_type, const Units::CelsiusTemperature temperature_offset);

   static std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> CreateBada3ReverseProgression(
         const std::string &bada_aircraft_code, double mass_percentile, Units::Length final_descent_altitude_msl) {
      BadaPerformanceInitialConditions internal_initial_conditions;
      internal_initial_conditions.mass_percentile = mass_percentile;
      internal_initial_conditions.faf_altitude_msl = final_descent_altitude_msl;
      internal_initial_conditions.initial_flap_configuration = GuessFinalFlapConfiguration(final_descent_altitude_msl);
      return CreateBada3Performance(bada_aircraft_code, internal_initial_conditions,
                                    std::make_shared<aaesim::bada::ReverseFlapProgression>());
   };

   static std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> CreateBada3ForwardProgression(
         const std::string &bada_aircraft_code, BadaPerformanceInitialConditions initial_conditions) {
      return CreateBada3Performance(bada_aircraft_code, initial_conditions,
                                    std::make_shared<aaesim::bada::NormalFlapProgression>());
   }

   static void SetBadaDataPath(const std::filesystem::path &path_to_bada_data) {
      path_to_bada_data_ = path_to_bada_data;
   };

   static std::shared_ptr<const BadaPerformanceData> LoadDataForAircraftType(const std::string &aircraft_type) {
      std::shared_ptr<BadaPerformanceData> bada_data = std::make_shared<BadaPerformanceData>();
      auto iter = loaded_bada_data_.find(aircraft_type);
      if (iter == loaded_bada_data_.end()) {
         bada_data->init(path_to_bada_data_, aircraft_type);
         loaded_bada_data_[aircraft_type] = bada_data;
      } else {
         bada_data = loaded_bada_data_.at(aircraft_type);
      }

      return bada_data;
   }

  private:
   static aaesim::open_source::bada_utils::FlapSpeeds DefineFlapSpeeds(
         std::shared_ptr<const BadaPerformanceData> bada_data, BadaPerformanceInitialConditions &initial_conditions) {
      // set minimum flap speeds based on a/c type and stall speeds coming from bada data
      const Units::Speed speed_padding = Units::KnotsSpeed(10.0);
      const aaesim::open_source::bada_utils::Aerodynamics aerodynamics = bada_data->GetAerodynamicsInformation();
      const aaesim::open_source::bada_utils::Mass mass_characteristics = bada_data->GetAircraftMassInformation();
      const Units::Mass aircraft_mass =
            (mass_characteristics.m_max - mass_characteristics.m_min) * initial_conditions.mass_percentile +
            mass_characteristics.m_min;
      const double mass_ratio = aircraft_mass / mass_characteristics.m_ref;
      const double mass_factor = 1.3 * std::sqrt(mass_ratio);
      aaesim::open_source::bada_utils::FlapSpeeds flap_speeds;
      flap_speeds.cas_takeoff_minimum = aerodynamics.take_off.V_stall * mass_factor + speed_padding;
      flap_speeds.cas_climb_minimum = aerodynamics.initial_climb.V_stall * mass_factor + speed_padding;
      flap_speeds.cas_cruise_minimum = aerodynamics.cruise.V_stall * mass_factor + speed_padding;
      flap_speeds.cas_approach_minimum = flap_speeds.cas_cruise_minimum;
      flap_speeds.cas_landing_minimum = aerodynamics.approach.V_stall * mass_factor + speed_padding;
      flap_speeds.cas_gear_out_minimum = aerodynamics.landing.V_stall * mass_factor + speed_padding;

      // set maximum flap speeds based on a/c type
      if (bada_data->GetBadaAircraftTypeCode() == "B737") {
         flap_speeds.cas_approach_maximum = Units::KnotsSpeed(250.0);
         flap_speeds.cas_landing_maximum = Units::KnotsSpeed(210.0);
         flap_speeds.cas_gear_out_maximum = Units::KnotsSpeed(165.0);
      } else if (bada_data->GetBadaAircraftTypeCode() == "B752") {
         flap_speeds.cas_approach_maximum = Units::KnotsSpeed(240.0);
         flap_speeds.cas_landing_maximum = Units::KnotsSpeed(210.0);
         flap_speeds.cas_gear_out_maximum = Units::KnotsSpeed(162.0);
      } else if (bada_data->GetBadaAircraftTypeCode() == "B763") {
         flap_speeds.cas_approach_maximum = Units::KnotsSpeed(250.0);
         flap_speeds.cas_landing_maximum = Units::KnotsSpeed(210.0);
         flap_speeds.cas_gear_out_maximum = Units::KnotsSpeed(170.0);
      } else if (bada_data->GetBadaAircraftTypeCode() == "A319") {
         flap_speeds.cas_approach_maximum = Units::KnotsSpeed(230.0);
         flap_speeds.cas_landing_maximum = Units::KnotsSpeed(200.0);
         flap_speeds.cas_gear_out_maximum = Units::KnotsSpeed(177.0);
      } else if (bada_data->GetBadaAircraftTypeCode() == "A320") {
         flap_speeds.cas_approach_maximum = Units::KnotsSpeed(230.0);
         flap_speeds.cas_landing_maximum = Units::KnotsSpeed(200.0);
         flap_speeds.cas_gear_out_maximum = Units::KnotsSpeed(177.0);
      } else if (bada_data->GetBadaAircraftTypeCode() == "CRJ9") {
         flap_speeds.cas_approach_maximum = Units::KnotsSpeed(215.0);
         flap_speeds.cas_landing_maximum = Units::KnotsSpeed(185.0);
         flap_speeds.cas_gear_out_maximum = Units::KnotsSpeed(170.0);
      } else {
         flap_speeds.cas_approach_maximum = Units::KnotsSpeed(240.0);
         flap_speeds.cas_landing_maximum = Units::KnotsSpeed(200.0);
         flap_speeds.cas_gear_out_maximum = Units::KnotsSpeed(170.0);
      }
      return flap_speeds;
   }

   static std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> CreateBada3Performance(
         const std::string &bada_aircraft_code, BadaPerformanceInitialConditions initial_conditions,
         std::shared_ptr<FlapProgressionCalculator> flap_calculator) {

      if (aaesim::open_source::bada_utils::FlapConfiguration::UNDEFINED ==
          initial_conditions.initial_flap_configuration) {
         throw std::logic_error("initial_flap_configuration must not equal FlapConfiguration::UNDEFINED");
      }
      BoundedValue<double, 0, 1> mass_fraction(0);
      try {
         mass_fraction = initial_conditions.mass_percentile;
      } catch (const std::exception &e) {
         throw std::logic_error("mass_percentile must be on the interval (0,1], but received " +
                                std::to_string(initial_conditions.mass_percentile));
      }

      std::shared_ptr<const BadaPerformanceData> bada_data = LoadDataForAircraftType(bada_aircraft_code);
      aaesim::open_source::bada_utils::FlapSpeeds flap_speeds = DefineFlapSpeeds(bada_data, initial_conditions);

      return std::make_shared<aaesim::bada::BadaFixedMassPerformanceCalculator>(bada_data, initial_conditions,
                                                                                flap_speeds, flap_calculator);
   };

   static aaesim::open_source::bada_utils::FlapConfiguration GuessFinalFlapConfiguration(
         const Units::Length final_descsent_altitude_msl) {
      if (final_descsent_altitude_msl >= BadaFixedMassPerformanceCalculator::FAF_ALTITUDE_MAXIMUM)
         return aaesim::open_source::bada_utils::FlapConfiguration::APPROACH;
      else
         return aaesim::open_source::bada_utils::FlapConfiguration::GEAR_DOWN;
   }

   static std::filesystem::path path_to_bada_data_;
   static std::map<std::string, std::shared_ptr<BadaPerformanceData>> loaded_bada_data_;
};
}  // namespace bada
}  // namespace aaesim
