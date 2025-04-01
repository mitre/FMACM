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

#include <memory>

#include "rtaalgs/FimAlgorithmDelegate.h"
#include "rtaalgs/RTAUtils.h"
#include "scalar/Time.h"
#include "scalar/Speed.h"
#include "scalar/Length.h"
#include "public/WaypointPassingMonitor.h"

namespace required_time_of_arrival {
class RtaRct final : public aaesim::open_source::FlightDeckApplication {
  public:
   struct AlgorithmOutputData {
      Units::Speed unlimited_ias_command;
      Units::Speed final_ias_command;
      double final_mach_command;
      Units::Length distance_to_rta_fix;
      Units::Time eta_at_rta_fix;
      Units::Time eta_error;
      bool implement_speed_command;
      int speed_command_count;
      unsigned long int fim_im_speed_limit_flags;
      Units::Time fim_predicted_spacing_interval;
      Units::Time fim_ownship_ttg_to_ptp;
      Units::Length fim_ownship_dtg_to_ptp;
      Units::Speed fim_ownship_reference_ias;
      Units::Speed fim_ownship_reference_groundspeed;
      Units::Length fim_ownship_reference_altitude;
      aaesim::LatitudeLongitudePoint fms_position;
   };

   RtaRct() = default;
   ~RtaRct() = default;
   bool IsBlendWind() const;
   void Initialize(aaesim::open_source::FlightDeckApplicationInitializer &initializer_visitor) override;
   aaesim::open_source::Guidance Update(const aaesim::open_source::SimulationTime &simtime,
                                        const aaesim::open_source::Guidance &prevguidance,
                                        const aaesim::open_source::DynamicsState &dynamicsstate,
                                        const aaesim::open_source::AircraftState &aircraftstate) override;
   bool IsActive() const override;
   const std::vector<std::pair<Units::Time, AlgorithmOutputData>> &GetAlgorithmOutputData() const;

   class Builder {
     private:
      bool m_enable_wind_blending{false};
      Units::Time m_rta_goal{};
      std::string m_rta_fix{};
      interval_management::open_source::FIMConfiguration m_fim_configuration{};
      aaesim::open_source::StatisticalPilotDelay m_pilot_delay{};

     public:
      Builder() = default;
      ~Builder() = default;
      required_time_of_arrival::RtaRct *Build() const;
      Builder *WithWindBlending(bool enable_wind_blending);
      Builder *WithPilotDelayAlgorithm(aaesim::open_source::StatisticalPilotDelay &pilot_delay);
      Builder *UseRequiredTimeOfArrivalGoal(Units::Time rta_goal);
      Builder *TerminateAtFix(const std::string &rta_fix);
      Builder *WithFimConfiguration(const interval_management::open_source::FIMConfiguration &configuration);
      bool IsWindBlendingEnabled() const { return m_enable_wind_blending; };
      std::string GetRtaFix() const { return m_rta_fix; };
      Units::Time GetRtaGoal() const { return m_rta_goal; };
      aaesim::open_source::StatisticalPilotDelay GetPilotDelayAlgorithm() const { return m_pilot_delay; }
      interval_management::open_source::FIMConfiguration GetFimConfiguration() const { return m_fim_configuration; }
   };

  private:
   inline static log4cplus::Logger m_logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("RtaRct"))};
   static void DoLogging(const aaesim::open_source::SimulationTime &simulation_time,
                         const AlgorithmOutputData &output_data);
   static AlgorithmOutputData BuildDefaultAlgorithmOutputDataSet();
   AlgorithmOutputData BuildOutputDataSet(const aaesim::open_source::SimulationTime &simulation_time,
                                          const aaesim::open_source::Guidance &fim_guidance,
                                          const aaesim::LatitudeLongitudePoint &current_position);
   interval_management::open_source::IMAlgorithm::OwnshipPredictionParameters BuildOwnshipPredictionParameters(
         const aaesim::open_source::FlightDeckApplicationInitializer &initializer) const;

   RtaRct(const Builder &builder);
   const Units::Time CalculateAssignedSpacingGoal(const Units::Time required_time_of_arrival,
                                                  const Units::Time current_time);
   std::vector<interval_management::open_source::AircraftState> CreateFakeTargetAdsbHistory(
         const interval_management::open_source::AircraftState &current_ownship_state) const;
   const bool ReadyToInitializeRtaOperation();

   std::string m_rta_waypoint_name{};
   Units::SecondsTime m_required_time_of_arrival{0};
   bool m_rta_operation_initialized{false};
   std::vector<interval_management::open_source::AircraftState> m_fake_target_adsb_history{};
   std::unique_ptr<FimAlgorithmDelegate> m_fim_algorithm{};
   std::vector<std::pair<Units::Time, AlgorithmOutputData>> m_algorithm_output_data{};
   std::shared_ptr<aaesim::open_source::WaypointPassingMonitor> m_rta_fix_monitor{};
   interval_management::open_source::AircraftState m_fake_target_state{};
   interval_management::open_source::FIMConfiguration m_fim_configuration{};
};
}  // namespace required_time_of_arrival

inline bool required_time_of_arrival::RtaRct::IsBlendWind() const { return m_fim_algorithm->IsBlendWind(); }

inline const std::vector<std::pair<Units::Time, required_time_of_arrival::RtaRct::AlgorithmOutputData>> &
      required_time_of_arrival::RtaRct::GetAlgorithmOutputData() const {
   return m_algorithm_output_data;
}

inline const Units::Time required_time_of_arrival::RtaRct::CalculateAssignedSpacingGoal(
      const Units::Time required_time_of_arrival, const Units::Time current_time) {
   return required_time_of_arrival - current_time;
}

inline required_time_of_arrival::RtaRct::AlgorithmOutputData
      required_time_of_arrival::RtaRct::BuildDefaultAlgorithmOutputDataSet() {
   required_time_of_arrival::RtaRct::AlgorithmOutputData data;
   data.distance_to_rta_fix = Units::infinity();
   data.eta_error = Units::infinity();
   data.eta_at_rta_fix = Units::infinity();
   data.final_ias_command = Units::infinity();
   data.implement_speed_command = false;
   data.speed_command_count = INT16_MIN;
   data.unlimited_ias_command = Units::infinity();
   data.final_mach_command = 0;
   data.fim_predicted_spacing_interval = Units::infinity();
   data.fim_im_speed_limit_flags = 0;
   data.fim_ownship_ttg_to_ptp = Units::infinity();
   data.fim_ownship_dtg_to_ptp = Units::infinity();
   data.fim_ownship_reference_ias = Units::infinity();
   data.fim_ownship_reference_groundspeed = Units::infinity();
   data.fim_ownship_reference_altitude = Units::infinity();
   return data;
}

inline interval_management::open_source::IMAlgorithm::OwnshipPredictionParameters
      required_time_of_arrival::RtaRct::BuildOwnshipPredictionParameters(
            const aaesim::open_source::FlightDeckApplicationInitializer &initializer) const {
   interval_management::open_source::IMAlgorithm::OwnshipPredictionParameters ownship_prediction_parameters;
   ownship_prediction_parameters.aerodynamics = initializer.performance_parameters.aerodynamics;
   ownship_prediction_parameters.flap_speeds = initializer.performance_parameters.flap_speeds;
   ownship_prediction_parameters.flight_envelope = initializer.performance_parameters.flight_envelope;
   ownship_prediction_parameters.mass_data = initializer.performance_parameters.mass_data;
   ownship_prediction_parameters.expected_cruise_altitude =
         initializer.fms_prediction_parameters.expected_cruise_altitude;
   ownship_prediction_parameters.maximum_allowable_bank_angle =
         initializer.fms_prediction_parameters.maximum_allowable_bank_angle;
   ownship_prediction_parameters.transition_altitude = initializer.fms_prediction_parameters.transition_altitude;
   ownship_prediction_parameters.transition_ias = initializer.fms_prediction_parameters.transition_ias;
   ownship_prediction_parameters.transition_mach = initializer.fms_prediction_parameters.transition_mach;
   return ownship_prediction_parameters;
}
