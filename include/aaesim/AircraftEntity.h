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

#include "public/ScenarioEntity.h"

#include "public/NullAdsbTransmitter.h"
#include "public/AircraftState.h"
#include "public/AircraftIntent.h"
#include "public/NullADSBReceiver.h"
#include "public/NullFlightDeckApplication.h"
#include "public/SimulationTime.h"
#include "public/ThreeDOFDynamics.h"
#include "public/SimulationTime.h"
#include "public/ADSBReceiver.h"
#include "public/ADSBTransmitter.h"
#include "public/FlightDeckApplication.h"
#include "avionics/FlightManagementSystem.h"

namespace aaesim {
class AircraftEntity final : public aaesim::open_source::ScenarioEntity {
  public:
   AircraftEntity();
   bool Update(const open_source::SimulationTime &simulation_time) override;
   const std::vector<aaesim::open_source::AircraftState> &GetTruthAircraftStates() const {
      return m_truth_aircraft_states;
   }
   const int GetStartTime() const override;
   bool IsFinished() const override;
   std::string const &GetAircraftId() const { return m_aircraft_id; }
   int GetAircraftNumericId() const { return m_unique_id; }
   const AircraftIntent GetAircraftIntent() const { return m_aircraft_intent; }
   std::shared_ptr<const aaesim::open_source::ThreeDOFDynamics> GetDynamics() const;
   std::shared_ptr<const FlightManagementSystem> GetFms() const;
   std::shared_ptr<const aaesim::open_source::ADSBReceiver> GetAdsbReceiver() const;
   std::shared_ptr<const aaesim::open_source::ADSBTransmitter> GetAdsbTransmitter() const;
   std::shared_ptr<const aaesim::open_source::FlightDeckApplication> GetFlightDeckApplication() const;

   class Builder final {
     public:
      Builder()
         : aircraft_dynamics_model_(),
           receiver_(std::make_unique<aaesim::open_source::NullADSBReceiver>()),
           transmitter_(std::make_unique<aaesim::open_source::NullAdsbTransmitter>()),
           flight_management_system_(),
           speed_application_(std::make_unique<aaesim::open_source::NullFlightDeckApplication>()),
           initial_state_(),
           speed_application_placebo_mode_enabled_(false){};
      ~Builder() = default;
      std::shared_ptr<AircraftEntity> Build() const;
      Builder *WithAircraftDynamics(std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> &dynamics);
      Builder *WithAdsbReceiver(std::shared_ptr<aaesim::open_source::ADSBReceiver> &receiver);
      Builder *WithAdsbTransmitter(std::shared_ptr<aaesim::open_source::ADSBTransmitter> &transmitter);
      Builder *WithFlightManagementSystem(std::shared_ptr<FlightManagementSystem> &flight_management_system);
      Builder *WithFlightDeckApplication(
            std::shared_ptr<aaesim::open_source::FlightDeckApplication> &speed_application);
      Builder *AtInitialState(const aaesim::open_source::AircraftState &initial_state);
      Builder *EnablePlaceboMode(const bool &enable_placebo_mode);

      std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> GetDynamicsModel() const {
         return aircraft_dynamics_model_;
      }
      std::shared_ptr<aaesim::open_source::ADSBReceiver> GetAdsbReceiver() const { return receiver_; }
      std::shared_ptr<FlightManagementSystem> GetFlightManagementSystem() const { return flight_management_system_; }
      std::shared_ptr<aaesim::open_source::FlightDeckApplication> GetFlightDeckApplication() const {
         return speed_application_;
      }
      aaesim::open_source::AircraftState GetInitialState() const { return initial_state_; }
      bool GetPlaceboModeSetting() const { return speed_application_placebo_mode_enabled_; }
      std::shared_ptr<aaesim::open_source::ADSBTransmitter> GetAdsbTransmitter() const { return transmitter_; }

     private:
      std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> aircraft_dynamics_model_;
      std::shared_ptr<aaesim::open_source::ADSBReceiver> receiver_;
      std::shared_ptr<aaesim::open_source::ADSBTransmitter> transmitter_;
      std::shared_ptr<FlightManagementSystem> flight_management_system_;
      std::shared_ptr<aaesim::open_source::FlightDeckApplication> speed_application_;
      aaesim::open_source::AircraftState initial_state_;
      bool speed_application_placebo_mode_enabled_;
   };

   AircraftEntity(const AircraftEntity::Builder &builder);

  private:
   static log4cplus::Logger m_logger;

   std::pair<aaesim::open_source::Guidance, std::shared_ptr<aaesim::open_source::AircraftControl>>
         CalculateAircraftGuidance(const aaesim::open_source::SimulationTime &time);

   std::string m_aircraft_id;
   int m_unique_id;
   bool m_is_finished;
   int m_start_time;
   std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> m_three_dof_dynamics;
   std::shared_ptr<FlightManagementSystem> m_flight_management_system;
   AircraftIntent m_aircraft_intent;
   std::vector<aaesim::open_source::AircraftState> m_truth_aircraft_states;
   std::shared_ptr<aaesim::open_source::ADSBReceiver> m_adsb_receiver;
   std::shared_ptr<aaesim::open_source::ADSBTransmitter> m_adsb_transmitter;
   std::shared_ptr<aaesim::open_source::FlightDeckApplication> m_flightdeck_application;
   bool m_enable_placebo_mode;
   bool m_has_flightdeck_application;
};

inline const int AircraftEntity::GetStartTime() const { return m_start_time; }

inline std::shared_ptr<const aaesim::open_source::ADSBReceiver> AircraftEntity::GetAdsbReceiver() const {
   return m_adsb_receiver;
}

inline std::shared_ptr<const aaesim::open_source::ADSBTransmitter> AircraftEntity::GetAdsbTransmitter() const {
   return m_adsb_transmitter;
}

inline std::shared_ptr<const aaesim::open_source::FlightDeckApplication> AircraftEntity::GetFlightDeckApplication()
      const {
   return m_flightdeck_application;
}

inline std::shared_ptr<const aaesim::open_source::ThreeDOFDynamics> AircraftEntity::GetDynamics() const {
   return m_three_dof_dynamics;
}

inline std::shared_ptr<const FlightManagementSystem> AircraftEntity::GetFms() const {
   return m_flight_management_system;
}

inline bool AircraftEntity::IsFinished() const { return m_flight_management_system->IsFinishedRoute(); }

}  // namespace aaesim
