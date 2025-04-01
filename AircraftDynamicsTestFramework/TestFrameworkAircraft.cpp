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

#include "framework/TestFrameworkAircraft.h"

#include "public/Guidance.h"

using namespace std;
using namespace aaesim::open_source;

log4cplus::Logger TestFrameworkAircraft::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("TestFrameworkAircraft"));

TestFrameworkAircraft::TestFrameworkAircraft()
   : m_weather_truth(),
     m_adsb_receiver(),
     m_dynamics(),
     m_guidance_calculator(),
     m_aircraft_control(),
     m_bada_calculator(),
     m_speed_application(),
     m_states() {}

bool TestFrameworkAircraft::Update(const SimulationTime &time) {

   if (time.GetCurrentSimulationTime() <= m_states.cbegin()->GetTime()) {
      return false;
   }

   aaesim::open_source::AircraftState previous_state = m_states.back();

   m_weather_truth->Update(time, m_guidance_calculator->GetEstimatedDistanceAlongPath(),
                           previous_state.GetAltitudeMsl());

   Guidance current_guidance = m_guidance_calculator->Update(previous_state);
   m_adsb_receiver->Receive(time, aaesim::open_source::AircraftState());
   Guidance speed_guidance_from_application =
         m_speed_application->Update(time, current_guidance, m_dynamics->GetDynamicsState(), previous_state);
   if (speed_guidance_from_application.IsValid() && speed_guidance_from_application.m_ias_command > Units::ZERO_SPEED) {
      current_guidance.m_ias_command = speed_guidance_from_application.m_ias_command;
   }
   AircraftState state_result =
         m_dynamics->Update(m_states.back().GetUniqueId(), time, current_guidance, m_aircraft_control);
   SaveState(state_result, time);

   return IsFinished();
}

void TestFrameworkAircraft::SaveState(AircraftState &state, const SimulationTime &time) { m_states.push_back(state); }

std::shared_ptr<TestFrameworkAircraft> TestFrameworkAircraft::Builder::Build() const {
   return std::make_shared<TestFrameworkAircraft>(*this);
}

TestFrameworkAircraft::TestFrameworkAircraft(const Builder &builder) {
   m_weather_truth = builder.GetWeatherTruth();
   m_adsb_receiver = builder.GetAdsbReceiver();
   m_dynamics = builder.GetDynamicsModel();
   m_guidance_calculator = builder.GetGuidanceCalculator();
   m_aircraft_control = builder.GetAircraftControl();
   m_bada_calculator = builder.GetAircraftPerformance();
   m_speed_application = builder.GetFligthDeckApplication();
   auto initial_state = builder.GetInitialState();
   m_states.push_back(initial_state);
}

TestFrameworkAircraft::Builder *TestFrameworkAircraft::Builder::WithAircraftPerformance(
      std::shared_ptr<aaesim::open_source::FixedMassAircraftPerformance> &performance) {
   performance_ = performance;
   return this;
}

TestFrameworkAircraft::Builder *TestFrameworkAircraft::Builder::WithAircraftDynamics(
      std::shared_ptr<aaesim::open_source::ThreeDOFDynamics> &dynamics) {
   aircraft_dynamics_model_ = dynamics;
   return this;
}

TestFrameworkAircraft::Builder *TestFrameworkAircraft::Builder::WithAircraftControl(
      std::shared_ptr<aaesim::open_source::AircraftControl> &control) {
   aircraft_control_ = control;
   return this;
}

TestFrameworkAircraft::Builder *TestFrameworkAircraft::Builder::WithTrueWeather(
      std::shared_ptr<fmacm::WeatherTruthFromStaticData> &true_weather) {
   true_weather_ = true_weather;
   return this;
}

TestFrameworkAircraft::Builder *TestFrameworkAircraft::Builder::WithAdsbReceiver(
      std::shared_ptr<aaesim::open_source::ADSBReceiver> &receiver) {
   receiver_ = receiver;
   return this;
}

TestFrameworkAircraft::Builder *TestFrameworkAircraft::Builder::WithGuidanceCalculator(
      std::shared_ptr<fmacm::GuidanceFromStaticData> &guidance_calculator) {
   guidance_calculator_ = guidance_calculator;
   return this;
}

TestFrameworkAircraft::Builder *TestFrameworkAircraft::Builder::WithFlightDeckApplication(
      std::shared_ptr<aaesim::open_source::FlightDeckApplication> &speed_application) {
   speed_application_ = speed_application;
   return this;
}

TestFrameworkAircraft::Builder *TestFrameworkAircraft::Builder::WithInitialState(
      const aaesim::open_source::AircraftState &initial_state) {
   initial_state_ = initial_state;
   return this;
}