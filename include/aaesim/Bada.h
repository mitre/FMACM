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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <map>
#include <stdexcept>
#include <scalar/Area.h>
#include <scalar/Length.h>
#include <scalar/Mass.h>
#include <scalar/MassFlowRate.h>
#include <scalar/Speed.h>
#include "utility/CustomUnits.h"
#include "public/BadaUtils.h"

namespace aaesim {

class Bada {
  public:
   static const std::string BADA_VERSION;
   static const Bada *LoadDataForAircraftType(std::string aircraft_type);
   Bada() = default;
   ~Bada() = default;

   aaesim::open_source::bada_utils::AircraftType GetAircraftTypeInformation() const;

   aaesim::open_source::bada_utils::Mass GetAircraftMassInformation() const;

   aaesim::open_source::bada_utils::FlightEnvelope GetFlightEnvelopeInformation() const;

   aaesim::open_source::bada_utils::Aerodynamics GetAerodynamicsInformation() const;

   aaesim::open_source::bada_utils::EngineThrust GetEngineThrustInformation() const;

   aaesim::open_source::bada_utils::FuelFlow GetFuelFlowInformation() const;

   aaesim::open_source::bada_utils::GroundMovement GetGroundMovementInformation() const;

   aaesim::open_source::bada_utils::Procedure GetProcedureInformation(unsigned int index) const;

   aaesim::open_source::bada_utils::AircraftPerformance GetAircraftPerformanceInformation() const;

   const Bada &operator=(const Bada &obj);

   bool operator==(const Bada &obj) const;

  protected:
   static char input_path[512];

   // Aircraft Type
   char *type_name;

   // Base file name
   std::string base_name;

   aaesim::open_source::bada_utils::AircraftType m_aircraft_type;
   aaesim::open_source::bada_utils::AircraftPerformance m_performance;
   aaesim::open_source::bada_utils::GroundMovement m_ground_movement;
   aaesim::open_source::bada_utils::EngineThrust m_engine_thrust;
   aaesim::open_source::bada_utils::FuelFlow m_fuel_flow;
   aaesim::open_source::bada_utils::Mass m_mass;
   aaesim::open_source::bada_utils::Aerodynamics m_aerodynamics;
   aaesim::open_source::bada_utils::FlightEnvelope m_flight_envelope;
   aaesim::open_source::bada_utils::Procedure m_procedures[3];

   void init(char *type_code);

   /**
    * Maps the aircraft type to an 8-character file name (no extension).
    * Multiple type codes may map to the same value.
    */
   std::string get_file(char *type_code /** aircraft type abbreviation, e.g. B737 */);

   bool read_OPF();

   bool read_APF();

   bool read_PTF();

  private:
   static std::map<std::string, Bada *> m_loaded_bada_performance;
};

inline aaesim::open_source::bada_utils::AircraftType Bada::GetAircraftTypeInformation() const {
   return m_aircraft_type;
}

inline aaesim::open_source::bada_utils::AircraftPerformance Bada::GetAircraftPerformanceInformation() const {
   return m_performance;
}

inline aaesim::open_source::bada_utils::GroundMovement Bada::GetGroundMovementInformation() const {
   return m_ground_movement;
}

inline aaesim::open_source::bada_utils::EngineThrust Bada::GetEngineThrustInformation() const {
   return m_engine_thrust;
}

inline aaesim::open_source::bada_utils::FuelFlow Bada::GetFuelFlowInformation() const { return m_fuel_flow; }

inline aaesim::open_source::bada_utils::Mass Bada::GetAircraftMassInformation() const { return m_mass; }

inline aaesim::open_source::bada_utils::Aerodynamics Bada::GetAerodynamicsInformation() const { return m_aerodynamics; }

inline aaesim::open_source::bada_utils::FlightEnvelope Bada::GetFlightEnvelopeInformation() const {
   return m_flight_envelope;
}

inline aaesim::open_source::bada_utils::Procedure Bada::GetProcedureInformation(unsigned int index) const {
   if (index >= sizeof(m_procedures)) {
      throw std::runtime_error("invalid array index");
   }
   return m_procedures[index];
}

}  // namespace aaesim