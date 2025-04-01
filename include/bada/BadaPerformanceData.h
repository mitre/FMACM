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

#include <map>
#include <stdexcept>
#include <cstring>
#include <scalar/Area.h>
#include <scalar/Length.h>
#include <scalar/Mass.h>
#include <scalar/MassFlowRate.h>
#include <scalar/Speed.h>
#include "utility/CustomUnits.h"
#include "public/BadaUtils.h"

namespace aaesim {

namespace bada {

class BadaPerformanceData {
  public:
   static const std::string BADA_VERSION;
   BadaPerformanceData();
   ~BadaPerformanceData() = default;

   aaesim::open_source::bada_utils::AircraftType GetAircraftTypeInformation() const;

   aaesim::open_source::bada_utils::Mass GetAircraftMassInformation() const;

   aaesim::open_source::bada_utils::FlightEnvelope GetFlightEnvelopeInformation() const;

   aaesim::open_source::bada_utils::Aerodynamics GetAerodynamicsInformation() const;

   aaesim::open_source::bada_utils::EngineThrust GetEngineThrustInformation() const;

   aaesim::open_source::bada_utils::FuelFlow GetFuelFlowInformation() const;

   aaesim::open_source::bada_utils::GroundMovement GetGroundMovementInformation() const;

   aaesim::open_source::bada_utils::Procedure GetProcedureInformation(unsigned int index) const;

   aaesim::open_source::bada_utils::AircraftPerformance GetAircraftPerformanceInformation() const;

   const BadaPerformanceData &operator=(const BadaPerformanceData &obj);

   bool operator==(const BadaPerformanceData &obj) const;

   void init(std::string bada_data_path, std::string type_code);

   std::string GetBadaAircraftTypeCode() const {
      std::string return_string = type_name_;
      return return_string;
   }

  protected:
   std::string type_name_;
   std::string bada_data_path_;
   std::string base_filename_;
   aaesim::open_source::bada_utils::AircraftType aircraft_type_;
   aaesim::open_source::bada_utils::AircraftPerformance performance_;
   aaesim::open_source::bada_utils::GroundMovement ground_movement_;
   aaesim::open_source::bada_utils::EngineThrust engine_thrust_;
   aaesim::open_source::bada_utils::FuelFlow fuel_flow_;
   aaesim::open_source::bada_utils::Mass mass_characteristics_;
   aaesim::open_source::bada_utils::Aerodynamics aerodynamics_;
   aaesim::open_source::bada_utils::FlightEnvelope flight_envelope_;
   aaesim::open_source::bada_utils::Procedure procedures_[3];

   /**
    * Maps the aircraft type to an 8-character file name (no extension).
    * Multiple type codes may map to the same value.
    */
   std::string get_file(std::string type_code /** aircraft type abbreviation, e.g. B737 */);

   bool read_OPF();

   bool read_APF();

   bool read_PTF();
};

inline aaesim::open_source::bada_utils::AircraftType BadaPerformanceData::GetAircraftTypeInformation() const {
   return aircraft_type_;
}

inline aaesim::open_source::bada_utils::AircraftPerformance BadaPerformanceData::GetAircraftPerformanceInformation()
      const {
   return performance_;
}

inline aaesim::open_source::bada_utils::GroundMovement BadaPerformanceData::GetGroundMovementInformation() const {
   return ground_movement_;
}

inline aaesim::open_source::bada_utils::EngineThrust BadaPerformanceData::GetEngineThrustInformation() const {
   return engine_thrust_;
}

inline aaesim::open_source::bada_utils::FuelFlow BadaPerformanceData::GetFuelFlowInformation() const {
   return fuel_flow_;
}

inline aaesim::open_source::bada_utils::Mass BadaPerformanceData::GetAircraftMassInformation() const {
   return mass_characteristics_;
}

inline aaesim::open_source::bada_utils::Aerodynamics BadaPerformanceData::GetAerodynamicsInformation() const {
   return aerodynamics_;
}

inline aaesim::open_source::bada_utils::FlightEnvelope BadaPerformanceData::GetFlightEnvelopeInformation() const {
   return flight_envelope_;
}

inline aaesim::open_source::bada_utils::Procedure BadaPerformanceData::GetProcedureInformation(
      unsigned int index) const {
   if (index >= sizeof(procedures_)) {
      throw std::runtime_error("invalid array index");
   }
   return procedures_[index];
}

}  // namespace bada
}  // namespace aaesim