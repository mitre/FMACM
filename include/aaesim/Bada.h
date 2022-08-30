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

namespace aaesim {
   enum ENGINE_TYPE {
      JET,
      TURBOPROP,
      PISTON
   };
   enum WAKE_CATEGORY {
      HEAVY_,
      MEDIUM_,
      LIGHT_
   };
#define FL_NMAX 40

   typedef struct {
      int FL;
      Units::Speed TAS;
      struct {
         Units::Speed lo;
         Units::Speed nom;
         Units::Speed hi;
      } ROCD;
      Units::MassFlowRate fuel;
   } climb_struct;

   typedef struct {
      int FL;
      Units::Speed TAS;
      struct {
         Units::MassFlowRate lo;
         Units::MassFlowRate nom;
         Units::MassFlowRate hi;
      } fuel;
   } cruise_struct;

   typedef struct {
      int FL;
      Units::Speed TAS;
      Units::Speed ROCD;
      Units::MassFlowRate fuel;
   } descent_struct;

   typedef struct {
      struct {
         int LO;
         int HI;
      } CAS;
      double Mach;
   } perf_speed_struct;

   class Bada {
    public:
      static const std::string BADA_VERSION;
      static const Bada* LoadDataForAircraftType(std::string aircraft_type);
      Bada() = default;
      ~Bada() = default;

      struct AircraftType {
         int n_eng;  // number of engines
         ENGINE_TYPE engine_type;
         WAKE_CATEGORY wake_category;
      };

      Bada::AircraftType GetAircraftTypeInformation() const;

      // Mass
      struct Mass {
         Units::Mass m_ref;  // reference mass
         Units::Mass m_min;  // minimum mass
         Units::Mass m_max;  // maximum mass
         Units::Mass m_pyld; // maximum payload mass
      };

      Bada::Mass GetAircraftMassInformation() const;

      // Flight Envelope
      struct FlightEnvelope {
         Units::Speed V_mo;   // maximum operating speed (CAS).
         double M_mo;         // maximum operating Mach number
         Units::Length h_mo;  // maximum operating altitude
         Units::Length h_max; // maximum altitude at MTOW and ISA
         Units::LengthToMassGradient G_w; // weight gradient on max. altitude
         double G_t;     // temperature gradient on max. altitude (feet/C)
      };

      Bada::FlightEnvelope GetFlightEnvelopeInformation() const;

      // Aerodynamics
      struct Aerodynamics {
         Units::Area S; // Reference wing surface area.
         double C_Lbo;  // Buffet onset lift coeff.  (jet only)
         double K;    // Buffeting gradient (jet only)
         double C_M16;    // No idea, but it's in the OPF

         struct {
            Units::Speed V_stall; // CAS
            double cd0;           // parasitic drag coeff.
            double cd2;           // induced drag coeff.
         } cruise;

         struct {
            Units::Speed V_stall; // CAS
            double cd0;           // parasitic drag coeff.
            double cd2;           // induced drag coeff.
         } initial_climb;

         struct {
            Units::Speed V_stall; // CAS
            double cd0;           // parasitic drag coeff.
            double cd2;           // induced drag coeff.
         } take_off;

         struct {
            Units::Speed V_stall; // CAS
            double cd0;           // parasitic drag coeff.
            double cd2;           // induced drag coeff.
         } approach;

         struct {
            Units::Speed V_stall; // CAS
            double cd0;           // parasitic drag coeff.
            double cd2;           // induced drag coeff.
         } landing;

         struct {
            double cd0;  // parasitic drag coeff.
         } landing_gear;

      };

      Bada::Aerodynamics GetAerodynamicsInformation() const;

      // Engine Thrust
      struct EngineThrust {
         struct {
            double CT_c1;                // 1st max. climb thrust coeff. (N, jet/piston)
            //                              (kt-N, turboprop)
            Units::Length CT_c2;         // 2nd max. climb thrust coeff.
            double CT_c3;                // 3rd max. climb thrust coeff. (1/feet^2, jet)
            //                              (N, turboprop)
            //                              (kt-N, piston)
            Units::AbsCelsiusTemperature CT_c4; // 1st thrust temperature coeff.
            double CT_c5;                // 2nd thrust temperature coeff. (1/deg. C)
         } max_climb;
         struct {
            double CT_low;      // low altitude descent thrust coeff.
            double CT_high;     // high altitude descent thrust coeff.
            Units::Length h;    // Transition altitude (feet)
            double CT_app;      // approach thrust coeff.
            double CT_ld;       // landing thrust coeff.
            Units::Speed V_ref; // reference descent speed (kt)
            double M_ref;       // reference descent Mach number
         } descent;
      };

      Bada::EngineThrust GetEngineThrustInformation() const;

      // fuel flow
      struct FuelFlow {
         double C_f1;              // 1st Thrust specific fuel consumption coeff.
         // (kg/min*kN) (jet)
         // (kg/min*kN*knot) (turboprop)
         // (kg/min)  (piston)
         Units::Speed C_f2;        // 2nd Thrust specific fuel consumption coeff.
         Units::MassFlowRate C_f3; // 1st descent thrust fuel flow coeff.
         Units::Length C_f4;       // 2nd descent thrust fuel flow coeff.
         double C_fcr;             // Cruise fuel flow coeff. (dimensionless)
      };

      Bada::FuelFlow GetFuelFlowInformation() const;

      // Ground movement
      struct GroundMovement {
         Units::Length TOL;     // Take-off length (m)
         Units::Length LDL;     // Landing length (m)
         Units::Length span;    // Wingspan (m)
         Units::Length length;  // Length (m)
      };

      Bada::GroundMovement GetGroundMovementInformation() const;

      // Procedure Specifications
      struct Procedure {
         struct {
            int V1;
            int V2;
            int M;
         } climb;
         struct {
            int V1;
            int V2;
            int M;
         } cruise;
         struct {
            int V1;
            int V2;
            int M;
         } descent;
      };

      Bada::Procedure GetProcedureInformation(unsigned int index) const;

      struct AircraftPerformance {
         struct {
            perf_speed_struct climb;
            perf_speed_struct cruise;
            perf_speed_struct descent;
         } speed;

         struct {
            Units::Mass low;
            Units::Mass nominal;
            Units::Mass high;
         } mass;

         cruise_struct cruise[FL_NMAX];
         climb_struct climb[FL_NMAX];
         descent_struct descent[FL_NMAX];
      };

      Bada::AircraftPerformance GetAircraftPerformanceInformation() const;

      const Bada &operator=(const Bada &obj);

      bool operator==(const Bada &obj) const;

    protected:
      static char input_path[512];

      // Aircraft Type
      char *type_name;

      // Base file name
      std::string base_name;

      Bada::AircraftType m_aircraft_type;
      Bada::AircraftPerformance m_performance;
      Bada::GroundMovement m_ground_movement;
      Bada::EngineThrust m_engine_thrust;
      Bada::FuelFlow m_fuel_flow;
      Bada::Mass m_mass;
      Bada::Aerodynamics m_aerodynamics;
      Bada::FlightEnvelope m_flight_envelope;
      Bada::Procedure m_procedures[3];

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
      static std::map<std::string, Bada*> m_loaded_bada_performance;
   };

   inline Bada::AircraftType Bada::GetAircraftTypeInformation() const {
      return m_aircraft_type;
   }

   inline Bada::AircraftPerformance Bada::GetAircraftPerformanceInformation() const {
      return m_performance;
   }

   inline Bada::GroundMovement Bada::GetGroundMovementInformation() const {
      return m_ground_movement;
   }

   inline Bada::EngineThrust Bada::GetEngineThrustInformation() const {
      return m_engine_thrust;
   }

   inline Bada::FuelFlow Bada::GetFuelFlowInformation() const {
      return m_fuel_flow;
   }

   inline Bada::Mass Bada::GetAircraftMassInformation() const {
      return m_mass;
   }

   inline Bada::Aerodynamics Bada::GetAerodynamicsInformation() const {
      return m_aerodynamics;
   }

   inline Bada::FlightEnvelope Bada::GetFlightEnvelopeInformation() const {
      return m_flight_envelope;
   }

   inline Bada::Procedure Bada::GetProcedureInformation(unsigned int index) const {
      if (index >= sizeof(m_procedures)) {
         throw std::runtime_error("invalid array index");
      }
      return m_procedures[index];
   }

}