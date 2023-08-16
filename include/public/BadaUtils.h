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

#include <stdexcept>
#include "utility/CustomUnits.h"
#include "scalar/Area.h"

namespace aaesim {
namespace open_source {
namespace bada_utils {

enum ENGINE_TYPE { JET, TURBOPROP, PISTON };

enum WAKE_CATEGORY { HEAVY_, MEDIUM_, LIGHT_ };

const int FL_NMAX(40);

enum FlapConfiguration {
   UNDEFINED = -1,
   TAKEOFF = 0,
   INITIAL_CLIMB = 1,
   CRUISE = 2,
   APPROACH = 3,
   LANDING = 4,
   GEAR_DOWN = 5
};

static std::string GetFlapConfigurationAsString(FlapConfiguration flap_configuration) {
   switch (flap_configuration) {
      case UNDEFINED:
         return "UNDEFINED";
      case TAKEOFF:
         return "TAKEOFF";
      case INITIAL_CLIMB:
         return "INITIAL_CLIMB";
      case CRUISE:
         return "CRUISE";
      case APPROACH:
         return "APPROACH";
      case LANDING:
         return "LANDING";
      case GEAR_DOWN:
         return "GEAR_DOWN";
      default:
         throw std::runtime_error("Invalid flap_configuration encountered: " + std::to_string(flap_configuration));
   }
}

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

enum EngineThrustMode { MAXIMUM_CLIMB = 0, MAXIMUM_CRUISE, DESCENT };

struct FlightEnvelope {
   Units::Speed V_mo;                // maximum operating speed (CAS).
   double M_mo;                      // maximum operating Mach number
   Units::Length h_mo;               // maximum operating altitude
   Units::Length h_max;              // maximum altitude at MTOW and ISA
   Units::LengthToMassGradient G_w;  // weight gradient on max. altitude
   double G_t;                       // temperature gradient on max. altitude (feet/C)
};

struct FlapSpeeds {
   Units::Speed cas_approach_minimum;
   Units::Speed cas_approach_maximum;
   Units::Speed cas_landing_minimum;
   Units::Speed cas_landing_maximum;
   Units::Speed cas_gear_out_minimum;
   Units::Speed cas_gear_out_maximum;
   Units::Speed cas_takeoff_minimum;
   Units::Speed cas_climb_minimum;
   Units::Speed cas_cruise_minimum;
};

struct Mass {
   Units::Mass m_ref;   // reference mass
   Units::Mass m_min;   // minimum mass
   Units::Mass m_max;   // maximum mass
   Units::Mass m_pyld;  // maximum payload mass
};

struct Aerodynamics {
   Units::Area S;  // Reference wing surface area.
   double C_Lbo;   // Buffet onset lift coeff.  (jet only)
   double K;       // Buffeting gradient (jet only)
   double C_M16;   // No idea, but it's in the OPF

   struct {
      Units::Speed V_stall;  // CAS
      double cd0;            // parasitic drag coeff.
      double cd2;            // induced drag coeff.
   } cruise;

   struct {
      Units::Speed V_stall;  // CAS
      double cd0;            // parasitic drag coeff.
      double cd2;            // induced drag coeff.
   } initial_climb;

   struct {
      Units::Speed V_stall;  // CAS
      double cd0;            // parasitic drag coeff.
      double cd2;            // induced drag coeff.
   } take_off;

   struct {
      Units::Speed V_stall;  // CAS
      double cd0;            // parasitic drag coeff.
      double cd2;            // induced drag coeff.
   } approach;

   struct {
      Units::Speed V_stall;  // CAS
      double cd0;            // parasitic drag coeff.
      double cd2;            // induced drag coeff.
   } landing;

   struct {
      double cd0;  // parasitic drag coeff.
   } landing_gear;
};

struct EngineThrust {
   struct {
      double CT_c1;  // 1st max. climb thrust coeff. (N, jet/piston)
      //                              (kt-N, turboprop)
      Units::Length CT_c2;  // 2nd max. climb thrust coeff.
      double CT_c3;         // 3rd max. climb thrust coeff. (1/feet^2, jet)
      //                              (N, turboprop)
      //                              (kt-N, piston)
      Units::AbsCelsiusTemperature CT_c4;  // 1st thrust temperature coeff.
      double CT_c5;                        // 2nd thrust temperature coeff. (1/deg. C)
   } max_climb;
   struct {
      double CT_low;       // low altitude descent thrust coeff.
      double CT_high;      // high altitude descent thrust coeff.
      Units::Length h;     // Transition altitude (feet)
      double CT_app;       // approach thrust coeff.
      double CT_ld;        // landing thrust coeff.
      Units::Speed V_ref;  // reference descent speed (kt)
      double M_ref;        // reference descent Mach number
   } descent;
};

struct AircraftType {
   int n_eng;  // number of engines
   ENGINE_TYPE engine_type;
   WAKE_CATEGORY wake_category;
};

struct FuelFlow {
   double C_f1;  // 1st Thrust specific fuel consumption coeff.
   // (kg/min*kN) (jet)
   // (kg/min*kN*knot) (turboprop)
   // (kg/min)  (piston)
   Units::Speed C_f2;         // 2nd Thrust specific fuel consumption coeff.
   Units::MassFlowRate C_f3;  // 1st descent thrust fuel flow coeff.
   Units::Length C_f4;        // 2nd descent thrust fuel flow coeff.
   double C_fcr;              // Cruise fuel flow coeff. (dimensionless)
};

struct GroundMovement {
   Units::Length TOL;     // Take-off length (m)
   Units::Length LDL;     // Landing length (m)
   Units::Length span;    // Wingspan (m)
   Units::Length length;  // Length (m)
};

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

}  // namespace bada_utils
}  // namespace open_source
}  // namespace aaesim