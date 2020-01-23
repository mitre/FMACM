// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

/* bada_class.h		Initial code 10/09/2009 
    REVISION HISTORY:
      DATE              INITIALS                REASON

	10/09/2009            jws                   create

*/

#pragma once

#include <Area.h>
#include <Length.h>
#include <Mass.h>
#include <MassFlowRate.h>
#include <Speed.h>
#include "utility/CustomUnits.h"


enum ENGINE_TYPE
{
   JET,
   TURBOPROP,
   PISTON
};
enum WAKE_CATEGORY
{
   HEAVY_,
   MEDIUM_,
   LIGHT_
};
#define FL_NMAX 40

typedef struct
{
   int FL;
   Units::Speed TAS;
   struct
   {
      Units::Speed lo;
      Units::Speed nom;
      Units::Speed hi;
   } ROCD;
   Units::MassFlowRate fuel;
} climb_struct;

typedef struct
{
   int FL;
   Units::Speed TAS;
   struct
   {
      Units::MassFlowRate lo;
      Units::MassFlowRate nom;
      Units::MassFlowRate hi;
   } fuel;
} cruise_struct;

typedef struct
{
   int FL;
   Units::Speed TAS;
   Units::Speed ROCD;
   Units::MassFlowRate fuel;
} descent_struct;

typedef struct
{
   struct
   {
      int LO;
      int HI;
   } CAS;
   double Mach;
} perf_speed_struct;


class Bada
{
public:
   // Aircraft Type
   char *type_name;

   // Base file name
   std::string base_name;
   static char input_path[512];

   struct
   {
      int n_eng;  // number of engines
      ENGINE_TYPE engine_type;
      WAKE_CATEGORY wake_category;
   } aircraft_type;

   // Mass
   struct
   {
      Units::Mass m_ref;  // reference mass
      Units::Mass m_min;  // minimum mass
      Units::Mass m_max;  // maximum mass
      Units::Mass m_pyld; // maximum payload mass
   } mass;

   // Flight Envelope
   struct
   {
      Units::Speed V_mo;   // maximum operating speed (CAS).
      double M_mo;         // maximum operating Mach number
      Units::Length h_mo;  // maximum operating altitude
      Units::Length h_max; // maximum altitude at MTOW and ISA
      Units::LengthToMassGradient G_w; // weight gradient on max. altitude
      double G_t;     // temperature gradient on max. altitude (feet/C)
   } flight_envelope;

   // Aerodynamics
   struct
   {
      Units::Area S; // Reference wing surface area.
      double C_Lbo;  // Buffet onset lift coeff.  (jet only)
      double K;    // Buffeting gradient (jet only)
      double C_M16;    // No idea, but it's in the OPF

      struct
      {
         Units::Speed V_stall; // CAS
         double cd0;           // parasitic drag coeff.
         double cd2;           // induced drag coeff.
      } cruise;

      struct
      {
         Units::Speed V_stall; // CAS
         double cd0;           // parasitic drag coeff.
         double cd2;           // induced drag coeff.
      } initial_climb;

      struct
      {
         Units::Speed V_stall; // CAS
         double cd0;           // parasitic drag coeff.
         double cd2;           // induced drag coeff.
      } take_off;

      struct
      {
         Units::Speed V_stall; // CAS
         double cd0;           // parasitic drag coeff.
         double cd2;           // induced drag coeff.
      } approach;

      struct
      {
         Units::Speed V_stall; // CAS
         double cd0;           // parasitic drag coeff.
         double cd2;           // induced drag coeff.
      } landing;

      struct
      {
         double cd0;  // parasitic drag coeff.
      } landing_gear;

   } aerodynamics;

   // Engine Thrust
   struct
   {
      struct
      {
         double CT_c1;                // 1st max. climb thrust coeff. (N, jet/piston)
         //                              (kt-N, turboprop)
         Units::Length CT_c2;         // 2nd max. climb thrust coeff.
         double CT_c3;                // 3rd max. climb thrust coeff. (1/feet^2, jet)
         //                              (N, turboprop)
         //                              (kt-N, piston)
         Units::AbsTemperature CT_c4; // 1st thrust temperature coeff.
         double CT_c5;                // 2nd thrust temperature coeff. (1/deg. C)
      } max_climb;
      struct
      {
         double CT_low;      // low altitude descent thrust coeff.
         double CT_high;     // high altitude descent thrust coeff.
         Units::Length h;    // Transition altitude (feet)
         double CT_app;      // approach thrust coeff.
         double CT_ld;       // landing thrust coeff.
         Units::Speed V_ref; // reference descent speed (kt)
         double M_ref;       // reference descent Mach number
      } descent;
   } engine_thrust;

   // fuel flow
   struct
   {
      double C_f1;              // 1st Thrust specific fuel consumption coeff.
      // (kg/min*kN) (jet)
      // (kg/min*kN*knot) (turboprop)
      // (kg/min)  (piston)
      Units::Speed C_f2;        // 2nd Thrust specific fuel consumption coeff.
      Units::MassFlowRate C_f3; // 1st descent thrust fuel flow coeff.
      Units::Length C_f4;       // 2nd descent thrust fuel flow coeff.
      double C_fcr;             // Cruise fuel flow coeff. (dimensionless)
   } fuel_flow;

   // Ground movement
   struct
   {
      Units::Length TOL;     // Take-off length (m)
      Units::Length LDL;     // Landing length (m)
      Units::Length span;    // Wingspan (m)
      Units::Length length;  // Length (m)
   } ground_movement;

   // Procedure Specifications
   struct
   {
      struct
      {
         int V1;
         int V2;
         int M;
      } climb;
      struct
      {
         int V1;
         int V2;
         int M;
      } cruise;
      struct
      {
         int V1;
         int V2;
         int M;
      } descent;
   } procedures[3];

   struct
   {
      struct
      {
         perf_speed_struct climb;
         perf_speed_struct cruise;
         perf_speed_struct descent;
      } speed;

      struct
      {
         Units::Mass low;
         Units::Mass nominal;
         Units::Mass high;
      } mass;

      cruise_struct cruise[FL_NMAX];
      climb_struct climb[FL_NMAX];
      descent_struct descent[FL_NMAX];
   } performance;

   void init(char *type_code);

   bool read_OPF();

   bool read_APF();

   bool read_PTF();

   Bada &operator=(const Bada &obj);

   bool operator==(const Bada &obj) const;

   void dump();


   Bada();

   /**
     * Maps the aircraft type to an 8-character file name (no extension).
     * Multiple type codes may map to the same value.
     */
   std::string get_file(char *type_code /** aircraft type abbreviation, e.g. B737 */);

private:

   void setDefaults(void);

};
