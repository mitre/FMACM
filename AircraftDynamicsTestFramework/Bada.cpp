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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "aaesim/Bada.h"

#include <string.h>

using namespace std;

char Bada::input_path[512];

/**
 * Maps the aircraft type to an 8-character file name (no extension).
 * Multiple type codes may map to the same value.
 */
string Bada::get_file(char *type_code /** aircraft type abbreviation, e.g. B737 */) {
   return (char *) NULL;
}

/**
 * Reads the .OPF file, which contains all the thrust,
 * drag and fuel coefficients to be used in TEM together
 * with information on weights, speeds, maximum altitude, etc.
 */
bool Bada::read_OPF() {
   return false;
}

/**
 * Reads the .APF file, which contains a default operational
 * climb, cruise and descent speed schedule that is likely
 * to be used by an airline.
 */
bool Bada::read_APF() {
   return false;
}

/**
 * Reads the .PTF file, which contains a table of performance
 * data.
 */
bool Bada::read_PTF() {
   return false;
}

/**
 * Writes out any relevant data about the Bada object.
 */
void Bada::dump() {
}

/**
 * The init() function reads in new data from somewhere
 * based on type_code.  It calls get_file to map the
 * type to a file name and then read_APF, read_OPF, and
 * read_PTF.
 */
void Bada::init(char *type_code /** aircraft type abbreviation, e.g. B737 */ ) {
}

Bada::Bada() {
   cout << "Bada.cpp: Implement this class!" << endl;
   mass.m_ref = Units::KilogramsMass(-999);
}

Bada &Bada::operator=(const Bada &obj) {

   if (this != &obj) {
      if (obj.type_name != NULL) {
         type_name = new char(sizeof(obj.type_name));
         strcpy(type_name, obj.type_name);
      } else {
         type_name = NULL;
      }

      base_name = obj.base_name;
      aircraft_type = obj.aircraft_type;
      mass = obj.mass;
      flight_envelope = obj.flight_envelope;
      aerodynamics = obj.aerodynamics;
      engine_thrust = obj.engine_thrust;
      fuel_flow = obj.fuel_flow;
      ground_movement = obj.ground_movement;

      for (auto ix = 0; ix < 3; ix++) {
         procedures[ix] = obj.procedures[ix];
      }

      performance.speed = obj.performance.speed;
      performance.mass = obj.performance.mass;
      for (auto ix = 0; ix < FL_NMAX; ix++) {
         performance.cruise[ix] = obj.performance.cruise[ix];
         performance.climb[ix] = obj.performance.climb[ix];
         performance.descent[ix] = obj.performance.descent[ix];
      }

   }

   return *this;
}


bool Bada::operator==(const Bada &obj) const {
   bool match(true);

   if ((type_name == NULL) && (obj.type_name == NULL)) {
      match = true;
   } else if ((type_name == NULL) || (obj.type_name == NULL)) {
      match = false;
   } else {
      match = (strcmp(type_name, obj.type_name) == 0);
   }

   match = match && (base_name == obj.base_name);


   // aircraft type

   match = match && (aircraft_type.n_eng == obj.aircraft_type.n_eng);
   match = match && (aircraft_type.engine_type == obj.aircraft_type.engine_type);
   match = match && (aircraft_type.wake_category == obj.aircraft_type.wake_category);


   // mass

   match = match && (mass.m_ref == obj.mass.m_ref);
   match = match && (mass.m_min == obj.mass.m_min);
   match = match && (mass.m_max == obj.mass.m_max);
   match = match && (mass.m_pyld == obj.mass.m_pyld);


   // flight envelope

   match = match && (flight_envelope.V_mo == obj.flight_envelope.V_mo);
   match = match && (flight_envelope.M_mo == obj.flight_envelope.M_mo);
   match = match && (flight_envelope.h_mo == obj.flight_envelope.h_mo);
   match = match && (flight_envelope.h_max == obj.flight_envelope.h_max);
   match = match && (flight_envelope.G_w == obj.flight_envelope.G_w);


   // aerodynamics

   match = match && (aerodynamics.S == obj.aerodynamics.S);
   match = match && (aerodynamics.C_Lbo == obj.aerodynamics.C_Lbo);
   match = match && (aerodynamics.K == obj.aerodynamics.K);
   match = match && (aerodynamics.C_M16 == obj.aerodynamics.C_M16);

   match = match && (aerodynamics.cruise.V_stall == obj.aerodynamics.cruise.V_stall);
   match = match && (aerodynamics.cruise.cd0 == obj.aerodynamics.cruise.cd0);
   match = match && (aerodynamics.cruise.cd2 == obj.aerodynamics.cruise.cd2);

   match = match && (aerodynamics.initial_climb.V_stall == obj.aerodynamics.initial_climb.V_stall);
   match = match && (aerodynamics.initial_climb.cd0 == obj.aerodynamics.initial_climb.cd0);
   match = match && (aerodynamics.initial_climb.cd2 == obj.aerodynamics.initial_climb.cd2);

   match = match && (aerodynamics.take_off.V_stall == obj.aerodynamics.take_off.V_stall);
   match = match && (aerodynamics.take_off.cd0 == obj.aerodynamics.take_off.cd0);
   match = match && (aerodynamics.take_off.cd2 == obj.aerodynamics.take_off.cd2);

   match = match && (aerodynamics.approach.V_stall == obj.aerodynamics.approach.V_stall);
   match = match && (aerodynamics.approach.cd0 == obj.aerodynamics.approach.cd0);
   match = match && (aerodynamics.approach.cd2 == obj.aerodynamics.approach.cd2);

   match = match && (aerodynamics.landing.V_stall == obj.aerodynamics.landing.V_stall);
   match = match && (aerodynamics.landing.cd0 == obj.aerodynamics.landing.cd0);
   match = match && (aerodynamics.landing.cd2 == obj.aerodynamics.landing.cd2);

   match = match && (aerodynamics.landing_gear.cd0 == obj.aerodynamics.landing_gear.cd0);


   // engine thrust

   match = match && (engine_thrust.max_climb.CT_c1 == obj.engine_thrust.max_climb.CT_c1);
   match = match && (engine_thrust.max_climb.CT_c2 == obj.engine_thrust.max_climb.CT_c2);
   match = match && (engine_thrust.max_climb.CT_c3 == obj.engine_thrust.max_climb.CT_c3);
   match = match && (engine_thrust.max_climb.CT_c4 == obj.engine_thrust.max_climb.CT_c4);
   match = match && (engine_thrust.max_climb.CT_c5 == obj.engine_thrust.max_climb.CT_c5);

   match = match && (engine_thrust.descent.CT_low == obj.engine_thrust.descent.CT_low);
   match = match && (engine_thrust.descent.CT_high == obj.engine_thrust.descent.CT_high);
   match = match && (engine_thrust.descent.h == obj.engine_thrust.descent.h);
   match = match && (engine_thrust.descent.CT_app == obj.engine_thrust.descent.CT_app);
   match = match && (engine_thrust.descent.CT_ld == obj.engine_thrust.descent.CT_ld);
   match = match && (engine_thrust.descent.V_ref == obj.engine_thrust.descent.V_ref);
   match = match && (engine_thrust.descent.M_ref == obj.engine_thrust.descent.M_ref);


   // fuel flow

   match = match && (fuel_flow.C_f1 == obj.fuel_flow.C_f1);
   match = match && (fuel_flow.C_f2 == obj.fuel_flow.C_f2);
   match = match && (fuel_flow.C_f3 == obj.fuel_flow.C_f3);
   match = match && (fuel_flow.C_f4 == obj.fuel_flow.C_f4);
   match = match && (fuel_flow.C_fcr == obj.fuel_flow.C_fcr);


   // ground movement

   match = match && (ground_movement.TOL == obj.ground_movement.TOL);
   match = match && (ground_movement.LDL == obj.ground_movement.LDL);
   match = match && (ground_movement.span == obj.ground_movement.span);
   match = match && (ground_movement.length == obj.ground_movement.length);


   // procedures

   for (auto ix = 0; match && (ix < 3); ix++) {
      match = match && (procedures[ix].climb.V1 == obj.procedures[ix].climb.V1);
      match = match && (procedures[ix].climb.V2 == obj.procedures[ix].climb.V2);
      match = match && (procedures[ix].climb.M == obj.procedures[ix].climb.M);

      match = match && (procedures[ix].cruise.V1 == obj.procedures[ix].cruise.V1);
      match = match && (procedures[ix].cruise.V2 == obj.procedures[ix].cruise.V2);
      match = match && (procedures[ix].cruise.M == obj.procedures[ix].cruise.M);

      match = match && (procedures[ix].descent.V1 == obj.procedures[ix].descent.V1);
      match = match && (procedures[ix].descent.V2 == obj.procedures[ix].descent.V2);
      match = match && (procedures[ix].descent.M == obj.procedures[ix].descent.M);
   }


   // performance

   match = match && (performance.speed.climb.CAS.LO == obj.performance.speed.climb.CAS.LO);
   match = match && (performance.speed.climb.CAS.HI == obj.performance.speed.climb.CAS.HI);
   match = match && (performance.speed.climb.Mach == obj.performance.speed.climb.Mach);

   match = match && (performance.speed.cruise.CAS.LO == obj.performance.speed.cruise.CAS.LO);
   match = match && (performance.speed.cruise.CAS.HI == obj.performance.speed.cruise.CAS.HI);
   match = match && (performance.speed.cruise.Mach == obj.performance.speed.cruise.Mach);

   match = match && (performance.speed.descent.CAS.LO == obj.performance.speed.descent.CAS.LO);
   match = match && (performance.speed.descent.CAS.HI == obj.performance.speed.descent.CAS.HI);
   match = match && (performance.speed.descent.Mach == obj.performance.speed.descent.Mach);

   match = match && (performance.mass.low == obj.performance.mass.low);
   match = match && (performance.mass.nominal == obj.performance.mass.nominal);
   match = match && (performance.mass.high == obj.performance.mass.high);


   for (auto ix = 0; match && (ix < FL_NMAX); ix++) {
      match = match && (performance.cruise[ix].FL == obj.performance.cruise[ix].FL);
      match = match && ((isnan(Units::MetersPerSecondSpeed(performance.cruise[ix].TAS).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.performance.cruise[ix].TAS).value())) ||
                        (performance.cruise[ix].TAS == obj.performance.cruise[ix].TAS));

      match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(performance.cruise[ix].fuel.lo).value()) &&
                         isnan(Units::KilogramsPerHourMassFlowRate(obj.performance.cruise[ix].fuel.lo).value())) ||
                        (performance.cruise[ix].fuel.lo == obj.performance.cruise[ix].fuel.lo));
      match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(performance.cruise[ix].fuel.nom).value()) &&
                         isnan(Units::KilogramsPerHourMassFlowRate(obj.performance.cruise[ix].fuel.nom).value())) ||
                        (performance.cruise[ix].fuel.nom == obj.performance.cruise[ix].fuel.nom));
      match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(performance.cruise[ix].fuel.hi).value()) &&
                         isnan(Units::KilogramsPerHourMassFlowRate(obj.performance.cruise[ix].fuel.hi).value())) ||
                        (performance.cruise[ix].fuel.hi == obj.performance.cruise[ix].fuel.hi));


      match = match && (performance.climb[ix].FL == obj.performance.climb[ix].FL);
      match = match && ((isnan(Units::MetersPerSecondSpeed(performance.climb[ix].TAS).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.performance.climb[ix].TAS).value())) ||
                        (performance.climb[ix].TAS == obj.performance.climb[ix].TAS));

      match = match && ((isnan(Units::MetersPerSecondSpeed(performance.climb[ix].ROCD.lo).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.performance.climb[ix].ROCD.lo).value())) ||
                        (performance.climb[ix].ROCD.lo == obj.performance.climb[ix].ROCD.lo));
      match = match && ((isnan(Units::MetersPerSecondSpeed(performance.climb[ix].ROCD.nom).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.performance.climb[ix].ROCD.nom).value())) ||
                        (performance.climb[ix].ROCD.nom == obj.performance.climb[ix].ROCD.nom));
      match = match && ((isnan(Units::MetersPerSecondSpeed(performance.climb[ix].ROCD.hi).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.performance.climb[ix].ROCD.hi).value())) ||
                        (performance.climb[ix].ROCD.hi == obj.performance.climb[ix].ROCD.hi));

      match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(performance.climb[ix].fuel).value()) &&
                         isnan(Units::KilogramsPerHourMassFlowRate(obj.performance.climb[ix].fuel).value())) ||
                        (performance.climb[ix].fuel == obj.performance.climb[ix].fuel));


      match = match && (performance.descent[ix].FL == obj.performance.descent[ix].FL);

      match = match && ((isnan(Units::MetersPerSecondSpeed(performance.descent[ix].TAS).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.performance.descent[ix].TAS).value())) ||
                        (performance.descent[ix].TAS == obj.performance.descent[ix].TAS));

      match = match && ((isnan(Units::MetersPerSecondSpeed(performance.descent[ix].ROCD).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.performance.descent[ix].ROCD).value())) ||
                        (performance.descent[ix].ROCD == obj.performance.descent[ix].ROCD));

      match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(performance.descent[ix].fuel).value()) &&
                         isnan(Units::KilogramsPerHourMassFlowRate(obj.performance.descent[ix].fuel).value())) ||
                        (performance.descent[ix].fuel == obj.performance.descent[ix].fuel));

   }

   return match;
}

