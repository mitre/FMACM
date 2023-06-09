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

#include "public/BadaUtils.h"
#include "aaesim/Bada.h"

#include <string.h>

using namespace std;
using namespace aaesim;

char Bada::input_path[512];

/**
 * Maps the aircraft type to an 8-character file name (no extension).
 * Multiple type codes may map to the same value.
 */
string Bada::get_file(char *type_code /** aircraft type abbreviation, e.g. B737 */) { return (char *)NULL; }

/**
 * Reads the .OPF file, which contains all the thrust,
 * drag and fuel coefficients to be used in TEM together
 * with information on weights, speeds, maximum altitude, etc.
 */
bool Bada::read_OPF() { return false; }

/**
 * Reads the .APF file, which contains a default operational
 * climb, cruise and descent speed schedule that is likely
 * to be used by an airline.
 */
bool Bada::read_APF() { return false; }

/**
 * Reads the .PTF file, which contains a table of performance
 * data.
 */
bool Bada::read_PTF() { return false; }

/**
 * The init() function reads in new data from somewhere
 * based on type_code.  It calls get_file to map the
 * type to a file name and then read_APF, read_OPF, and
 * read_PTF.
 */
void Bada::init(char *type_code /** aircraft type abbreviation, e.g. B737 */) {}

const Bada &Bada::operator=(const Bada &obj) {

   if (this != &obj) {
      if (obj.type_name != NULL) {
         type_name = new char(sizeof(obj.type_name));
         strcpy(type_name, obj.type_name);
      } else {
         type_name = NULL;
      }

      base_name = obj.base_name;
      m_aircraft_type = obj.m_aircraft_type;
      m_mass = obj.m_mass;
      m_flight_envelope = obj.m_flight_envelope;
      m_aerodynamics = obj.m_aerodynamics;
      m_engine_thrust = obj.m_engine_thrust;
      m_fuel_flow = obj.m_fuel_flow;
      m_ground_movement = obj.m_ground_movement;

      for (auto ix = 0; ix < 3; ix++) {
         m_procedures[ix] = obj.m_procedures[ix];
      }

      m_performance.speed = obj.m_performance.speed;
      m_performance.mass = obj.m_performance.mass;
      for (auto ix = 0; ix < aaesim::open_source::bada_utils::FL_NMAX; ix++) {
         m_performance.cruise[ix] = obj.m_performance.cruise[ix];
         m_performance.climb[ix] = obj.m_performance.climb[ix];
         m_performance.descent[ix] = obj.m_performance.descent[ix];
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

   match = match && (m_aircraft_type.n_eng == obj.m_aircraft_type.n_eng);
   match = match && (m_aircraft_type.engine_type == obj.m_aircraft_type.engine_type);
   match = match && (m_aircraft_type.wake_category == obj.m_aircraft_type.wake_category);

   // mass

   match = match && (m_mass.m_ref == obj.m_mass.m_ref);
   match = match && (m_mass.m_min == obj.m_mass.m_min);
   match = match && (m_mass.m_max == obj.m_mass.m_max);
   match = match && (m_mass.m_pyld == obj.m_mass.m_pyld);

   // flight envelope

   match = match && (m_flight_envelope.V_mo == obj.m_flight_envelope.V_mo);
   match = match && (m_flight_envelope.M_mo == obj.m_flight_envelope.M_mo);
   match = match && (m_flight_envelope.h_mo == obj.m_flight_envelope.h_mo);
   match = match && (m_flight_envelope.h_max == obj.m_flight_envelope.h_max);
   match = match && (m_flight_envelope.G_w == obj.m_flight_envelope.G_w);

   // m_aerodynamics

   match = match && (m_aerodynamics.S == obj.m_aerodynamics.S);
   match = match && (m_aerodynamics.C_Lbo == obj.m_aerodynamics.C_Lbo);
   match = match && (m_aerodynamics.K == obj.m_aerodynamics.K);
   match = match && (m_aerodynamics.C_M16 == obj.m_aerodynamics.C_M16);

   match = match && (m_aerodynamics.cruise.V_stall == obj.m_aerodynamics.cruise.V_stall);
   match = match && (m_aerodynamics.cruise.cd0 == obj.m_aerodynamics.cruise.cd0);
   match = match && (m_aerodynamics.cruise.cd2 == obj.m_aerodynamics.cruise.cd2);

   match = match && (m_aerodynamics.initial_climb.V_stall == obj.m_aerodynamics.initial_climb.V_stall);
   match = match && (m_aerodynamics.initial_climb.cd0 == obj.m_aerodynamics.initial_climb.cd0);
   match = match && (m_aerodynamics.initial_climb.cd2 == obj.m_aerodynamics.initial_climb.cd2);

   match = match && (m_aerodynamics.take_off.V_stall == obj.m_aerodynamics.take_off.V_stall);
   match = match && (m_aerodynamics.take_off.cd0 == obj.m_aerodynamics.take_off.cd0);
   match = match && (m_aerodynamics.take_off.cd2 == obj.m_aerodynamics.take_off.cd2);

   match = match && (m_aerodynamics.approach.V_stall == obj.m_aerodynamics.approach.V_stall);
   match = match && (m_aerodynamics.approach.cd0 == obj.m_aerodynamics.approach.cd0);
   match = match && (m_aerodynamics.approach.cd2 == obj.m_aerodynamics.approach.cd2);

   match = match && (m_aerodynamics.landing.V_stall == obj.m_aerodynamics.landing.V_stall);
   match = match && (m_aerodynamics.landing.cd0 == obj.m_aerodynamics.landing.cd0);
   match = match && (m_aerodynamics.landing.cd2 == obj.m_aerodynamics.landing.cd2);

   match = match && (m_aerodynamics.landing_gear.cd0 == obj.m_aerodynamics.landing_gear.cd0);

   // engine thrust

   match = match && (m_engine_thrust.max_climb.CT_c1 == obj.m_engine_thrust.max_climb.CT_c1);
   match = match && (m_engine_thrust.max_climb.CT_c2 == obj.m_engine_thrust.max_climb.CT_c2);
   match = match && (m_engine_thrust.max_climb.CT_c3 == obj.m_engine_thrust.max_climb.CT_c3);
   match = match && (m_engine_thrust.max_climb.CT_c4 == obj.m_engine_thrust.max_climb.CT_c4);
   match = match && (m_engine_thrust.max_climb.CT_c5 == obj.m_engine_thrust.max_climb.CT_c5);

   match = match && (m_engine_thrust.descent.CT_low == obj.m_engine_thrust.descent.CT_low);
   match = match && (m_engine_thrust.descent.CT_high == obj.m_engine_thrust.descent.CT_high);
   match = match && (m_engine_thrust.descent.h == obj.m_engine_thrust.descent.h);
   match = match && (m_engine_thrust.descent.CT_app == obj.m_engine_thrust.descent.CT_app);
   match = match && (m_engine_thrust.descent.CT_ld == obj.m_engine_thrust.descent.CT_ld);
   match = match && (m_engine_thrust.descent.V_ref == obj.m_engine_thrust.descent.V_ref);
   match = match && (m_engine_thrust.descent.M_ref == obj.m_engine_thrust.descent.M_ref);

   // fuel flow

   match = match && (m_fuel_flow.C_f1 == obj.m_fuel_flow.C_f1);
   match = match && (m_fuel_flow.C_f2 == obj.m_fuel_flow.C_f2);
   match = match && (m_fuel_flow.C_f3 == obj.m_fuel_flow.C_f3);
   match = match && (m_fuel_flow.C_f4 == obj.m_fuel_flow.C_f4);
   match = match && (m_fuel_flow.C_fcr == obj.m_fuel_flow.C_fcr);

   // ground movement

   match = match && (m_ground_movement.TOL == obj.m_ground_movement.TOL);
   match = match && (m_ground_movement.LDL == obj.m_ground_movement.LDL);
   match = match && (m_ground_movement.span == obj.m_ground_movement.span);
   match = match && (m_ground_movement.length == obj.m_ground_movement.length);

   // procedures

   for (auto ix = 0; match && (ix < 3); ix++) {
      match = match && (m_procedures[ix].climb.V1 == obj.m_procedures[ix].climb.V1);
      match = match && (m_procedures[ix].climb.V2 == obj.m_procedures[ix].climb.V2);
      match = match && (m_procedures[ix].climb.M == obj.m_procedures[ix].climb.M);

      match = match && (m_procedures[ix].cruise.V1 == obj.m_procedures[ix].cruise.V1);
      match = match && (m_procedures[ix].cruise.V2 == obj.m_procedures[ix].cruise.V2);
      match = match && (m_procedures[ix].cruise.M == obj.m_procedures[ix].cruise.M);

      match = match && (m_procedures[ix].descent.V1 == obj.m_procedures[ix].descent.V1);
      match = match && (m_procedures[ix].descent.V2 == obj.m_procedures[ix].descent.V2);
      match = match && (m_procedures[ix].descent.M == obj.m_procedures[ix].descent.M);
   }

   // performance

   match = match && (m_performance.speed.climb.CAS.LO == obj.m_performance.speed.climb.CAS.LO);
   match = match && (m_performance.speed.climb.CAS.HI == obj.m_performance.speed.climb.CAS.HI);
   match = match && (m_performance.speed.climb.Mach == obj.m_performance.speed.climb.Mach);

   match = match && (m_performance.speed.cruise.CAS.LO == obj.m_performance.speed.cruise.CAS.LO);
   match = match && (m_performance.speed.cruise.CAS.HI == obj.m_performance.speed.cruise.CAS.HI);
   match = match && (m_performance.speed.cruise.Mach == obj.m_performance.speed.cruise.Mach);

   match = match && (m_performance.speed.descent.CAS.LO == obj.m_performance.speed.descent.CAS.LO);
   match = match && (m_performance.speed.descent.CAS.HI == obj.m_performance.speed.descent.CAS.HI);
   match = match && (m_performance.speed.descent.Mach == obj.m_performance.speed.descent.Mach);

   match = match && (m_performance.mass.low == obj.m_performance.mass.low);
   match = match && (m_performance.mass.nominal == obj.m_performance.mass.nominal);
   match = match && (m_performance.mass.high == obj.m_performance.mass.high);

   for (auto ix = 0; match && (ix < aaesim::open_source::bada_utils::FL_NMAX); ix++) {
      match = match && (m_performance.cruise[ix].FL == obj.m_performance.cruise[ix].FL);
      match = match && ((isnan(Units::MetersPerSecondSpeed(m_performance.cruise[ix].TAS).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.m_performance.cruise[ix].TAS).value())) ||
                        (m_performance.cruise[ix].TAS == obj.m_performance.cruise[ix].TAS));

      match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(m_performance.cruise[ix].fuel.lo).value()) &&
                         isnan(Units::KilogramsPerHourMassFlowRate(obj.m_performance.cruise[ix].fuel.lo).value())) ||
                        (m_performance.cruise[ix].fuel.lo == obj.m_performance.cruise[ix].fuel.lo));
      match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(m_performance.cruise[ix].fuel.nom).value()) &&
                         isnan(Units::KilogramsPerHourMassFlowRate(obj.m_performance.cruise[ix].fuel.nom).value())) ||
                        (m_performance.cruise[ix].fuel.nom == obj.m_performance.cruise[ix].fuel.nom));
      match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(m_performance.cruise[ix].fuel.hi).value()) &&
                         isnan(Units::KilogramsPerHourMassFlowRate(obj.m_performance.cruise[ix].fuel.hi).value())) ||
                        (m_performance.cruise[ix].fuel.hi == obj.m_performance.cruise[ix].fuel.hi));

      match = match && (m_performance.climb[ix].FL == obj.m_performance.climb[ix].FL);
      match = match && ((isnan(Units::MetersPerSecondSpeed(m_performance.climb[ix].TAS).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.m_performance.climb[ix].TAS).value())) ||
                        (m_performance.climb[ix].TAS == obj.m_performance.climb[ix].TAS));

      match = match && ((isnan(Units::MetersPerSecondSpeed(m_performance.climb[ix].ROCD.lo).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.m_performance.climb[ix].ROCD.lo).value())) ||
                        (m_performance.climb[ix].ROCD.lo == obj.m_performance.climb[ix].ROCD.lo));
      match = match && ((isnan(Units::MetersPerSecondSpeed(m_performance.climb[ix].ROCD.nom).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.m_performance.climb[ix].ROCD.nom).value())) ||
                        (m_performance.climb[ix].ROCD.nom == obj.m_performance.climb[ix].ROCD.nom));
      match = match && ((isnan(Units::MetersPerSecondSpeed(m_performance.climb[ix].ROCD.hi).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.m_performance.climb[ix].ROCD.hi).value())) ||
                        (m_performance.climb[ix].ROCD.hi == obj.m_performance.climb[ix].ROCD.hi));

      match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(m_performance.climb[ix].fuel).value()) &&
                         isnan(Units::KilogramsPerHourMassFlowRate(obj.m_performance.climb[ix].fuel).value())) ||
                        (m_performance.climb[ix].fuel == obj.m_performance.climb[ix].fuel));

      match = match && (m_performance.descent[ix].FL == obj.m_performance.descent[ix].FL);

      match = match && ((isnan(Units::MetersPerSecondSpeed(m_performance.descent[ix].TAS).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.m_performance.descent[ix].TAS).value())) ||
                        (m_performance.descent[ix].TAS == obj.m_performance.descent[ix].TAS));

      match = match && ((isnan(Units::MetersPerSecondSpeed(m_performance.descent[ix].ROCD).value()) &&
                         isnan(Units::MetersPerSecondSpeed(obj.m_performance.descent[ix].ROCD).value())) ||
                        (m_performance.descent[ix].ROCD == obj.m_performance.descent[ix].ROCD));

      match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(m_performance.descent[ix].fuel).value()) &&
                         isnan(Units::KilogramsPerHourMassFlowRate(obj.m_performance.descent[ix].fuel).value())) ||
                        (m_performance.descent[ix].fuel == obj.m_performance.descent[ix].fuel));
   }

   return match;
}
