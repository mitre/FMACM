// ****************************************************************************
//  Copyright © 2015 The MITRE Corporation. All Rights Reserved.  
// ****************************************************************************

#include "aaesim/Bada.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <iostream>

using namespace std;

char Bada::input_path[512];

/**
 * Maps the aircraft type to an 8-character file name (no extension).
 * Multiple type codes may map to the same value.
 */
string Bada::get_file( char *type_code /** aircraft type abbreviation, e.g. B737 */) {
    return (char *)NULL;
}

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
 * Writes out any relevant data about the Bada object.
 */
void Bada::dump()
{
}  

/**
 * The init() function reads in new data from somewhere
 * based on type_code.  It calls get_file to map the
 * type to a file name and then read_APF, read_OPF, and
 * read_PTF.
 */
void Bada::init( char *type_code /** aircraft type abbreviation, e.g. B737 */ )
{
}

Bada::Bada( )
{
   cout << "Bada.cpp: Implement this class!" << endl;
   mass.m_ref = Units::KilogramsMass(-999);
}

Bada& Bada::operator=(const Bada &obj)
{

  if (this != &obj)
  {
    if (obj.type_name != NULL)
    {
      type_name = new char(sizeof(obj.type_name));
      strcpy(type_name, obj.type_name);
    }
    else
    {
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

    for (auto ix = 0;ix < 3; ix++)
    {
      procedures[ix] = obj.procedures[ix];
    }

    performance.speed = obj.performance.speed;
    performance.mass = obj.performance.mass;
    for (auto ix = 0; ix < FL_NMAX; ix++)
    {
      performance.cruise[ix] = obj.performance.cruise[ix];
      performance.climb[ix] = obj.performance.climb[ix];
      performance.descent[ix] = obj.performance.descent[ix];
    }

  }

  return *this;
}

bool Bada::operator==(const Bada &obj) const
{
  bool match(true);

  if ((this->type_name == NULL) && (obj.type_name == NULL))
  {
    match = true;
  }
  else if ((this->type_name == NULL) || (obj.type_name == NULL))
  {
    match = false;
  }
  else
  {
    match = (strcmp(this->type_name, obj.type_name) == 0);
  }

  match = match && (this->base_name == obj.base_name);


  // aircraft type

  match = match && (this->aircraft_type.n_eng == obj.aircraft_type.n_eng);
  match = match && (this->aircraft_type.engine_type == obj.aircraft_type.engine_type);
  match = match && (this->aircraft_type.wake_category == obj.aircraft_type.wake_category);


  // mass

  match = match && (this->mass.m_ref == obj.mass.m_ref);
  match = match && (this->mass.m_min == obj.mass.m_min);
  match = match && (this->mass.m_max == obj.mass.m_max);
  match = match && (this->mass.m_pyld == obj.mass.m_pyld);


  // flight envelope

  match = match && (this->flight_envelope.V_mo == obj.flight_envelope.V_mo);
  match = match && (this->flight_envelope.M_mo == obj.flight_envelope.M_mo);
  match = match && (this->flight_envelope.h_mo == obj.flight_envelope.h_mo);
  match = match && (this->flight_envelope.h_max == obj.flight_envelope.h_max);
  match = match && (this->flight_envelope.G_w == obj.flight_envelope.G_w);


  // aerodynamics

  match = match && (this->aerodynamics.S == obj.aerodynamics.S);
  match = match && (this->aerodynamics.C_Lbo == obj.aerodynamics.C_Lbo);
  match = match && (this->aerodynamics.K == obj.aerodynamics.K);
  match = match && (this->aerodynamics.C_M16 == obj.aerodynamics.C_M16);

  match = match && (this->aerodynamics.cruise.V_stall == obj.aerodynamics.cruise.V_stall);
  match = match && (this->aerodynamics.cruise.cd0 == obj.aerodynamics.cruise.cd0);
  match = match && (this->aerodynamics.cruise.cd2 == obj.aerodynamics.cruise.cd2);

  match = match && (this->aerodynamics.initial_climb.V_stall == obj.aerodynamics.initial_climb.V_stall);
  match = match && (this->aerodynamics.initial_climb.cd0 == obj.aerodynamics.initial_climb.cd0);
  match = match && (this->aerodynamics.initial_climb.cd2 == obj.aerodynamics.initial_climb.cd2);

  match = match && (this->aerodynamics.take_off.V_stall == obj.aerodynamics.take_off.V_stall);
  match = match && (this->aerodynamics.take_off.cd0 == obj.aerodynamics.take_off.cd0);
  match = match && (this->aerodynamics.take_off.cd2 == obj.aerodynamics.take_off.cd2);

  match = match && (this->aerodynamics.approach.V_stall == obj.aerodynamics.approach.V_stall);
  match = match && (this->aerodynamics.approach.cd0 == obj.aerodynamics.approach.cd0);
  match = match && (this->aerodynamics.approach.cd2 == obj.aerodynamics.approach.cd2);

  match = match && (this->aerodynamics.landing.V_stall == obj.aerodynamics.landing.V_stall);
  match = match && (this->aerodynamics.landing.cd0 == obj.aerodynamics.landing.cd0);
  match = match && (this->aerodynamics.landing.cd2 == obj.aerodynamics.landing.cd2);

  match = match && (this->aerodynamics.landing_gear.cd0 == obj.aerodynamics.landing_gear.cd0);


  // engine thrust

  match = match && (this->engine_thrust.max_climb.CT_c1 == obj.engine_thrust.max_climb.CT_c1);
  match = match && (this->engine_thrust.max_climb.CT_c2 == obj.engine_thrust.max_climb.CT_c2);
  match = match && (this->engine_thrust.max_climb.CT_c3 == obj.engine_thrust.max_climb.CT_c3);
  match = match && (this->engine_thrust.max_climb.CT_c4 == obj.engine_thrust.max_climb.CT_c4);
  match = match && (this->engine_thrust.max_climb.CT_c5 == obj.engine_thrust.max_climb.CT_c5);

  match = match && (this->engine_thrust.descent.CT_low == obj.engine_thrust.descent.CT_low);
  match = match && (this->engine_thrust.descent.CT_high == obj.engine_thrust.descent.CT_high);
  match = match && (this->engine_thrust.descent.h == obj.engine_thrust.descent.h);
  match = match && (this->engine_thrust.descent.CT_app == obj.engine_thrust.descent.CT_app);
  match = match && (this->engine_thrust.descent.CT_ld == obj.engine_thrust.descent.CT_ld);
  match = match && (this->engine_thrust.descent.V_ref == obj.engine_thrust.descent.V_ref);
  match = match && (this->engine_thrust.descent.M_ref == obj.engine_thrust.descent.M_ref);


  // fuel flow

  match = match && (this->fuel_flow.C_f1 == obj.fuel_flow.C_f1);
  match = match && (this->fuel_flow.C_f2 == obj.fuel_flow.C_f2);
  match = match && (this->fuel_flow.C_f3 == obj.fuel_flow.C_f3);
  match = match && (this->fuel_flow.C_f4 == obj.fuel_flow.C_f4);
  match = match && (this->fuel_flow.C_fcr == obj.fuel_flow.C_fcr);


  // ground movement

  match = match && (this->ground_movement.TOL == obj.ground_movement.TOL);
  match = match && (this->ground_movement.LDL == obj.ground_movement.LDL);
  match = match && (this->ground_movement.span == obj.ground_movement.span);
  match = match && (this->ground_movement.length == obj.ground_movement.length);


  // procedures

  for (auto ix = 0; match && (ix < 3); ix++)
  {
    match = match && ((isnan(this->procedures[ix].climb.V1) && isnan(obj.procedures[ix].climb.V1)) ||
		      (this->procedures[ix].climb.V1 == obj.procedures[ix].climb.V1));
    match = match && ((isnan(this->procedures[ix].climb.V2) && isnan(obj.procedures[ix].climb.V2)) ||
		      (this->procedures[ix].climb.V2 == obj.procedures[ix].climb.V2));
    match = match && ((isnan(this->procedures[ix].climb.M) && isnan(obj.procedures[ix].climb.M)) ||
		      (this->procedures[ix].climb.M == obj.procedures[ix].climb.M));

    match = match && ((isnan(this->procedures[ix].cruise.V1) && isnan(obj.procedures[ix].cruise.V1)) ||
		      (this->procedures[ix].cruise.V1 == obj.procedures[ix].cruise.V1));
    match = match && ((isnan(this->procedures[ix].cruise.V2) && isnan(obj.procedures[ix].cruise.V2)) ||
		      (this->procedures[ix].cruise.V2 == obj.procedures[ix].cruise.V2));
    match = match && ((isnan(this->procedures[ix].cruise.M) && isnan(obj.procedures[ix].cruise.M)) ||
		      (this->procedures[ix].cruise.M == obj.procedures[ix].cruise.M));

    match = match && ((isnan(this->procedures[ix].descent.V1) && isnan(obj.procedures[ix].descent.V1)) ||
		      (this->procedures[ix].descent.V1 == obj.procedures[ix].descent.V1));
    match = match && ((isnan(this->procedures[ix].descent.V2) && isnan(obj.procedures[ix].descent.V2)) ||
		      (this->procedures[ix].descent.V2 == obj.procedures[ix].descent.V2));
    match = match && ((isnan(this->procedures[ix].descent.M) && isnan(obj.procedures[ix].descent.M)) ||
		      (this->procedures[ix].descent.M == obj.procedures[ix].descent.M));
  }


  // performance

  match = match && (this->performance.speed.climb.CAS.LO == obj.performance.speed.climb.CAS.LO);
  match = match && (this->performance.speed.climb.CAS.HI == obj.performance.speed.climb.CAS.HI);
  match = match && (this->performance.speed.climb.Mach == obj.performance.speed.climb.Mach);

  match = match && (this->performance.speed.cruise.CAS.LO == obj.performance.speed.cruise.CAS.LO);
  match = match && (this->performance.speed.cruise.CAS.HI == obj.performance.speed.cruise.CAS.HI);
  match = match && (this->performance.speed.cruise.Mach == obj.performance.speed.cruise.Mach);

  match = match && (this->performance.speed.descent.CAS.LO == obj.performance.speed.descent.CAS.LO);
  match = match && (this->performance.speed.descent.CAS.HI == obj.performance.speed.descent.CAS.HI);
  match = match && (this->performance.speed.descent.Mach == obj.performance.speed.descent.Mach);

  match = match && (this->performance.mass.low == obj.performance.mass.low);
  match = match && (this->performance.mass.nominal == obj.performance.mass.nominal);
  match = match && (this->performance.mass.high == obj.performance.mass.high);


  for (auto ix = 0; match && (ix < FL_NMAX); ix++)
  {
    match = match && ((isnan(this->performance.cruise[ix].FL) && isnan(obj.performance.cruise[ix].FL)) ||
		      (this->performance.cruise[ix].FL == obj.performance.cruise[ix].FL));
    match = match && ((isnan(Units::MetersPerSecondSpeed(this->performance.cruise[ix].TAS).value()) &&
		       isnan(Units::MetersPerSecondSpeed(obj.performance.cruise[ix].TAS).value())) ||
		      (this->performance.cruise[ix].TAS == obj.performance.cruise[ix].TAS));

    match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(this->performance.cruise[ix].fuel.lo).value()) &&
		       isnan(Units::KilogramsPerHourMassFlowRate(obj.performance.cruise[ix].fuel.lo).value())) ||
		      (this->performance.cruise[ix].fuel.lo == obj.performance.cruise[ix].fuel.lo));
    match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(this->performance.cruise[ix].fuel.nom).value()) &&
		       isnan(Units::KilogramsPerHourMassFlowRate(obj.performance.cruise[ix].fuel.nom).value())) ||
		      (this->performance.cruise[ix].fuel.nom == obj.performance.cruise[ix].fuel.nom));
    match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(this->performance.cruise[ix].fuel.hi).value()) &&
		       isnan(Units::KilogramsPerHourMassFlowRate(obj.performance.cruise[ix].fuel.hi).value())) ||
		      (this->performance.cruise[ix].fuel.hi == obj.performance.cruise[ix].fuel.hi));


    match = match && ((isnan(this->performance.climb[ix].FL) && isnan(obj.performance.climb[ix].FL)) ||
		      (this->performance.climb[ix].FL == obj.performance.climb[ix].FL));
    match = match && ((isnan(Units::MetersPerSecondSpeed(this->performance.climb[ix].TAS).value()) &&
		       isnan(Units::MetersPerSecondSpeed(obj.performance.climb[ix].TAS).value())) ||
		      (this->performance.climb[ix].TAS == obj.performance.climb[ix].TAS));

    match = match && ((isnan(Units::MetersPerSecondSpeed(this->performance.climb[ix].ROCD.lo).value()) &&
		       isnan(Units::MetersPerSecondSpeed(obj.performance.climb[ix].ROCD.lo).value())) ||
		      (this->performance.climb[ix].ROCD.lo == obj.performance.climb[ix].ROCD.lo));
    match = match && ((isnan(Units::MetersPerSecondSpeed(this->performance.climb[ix].ROCD.nom).value()) &&
		       isnan(Units::MetersPerSecondSpeed(obj.performance.climb[ix].ROCD.nom).value())) ||
		      (this->performance.climb[ix].ROCD.nom == obj.performance.climb[ix].ROCD.nom));
    match = match && ((isnan(Units::MetersPerSecondSpeed(this->performance.climb[ix].ROCD.hi).value()) &&
		       isnan(Units::MetersPerSecondSpeed(obj.performance.climb[ix].ROCD.hi).value())) ||
		      (this->performance.climb[ix].ROCD.hi == obj.performance.climb[ix].ROCD.hi));

    match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(this->performance.climb[ix].fuel).value()) &&
		       isnan(Units::KilogramsPerHourMassFlowRate(obj.performance.climb[ix].fuel).value())) ||
		      (this->performance.climb[ix].fuel == obj.performance.climb[ix].fuel));


    match = match && ((isnan(this->performance.descent[ix].FL) && isnan(obj.performance.descent[ix].FL)) ||
		      (this->performance.descent[ix].FL == obj.performance.descent[ix].FL));

    match = match && ((isnan(Units::MetersPerSecondSpeed(this->performance.descent[ix].TAS).value()) &&
		       isnan(Units::MetersPerSecondSpeed(obj.performance.descent[ix].TAS).value())) ||
		      (this->performance.descent[ix].TAS == obj.performance.descent[ix].TAS));

    match = match && ((isnan(Units::MetersPerSecondSpeed(this->performance.descent[ix].ROCD).value()) &&
		       isnan(Units::MetersPerSecondSpeed(obj.performance.descent[ix].ROCD).value())) ||
		      (this->performance.descent[ix].ROCD == obj.performance.descent[ix].ROCD));

    match = match && ((isnan(Units::KilogramsPerHourMassFlowRate(this->performance.descent[ix].fuel).value()) &&
		       isnan(Units::KilogramsPerHourMassFlowRate(obj.performance.descent[ix].fuel).value())) ||
		      (this->performance.descent[ix].fuel == obj.performance.descent[ix].fuel));

  }

  return match;
}
