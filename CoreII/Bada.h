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
// Copyright 2015 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

/* bada_class.h		Initial code 10/09/2009 
    REVISION HISTORY:
      DATE              INITIALS                REASON

	10/09/2009            jws                   create

*/

#pragma once
#include <stdio.h>
#include <string>

enum ENGINE_TYPE { JET, TURBOPROP, PISTON };
enum WAKE_CATEGORY { HEAVY_, MEDIUM_, LIGHT_ };
#define FL_NMAX 40

typedef struct {
  int FL;
  int TAS;
  struct{
    int lo;
    int nom;
    int hi;
  } ROCD;
  double fuel;
} climb_struct;

typedef struct {
  int FL;
  int TAS;
  struct{
    double lo;
    double nom;
    double hi;
  } fuel;
} cruise_struct;

typedef struct {
  int FL;
  int TAS;
  int ROCD;
  double fuel;
} descent_struct;

/*
 * Reduced BADA data for test framework:
 *
 * in the limited BADA data and should not be used in the
 * test framework.
 *
 * They have been left as placeholders so that organizations who
 * license the full dataset have the option to use their full
 * loading methods.
 */

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
		double m_ref;   // reference mass (tonnes)
		double m_min;   // minimum mass (tonnes)
		double m_max;   // maximum mass (tonnes)
	} mass;

	// Flight Envelope
	struct
	{
		double V_mo;    // maximum operating speed (knots CAS)
		double M_mo;    // maximum operating Mach number
	} flight_envelope;

	// Aerodynamics
	struct
	{
          double S;       // Reference wing surface area (m2)
          
          struct
          {
            double V_stall;  // knots (CAS)
            double cd0;      // parasitic drag coeff.
            double cd2;      // induced drag coeff.
          } cruise;

          struct
          {
            double V_stall;  // knots (CAS)
            double cd0;      // parasitic drag coeff.
            double cd2;      // induced drag coeff.
          } approach;

          struct
          {
            double V_stall;  // knots (CAS)
            double cd0;      // parasitic drag coeff.
            double cd2;      // induced drag coeff.
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
            double CT_c1;  // 1st max. climb thrust coeff. (N, jet/piston)
                           //                              (kt-N, turboprop)
            double CT_c2;  // 2nd max. climb thrust coeff. (feet)
            double CT_c3;  // 3rd max. climb thrust coeff. (1/feet^2, jet)
                           //                              (N, turboprop)
	                   //                              (kt-N, piston)
          } max_climb;
          struct
          {
            double CT_low;    // low altitude descent thrust coeff.
            double CT_high;   // high altitude descent thrust coeff.
            double h;         // Transition altitude (feet)
            double CT_app;    // approach thrust coeff.
            double CT_ld;     // landing thrust coeff.
          } descent;
	} engine_thrust;

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
            
	void init( char *type_code );
	/**
	 * Reads the .OPF file, which contains all the thrust,
	 * drag and fuel coefficients to be used in TEM together
	 * with information on weights, speeds, maximum altitude, etc.
	 */
	void read_OPF();
	/**
	 * Reads the .APF file, which contains a default operational
	 * climb, cruise and descent speed schedule that is likely
	 * to be used by an airline.
	 */
	void read_APF();
	/**
	 * Reads the .PTF file, which contains a table of performance
	 * data.
	 */
	void read_PTF();
	/**
	 * Writes out any relevant data about the Bada object
	 * for diagnostic purposes.
	 */
	void dump();

    Bada();

    /**
     * Maps the aircraft type to an 8-character file name (no extension).
     * Multiple type codes may map to the same value.
     */
    std::string get_file( char *type_code /** aircraft type abbreviation, e.g. JET1 */);
};
