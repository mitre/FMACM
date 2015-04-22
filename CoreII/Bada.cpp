// ****************************************************************************
//  Copyright © 2015 The MITRE Corporation. All Rights Reserved.  
// ****************************************************************************

#include "Bada.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

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
void Bada::read_OPF() { }

/**
 * Reads the .APF file, which contains a default operational
 * climb, cruise and descent speed schedule that is likely
 * to be used by an airline.
 */
void Bada::read_APF() { }

/**
 * Reads the .PTF file, which contains a table of performance
 * data.
 */
void Bada::read_PTF() { }

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
   mass.m_ref = -999;
}
