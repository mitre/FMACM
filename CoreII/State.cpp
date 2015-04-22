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

#include "State.h"
#include <cstdio>
#include <cmath>
#include <iostream>
#include "constants.h"

State::State(void)
{
  //ADS_B_ether.clear();
}

State::State(FILE *f) {
    outfile = f;
}

State::~State(void)
{
}

State::State(const State &in)
{
  aircraft_truth_state_vector_list = in.aircraft_truth_state_vector_list;
  outfile = in.outfile;
  test_string = in.test_string;
}

State& State::operator=(const State &in)
{
  if (this != &in) {
    aircraft_truth_state_vector_list = in.aircraft_truth_state_vector_list;
    outfile = in.outfile;
    test_string = in.test_string;
  }
  return *this;
}

void State::push_back(const AircraftState &aircraftState) {
    aircraft_truth_state_vector_list.push_back(aircraftState);

    if (!this->outfile) {
        std::cout << "WARNING: unable to write states file. No file provided." << std::endl;
        return;
    }

    double v = hypot(aircraftState.xd * FT_M - aircraftState.Vwx,
    			aircraftState.yd * FT_M - aircraftState.Vwy)
    	    / cos(aircraftState.gamma);	// m/s

    fprintf(this->outfile, "%.0f,%.5f,%.3f,%.3f,%.5f,%.5f,%.5f\n",
            aircraftState.time,
	    aircraftState.distToGo,
	    v,
	    aircraftState.zd * FT_M,
            aircraftState.x * FT_M,
            aircraftState.y * FT_M,
            aircraftState.z * FT_M
    );
    fflush(this->outfile);
}
