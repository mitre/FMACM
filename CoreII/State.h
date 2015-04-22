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

#pragma once

#include <string>
#include <stdio.h>
#include "list"
#include "AircraftState.h"

class State

{
public:
  State(void);
  State(FILE *f);
  ~State(void);
  State(const State &in);
  State& operator=(const State &in);
  void push_back(const AircraftState &acState);
  void clear() { fclose(outfile); outfile = 0;}
  
  //Data:
  // ADSBEther  ADS_B_ether;
private:
  std::list<AircraftState> aircraft_truth_state_vector_list;
  
  std::string test_string;
  FILE *outfile;
};
