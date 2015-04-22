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

#include "TurnTraj.h"
#include <sstream>


TurnTraj::TurnTraj(void)
{
  this->x_turn = 0;
  this->y_turn = 0;
  this->q_start = 0;
  this->q_end = 0;
  this->radius = 0;
}


TurnTraj::TurnTraj(const double x_turn, const double y_turn, const double q_start,
		   const double q_end, const double radius)
{

  this->x_turn = x_turn;
  this->y_turn = y_turn;
  this->q_start = q_start;
  this->q_end = q_end;
  this->radius = radius;

}


TurnTraj::~TurnTraj(void)
{
}


std::string TurnTraj::toStr(void)
{

  std::ostringstream strm;

  strm << this->x_turn << " " << this->y_turn << " " << this->q_start << " " << this->q_end << " " << this->radius;

  return strm.str();

}


bool TurnTraj::operator==(const TurnTraj &tt) const
{

  return ((this->x_turn == tt.x_turn) &&
	  (this->y_turn == tt.y_turn) &&
	  (this->q_start == tt.q_start) &&
	  (this->q_end == tt.q_end) &&
	  (this->radius == tt.radius));

}
