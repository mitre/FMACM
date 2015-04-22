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

#include "HorizontalTraj.h"
#include <sstream>



HorizontalTraj::HorizontalTraj(void)
{

  this->x = 0.0; // in meters
  this->y = 0.0; // in meters
  this->segment = "";
  this->L = 0.0; // leg length in meters
  this->course = 0.0;

}


HorizontalTraj::HorizontalTraj(const double x, const double y, const double L,
			       const std::string segment, const double course,
			       const TurnTraj &turns)
{

  this->x = x;
  this->y = y;
  this->L = L;
  this->segment = segment;
  this->course = course;

  this->turns = turns;

}


HorizontalTraj::~HorizontalTraj(void)
{
}


std::string HorizontalTraj::toStr(void)
{

  std::ostringstream strm;

  strm << this->x << " " << this->y << " " << this->L << " " << this->L << " "
       << this->segment.c_str() << " " << course << " " << this->turns.toStr().c_str();

  return strm.str();

}


bool HorizontalTraj::operator==(const HorizontalTraj &ht) const
{

  return ((this->x == ht.x) && (this->y == ht.y) &&
	  (this->L == ht.L) && (this->segment == ht.segment) &&
	  (this->course == ht.course) && (this->turns == ht.turns));

}


bool HorizontalTraj::operator!=(const HorizontalTraj &ht) const
{

  return !(*this == ht);

}
