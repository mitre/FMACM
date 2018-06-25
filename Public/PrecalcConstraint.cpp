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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/PrecalcConstraint.h"

bool operator<=(ActiveFlagType l, ActiveFlagType r) {
    return ((static_cast<int> (l)) <= (static_cast<int> (r)));
}

PrecalcConstraint::PrecalcConstraint(void)
{

  active_flag = ActiveFlagType::UNSET;
  constraint_dist = 0.0; // distance constraints-meters.
  constraint_altHi = 0.0; // altitude max constraints-meters.
  constraint_altLow = 0.0; // altitude min constraints-meters.
  constraint_speedHi = 0.0; // speed max constraint-meters per second.
  constraint_speedLow = 0.0; // speed min constraint-meters per second.
  index = -1;
  violation_flag = false;

}


PrecalcConstraint::~PrecalcConstraint(void)
{
}


PrecalcConstraint& PrecalcConstraint::operator=(const PrecalcConstraint &obj)
{

	// Generic = operator.
	//
	// obj:VerticalPredictor object to set with.
	// returns this object with new values.

	if (this != &obj)
	{

		this->constraint_dist = obj.constraint_dist;
		this->constraint_altHi = obj.constraint_altHi;
		this->constraint_altLow = obj.constraint_altLow;
		this->constraint_speedHi = obj.constraint_speedHi;
		this->constraint_speedLow = obj.constraint_speedLow;
		this->index = obj.index;
		this->active_flag = obj.active_flag;
    this->violation_flag = obj.violation_flag;

	}

	return *this;

}


bool PrecalcConstraint::operator<(const PrecalcConstraint &obj) const
{

  bool result = false;

  if (this->constraint_dist < obj.constraint_dist)
  {
    result = true;
  }

  return result;

}


bool PrecalcConstraint::operator==(const PrecalcConstraint &obj) const
{

	// Generic equals operator.
	//
	// obj:comparison object.
	// returns true if obj matches.
	//         false if object doesn't match.


  bool match = (this->constraint_dist == obj.constraint_dist);

  match = match && (this->constraint_altHi == obj.constraint_altHi);
  match = match && (this->constraint_altLow == obj.constraint_altLow);

  match = match && (this->constraint_speedHi == obj.constraint_speedHi);
  match = match && (this->constraint_speedLow == obj.constraint_speedLow);

  match = match && (this->index == obj.index);

  match = match && (this->active_flag == obj.active_flag);
  match = match && (this->violation_flag == obj.violation_flag);

  return match;

}


bool PrecalcConstraint::operator!=(const PrecalcConstraint &obj) const
{

  // Generic not equals operator.
  //
  // obj:comparison object.
  // returns true if obj doesn't match.
  //         false if obj matches.

  return !this->operator==(obj);

}

