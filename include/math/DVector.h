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

#pragma once

#ifndef DVECTOR_H
#define DVECTOR_H

class DVector
{
private:
	int minIndex;
	int maxIndex;
	double *vector;

public:
	// constructor/destructors
	DVector(void);
	DVector(int min, int max);
	DVector(const DVector& in);
	~DVector(void);

	// get/set methods
	double get(int index);
	void set(int index, double value);

	// method to set the bouds of the Vector
	void setBounds(int min, int max);

	// method to check if a given index is in the array
	bool inRange(int index) const;

	// getter methods
	int get_min();
	int get_max();

	// overloads the array index operator
	double& operator[](int);
	const double& operator[](int) const;

	// overloads the equals operator
	DVector& operator=(const DVector &in);
	
	// overloads less than opertor for sort
	bool operator<(const DVector &other) const;
};

#endif
