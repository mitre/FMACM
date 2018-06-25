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

#include "math/DVector.h"
#include <exception>

class DMatrix
{
private:
	static char *MULTIPLICATION_DIMENSIONS_MESSAGE;
	DVector *rows; 
	int minRow;
	int maxRow;

public:
	class IncompatibleDimensionsException : public std::exception
	{
	public:
		IncompatibleDimensionsException(char *explanation);
		virtual ~IncompatibleDimensionsException() throw();
		virtual const char* what() const throw();
	private:
		const char *explanation;
	};

	// default Constructor/Destructors
	DMatrix(void);
	~DMatrix(void);
	// Copy Constructor
	DMatrix(const DMatrix &in); 
	DMatrix(double **array_in, int inRowMin, int inRowMax, int inColMin, int inColMax);

	// Constructor that takes row and colomn min/max values
	DMatrix(int inRowMin, int inRowMax, int inColMin, int inColMax);

	//DMatrix(const WindStack &windStack);
	//void load(const WindStack &windStack);

	// get/set methods
	double get(const int row, const int colomn) const;
	void set(const int row, const int colomn, const double value);

	void ascendSort();

	// method to set the bounds of the Matrix
	void setBounds(int inRowMin, int inRowMax, int inColMin, int inColMax);

	// method to check if a given index is in the array
	bool inRange(const int row, const int colomn) const;
	bool inRange(const int row) const;

	// overloads the array index operator
	DVector& operator[](const int);
	const DVector& operator[](const int) const;

	// overloads the equals operator
	DMatrix& operator=(const DMatrix &in);

	DMatrix& operator*(const DMatrix &that) const;

	/**
	 * Appends a new row to the matrix.
	 * @param newRow
	 */
//	void append(DVector& newRow); // TODO Complete append operation

	// getters
	int get_min_row() const;
	int get_max_row() const;
	int get_min_colomn() const;
	int get_max_colomn() const;
};
