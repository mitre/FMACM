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

#include "math/DMatrix.h"
#include "math/InvalidIndexException.h"
#include <algorithm>

char *DMatrix::MULTIPLICATION_DIMENSIONS_MESSAGE = (char *) "Cannot multiply DMatrix unless inner dimensions match.";

// default Constructor/Destructors
DMatrix::DMatrix(void)
{
	minRow = 0;
	maxRow = -1;
	rows = NULL;
}
DMatrix::~DMatrix(void)
{
	delete [] rows;
	rows = NULL;
}
// Copy Constructor
DMatrix::DMatrix(const DMatrix &in)
{
	// calculates size of Matrix
	int size = in.maxRow - in.minRow + 1;

	// sets row min/max
	maxRow = in.maxRow;
	minRow = in.minRow;

//	delete [] rows;

	// allocates the new DMatrix (array of DVectors)
	rows = new DVector[size];

	// loop to copy the values of the given DMatrix
	for(int loop = 0; loop < size; loop++)
	{
		rows[loop] = in.rows[loop];
	}
}

// TODO Complete append operation
//void DMatrix::append(DVector& newRow) {
//
//	// copy rows
//	DVector cpyrows = *rows;
//
//	// update bounds
//	this->setBounds(minRow,maxRow+1,get_min_colomn(),get_max_colomn());
//
//	// loop to copy the values back to rows
//	int size = maxRow - minRow + 1;
//	for(int loop = 0; loop < size-1; loop++)
//	{
//		printf("cpyrows:%g\n",rows[loop]);
////		rows[loop] = cpyrows[loop];
//	}
//
//	// add new row as last member
////	rows[size] = newRow;
//}

// Constructor that takes row and colomn min/max values
DMatrix::DMatrix(int inRowMin, int inRowMax, int inColMin, int inColMax)
{
	// calculates the size of the DMatrix
	int size = inRowMax - inRowMin + 1;

	// sets row min/max
	maxRow = inRowMax;
	minRow = inRowMin;

	// allocates the new DMatrix (array of DVectors)
	rows = new DVector[size];

	// loop to set the colomn size (DVector min/max)
	for(int loop = 0; loop < size; loop++)
	{
		rows[loop].setBounds(inColMin, inColMax);
	}
}

// Constructor that takes a 2D array
DMatrix::DMatrix(double **array_in, int inRowMin, int inRowMax, int inColMin, int inColMax)
{
	// calculates the size of the DMatrix
	int size = inRowMax - inRowMin + 1;

	// sets row min/max
	maxRow = inRowMax;
	minRow = inRowMin;

	// allocates the new DMatrix (array of DVectors)
	rows = new DVector[size];

	// loop to set the colomn size (DVector min/max)
	for(int loop = 0; loop < size; loop++)
	{
		rows[loop].setBounds(inColMin, inColMax);
	}

	// get row and colomn sizes
	//int column_size = inColMax-inColMin;
	//int row_size = inRowMax-inRowMin;

	// NOTE this is dangerous code, the array MUST be the same size as the given row/colomn information
	// failure to match the bounds given will cause dangerous memory access!!!
	// this only works because 2d arrays are stored in sequential memory addresses
	// if the array is dynamically allocated this WILL cause dangerous memory access!!!
	double *memory_pointer; // pointer to point to the memory address of the 2d array
	memory_pointer = (double *)array_in; // accesses the the memory address of the first element
	for(int outer = 0; outer < size; outer++)
	{
		for(int inner = inColMin; inner <= inColMax; inner++)
		{
			rows[outer].set(inner,(*memory_pointer));
			memory_pointer++; // iterates the memory address
		}
	}
}
/*
This was useful for migration, but we don't need it any more.

DMatrix::DMatrix(const WindStack& windStack) {
	minRow = 0;
	maxRow = -1;
	rows = NULL;
	load(windStack);
}

void DMatrix::load(const WindStack& windStack) {
	setBounds(windStack.get_min_row(), windStack.get_max_row(), 1, 2);
	for (int i = minRow; i <= maxRow; i++) {
		set(i, 1, windStack.getAltitude(i).value());
		set(i, 2, windStack.getSpeed(i).value());
	}
}
*/


// get/set methods
double DMatrix::get(int row, int colomn) const
{
	// check if in valid range of DMatrix
	if( inRange(row))
	{
		// uses the DVectors overloaded array operator to get the value
		return rows[row - minRow][colomn]; 
	}

	// if not in valid range throw Invalid Index Exception
	throw InvalidIndexException(row, minRow, maxRow);
}
void DMatrix::set(int row, int colomn, double value)
{
	// check if in valid range of DMatrix
	if( inRange(row))
	{
		// uses the DVector overloaded array operator to set the value
		rows[row-minRow][colomn] = value;
	}
	else
	{
		// else not in range throw Invalid Index Exception
		throw InvalidIndexException(row, minRow, maxRow);
	}
}

// method to set the bounds of the Matrix
void DMatrix::setBounds(int inRowMin, int inRowMax, int inColMin, int inColMax)
{
	// calculates the size of the DMatrix
	int size = inRowMax - inRowMin + 1;

	// sets row min/max
	maxRow = inRowMax;
	minRow = inRowMin;

	// allocates the new DMatrix (array of DVectors)
	delete[] rows;
	rows = new DVector[size];

	// loop to set the colomn size (DVector min/max)
	for(int loop = 0; loop < size; loop++)
	{
		rows[loop].setBounds(inColMin, inColMax);
	}
}

// method to check if a given index is in the array
bool DMatrix::inRange(int row, int colomn) const
{
	// initializes results to false
	bool results = false;

	// check if row is in the range of [min,max] inclusive
	if( row >= minRow && row <= maxRow )
	{
		// if in rows range call the DVector of that row and check range
		results = rows[row].inRange(colomn);
	}

	return results;
}

// method to check if a given index is in the array
bool DMatrix::inRange(int row) const
{
	// initializes results to false
	bool results = false;

	// check if row is in the range of [min,max] inclusive
	if( row >= minRow && row <= maxRow )
	{
		// if in rows range then results is true
		results = true;
	}

	return results;
}

// overloads the array index operator
DVector& DMatrix::operator[](int row)
{
	// check if in valid range of DMatrix
	if( inRange(row))
	{
		// returns the DVector of the given row
		return rows[row - minRow]; 
	}

	// if not in valid range throw Invalid Index Exception
	throw InvalidIndexException(row, minRow, maxRow);
}
const DVector& DMatrix::operator[](int row) const
{
	// check if in valid range of DMatrix
	if( inRange(row))
	{
		// returns the DVector of the given row
		return rows[row - minRow]; 
	}

	// if not in valid range throw Invalid Index Exception
	throw InvalidIndexException(row, minRow, maxRow);
}

// overloads the equals operator
DMatrix& DMatrix::operator=(const DMatrix &in)
{
  if (this != &in) {
    // calculates size of Matrix
    int size = in.maxRow - in.minRow + 1;
    
    // sets row min/max
    maxRow = in.maxRow;
    minRow = in.minRow;
    
    delete [] rows;
    
    // allocates the new DMatrix (array of DVectors)
    rows = new DVector[size];
    
    // loop to copy the values of the given DMatrix
    for(int loop = 0; loop < size; loop++) {
      rows[loop] = in.rows[loop];
    }
  }

  return *this;
}

/**
 * Matrix multiplication
 */
DMatrix& DMatrix::operator*(const DMatrix &that) const {
	int rowStart1 = get_min_row();
	int rowEnd1 = get_max_row();
	int colStart1 = get_min_colomn();
	int colEnd1 = get_max_colomn();
	int rowStart2 = that.get_min_row();
	int rowEnd2 = that.get_max_row();
	int colStart2 = that.get_min_colomn();
	int colEnd2 = that.get_max_colomn();
	if (colStart1 != rowStart2 || colEnd1 != rowEnd2) {
		// cannot be multiplied because inner dimensions don't match
		throw IncompatibleDimensionsException(
				MULTIPLICATION_DIMENSIONS_MESSAGE);
/*		DMatrix *result = 0;
		return *result;*/
	}
	DMatrix *result = new DMatrix(rowStart1, rowEnd1, colStart2, colEnd2);
	for (int i = rowStart1; i <= rowEnd1; i++) {
		for (int j = colStart2; j <= colEnd2; j++) {
			double x = 0;
			for (int k = colStart1; k <= colEnd1; k++) {
				x += (*this)[i][k] * that[k][j];
			}
			result->set(i, j, x);
		}
	}
	return *result;
}

void DMatrix::ascendSort() {
	//std::sort(&rows[0],&rows[maxRow-minRow+1],&DMatrix::rowsComparator);
	std::sort(&rows[0],&rows[maxRow-minRow+1]);
}

// getters
int DMatrix::get_min_row() const
{
	return minRow;
}
int DMatrix::get_max_row() const
{
	return maxRow;
}
int DMatrix::get_min_colomn() const
{
	return rows[0].get_min();
}


int DMatrix::get_max_colomn() const
{
	return rows[0].get_max();
}

DMatrix::IncompatibleDimensionsException::IncompatibleDimensionsException(char *explanation) :
		exception(),
		explanation(explanation)
{
}

DMatrix::IncompatibleDimensionsException::~IncompatibleDimensionsException() throw() { }

const char *DMatrix::IncompatibleDimensionsException::what() const throw() {
	return explanation;
}
