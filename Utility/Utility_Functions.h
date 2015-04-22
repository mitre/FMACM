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

#ifndef MAX 
#define MAX(X, Y) X > Y ? X : Y
#endif 

#ifndef MIN 
#define MIN(X, Y) X < Y ? X : Y
#endif 

#define DELETE_IF_DEF(X)       if(X != NULL) { delete   X;}
#define DELETE_ARRAY_IF_DEF(X) if(X != NULL) { delete[] X;}

// code taken from some other porgram 
// should turn these into macros possabaly 

template <class X>
inline void SWAP(X &a, X &b)
{
	X temp = a;
	a = b;
	b = temp;
}

template <class X>
inline X QUANTIZE(X x, X q) 
{
	x = (x + q/2) / q;
	x = x * q;
	return x;
}

template <class X>
inline void ORDER(X &a, X &b)
{
	if(a < b) 
		return;
	else 
	{
		X temp;
		temp = a;
		a = b;
		b = temp;
	}
}

template <class X>
bool array_equal(X a, X b, int len)
{
	int i;

	for(i = 0; i < len; i++)
	{
		if(a[i] != b[i])
			return false;
	}
}
