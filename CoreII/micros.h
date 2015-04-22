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


#ifndef SQR
#define SQR(A)   		((A)*(A))
#endif

#ifndef MAX
#define MAX(A, B)		(((A) > (B)) ? (A) : (B))
#endif

#ifndef MIN
#define MIN(A, B)		(((A) < (B)) ? (A) : (B))
#endif


#ifndef ABS
#define ABS(A)			(((A) > (0)) ? (A) : (-(A)))
#endif

#ifndef NORM
#define NORM(DX, DY)      sqrt(SQR(DX) + SQR(DY))
#endif

#ifndef LIMIT
#define	LIMIT(x,xmin,xmax) ( x < xmin ? xmin : (x > xmax ? xmax : x) )
#endif

#ifndef SIGN
#define SIGN(A)		(((A) == (0)) ?	0: (((A) > (0)) ? (1) : (-1)) )
#endif
