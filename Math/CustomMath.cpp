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

/* CustomMath.cpp		Initial code from Survsim 2.00R1  11/2/99*/


#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "CustomMath.h"
#include "constants.h"
#include "micros.h"


//uniform

#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836

//generate a uniform random number between 0 and 1
//From "Numerical Recipe"

double uniform(double&  seed) //seed should not be 0.0
{
    double ans;
 
    long k = (long) (seed/(double)IQ);
    seed = (double)IA*(seed - (double)(k*IQ)) - (double)(IR*k);
    if(seed <0) seed += (double) IM;
    ans = (double) AM * seed;
    return ans;
}


double gauss(double mean, double sigma, double&  seed)
{

	// returns gaussian distributed random variable with mean mean and standard
	// deviation sigma.
	// usage example:  x = gauss(0., 32.);

	double u1, u2, eln, ang, v1;
	
	u1 = uniform(seed);
	u2 = uniform(seed);	                                                                                            
 	eln = -2.0 * log(u1);     // ALOG(U1)                                               
	ang = 2.0 * M_PI * u2;                                                                                                               
	v1  = sqrt(eln) * cos(ang);
	v1 = mean + sigma * v1;
	 
 	return(v1);
	
} // gauss


double trunc_gauss(double mean, double sigma, double max_std_dev, double&  seed)
{

	// returns truncated gaussian random varialbe with mean mean, standard
	// deviation sigma, and maximum standard deviation max_std_dev
	double val;
	
	val = mean + sigma * max_std_dev + 1.;
	//gwang 09/05/2002
	//while (val > (mean + sigma * max_std_dev) )
	while (val > (mean + sigma * max_std_dev)  ||  val < (mean - sigma * max_std_dev))
	//end gwang 09/05/2002
		val = gauss(mean, sigma, seed);
	
	return(val);
	
} // trunc_gauss


Units::Time Rayleigh(Units::Time mean, Units::Time sigma, double&  seed)
{

       // returns Rayleigh distributed random variable with mean mean and standard
       // deviation sigma.
       // usage example:  x = Rayleigh(0., 32.);

       double u1, v1;

       u1 = uniform(seed);
       v1 = (sqrt(-2.0*log(u1))-1.253)/sqrt(0.429);
       //v1 = mean + sigma * v1;

       return(mean + sigma * v1);

} // Rayleigh


double atan3(double x, double y)
{
 
    // returns arc tangent as an angle measured from north in the range 0, 2pi  

   	double temp;
   
   	temp =  (double) atan2( x,  y);
   
   	if (temp < 0.0) 
   		temp = temp + 2.0 * M_PI;

   	return (temp);

} // atan3

  
double laplace(double lambda, double&  seed)
{

	// returns laplacian r.v. with parameter lambda.
 	
	double uni, err;
 	
	uni = uniform(seed);
	err = -lambda*log(uni);
	uni = uniform(seed);

	if (uni < 0.5) err = -err;
		
	return(err);

} // laplace
  

double subtract_headings(double hd1, double hd2)
{
	// subtract heading 2 from heading 1 with the following convention:
	// negative (counterclockwise) deltas are indicated by being greater than pi.
	// positive (clockwise) deltas are less than pi.

	double t;

	t = hd1 - hd2;

	if (t < 0.) t = TWOPI + t;

	return (t);

} // subtract_headings


//-------------------------------------------------------------
// Speed conversion using MACH & altitude as inputs; unit of output is FPS
//-------------------------------------------------------------
double MachToTas(double mach, double altitude) {

    float speedOfSound;
    double tas;
	

    if (0 <= altitude && altitude <= 36000) {
            speedOfSound = 662.4 - 243.0 *altitude/ 100000.0; }
    else if (36000 < altitude && altitude <= 82000) {
            speedOfSound = 573.8; }
    else if (82000 < altitude && altitude <= 99900) {
            speedOfSound = 120 * altitude / 100000. + 475.4; }
    else {
    	printf("Unexpected altitude in MachToTas:  %f\n", altitude);
    	exit(1);
    }
           
    tas = (mach * speedOfSound);

	//before this point tas is in knots
	//gwang 2009-03
	tas *= KT2FPS; 
	//end gwang

  return(tas); //FPS
} 


//output CAS in FPS
double MachToCas_MITRE(double mach, double alt)
{
  double cas, thetas, deltam;

  if (alt < 36089.24)
  {
    thetas = (1.0 - 6.8755856E-6 * alt);
    deltam = pow(thetas,5.2558797);
  }
  else
  {
    thetas = 0.7519;
    deltam = 0.2233609 * pow(2.718, (-((alt-36089.24)/20806.0)) );
  }
  cas = 661.4786 * sqrt(5.0 * ((pow( (1.0 + deltam *
        ((pow( (1.0 + 0.2 * mach * mach),3.5 ) - 1.0))),
        (2.0/7.0) ))-1.0));

  cas *= KT2FPS;

  return(cas);

} /* MachToCas_MITRE */

//inverse = inverse(in)
/* Gauss-Jordan elimination from Numerical recipe:*/
bool inverse( DMatrix &in, int n, DMatrix &out)
{
	int irow, icol;

	DVector indxc(1, n);
	DVector indxr(1, n);
	DVector ipiv(1, n);
	DMatrix a(1, n, 1, n);

	//copy the "in" matrix into the "a" matrix:
	int in_min_row = in.get_min_row();
	int in_min_column = in.get_min_colomn();
	for(int i=1; i<=n; i++)
	{
		for(int j=1; j<=n; j++)
		{
			a.set(i,j, in.get(i-1+in_min_row, j-1+in_min_column));
		}
	}



	for(int j=1; j<= n; j++)
	{
		ipiv.set(j, 0.);
	}

	for(int i=1; i<=n; i++)
	{
		double big = 0.0;
		for(int j=1; j<=n; j++)
		{
			if(ipiv.get(j) != 1.)
			{
				for(int k=1; k<=n; k++)
				{
					if(ipiv.get(k) == 0.0)
					{
						if( fabs(a.get(j, k)) >= big)
						{
							big = fabs(a.get(j, k));
							irow = j;
							icol = k;
						}
					}
					else if (ipiv.get(k) > 1.)
					{
						//singular matrix
						printf("\nWarning: Inversion of a singular matrix in the inverse() function (> 1 val).\n");
						return false;
					}
				} //end for(int k=1; k<=n; k++)
			} //end if(ipiv.get(j) != 1.)
		} //end for(int j=1; i<=n; j++)
		ipiv.set(icol, ipiv.get(icol)+1);
		if(irow != icol)
		{
			//swap
			for(int l=1; l<=n; l++) 
			{
				double temp_swap;
				temp_swap = a.get(irow, l);
				a.set(irow, l, a.get(icol, l));
				a.set(icol, l, temp_swap);
			} //end for(int l=1; l<=n; l++) 
		} //end if(irow != icol)
		indxr.set(i, (double)irow);
		indxc.set(i, (double)icol);
		if(a.get(icol, icol) == 0.0)
		{
			//singular matrix
			printf("\nWarning: Inversion of a singular matrix in the inverse() function (0 val).\n");
			return false;
		}
		double pivinv = 1.0/a.get(icol, icol);
		a.set(icol, icol, 1.);
		for(int l=1; l<=n; l++) 
		{
			a.set(icol, l, pivinv * a.get(icol, l));
		}//end for(int l=1; l<=n; l++) 

		for(int ll=1; ll<=n; ll++)
		{
			if(ll != icol)
			{
				double dum = a.get(ll, icol);
				a.set(ll, icol, 0.);
				for(int l=1; l<=n; l++)
				{
					a.set(ll, l, a.get(ll, l) - dum*a.get(icol, l));
				} //end for(int l=1; l<=n; l++)
			} //end if(ll != icol)
		}//end for(int ll=1; ll<=n; ll++)
	}//end for(int i=1; i<=n; i++)



	for(int l=n; l>=1; l--)
	{
		if(indxr.get(l) != indxc.get(l))
		{
			for(int k=1; k<=n; k++)
			{
				//swap:
				double temp;
				temp = a.get(k, (int)indxr.get(l));
				a.set(k, (int)indxr.get(l), a.get(k, (int)indxc.get(l)));
				a.set(k, (int)indxc.get(l), temp);
			}
		}//end if(indxr.get(l) != indxc.get(l))
	}//end for(int l=n; l>=1; l--)



	//copy the "a" matrix into the "out" matrix:
	int out_min_row = out.get_min_row();
	int out_min_column = out.get_min_colomn();
	for(int i=1; i<=n; i++)
	{
		for(int j=1; j<=n; j++)
		{
			out.set(i-1+out_min_row, j-1+out_min_column, a.get(i,j));
		}
	}
	return true;
}


void matrix_times_vector( DMatrix &matrix_in, DVector &vector_in, int n, DVector &vector_out)
{


	for(int i = 0; i <n; i++ )
	{
		int ii = i + vector_out.get_min();
		vector_out[ii] = 0.0;
		for(int j = 0; j <n; j++ )
		{
			vector_out[ii] += matrix_in[i + matrix_in.get_min_row()][j + matrix_in.get_min_colomn()] * vector_in[j + vector_in.get_min()];
		}
	}
}


#ifndef _LINUX_
int roundToInt(double d)
{
	// Rounds double to int, away from 0 for the midpoint values.
	//
	// d:double value to be rounded
	//
	// returns rounded integer value.

	double val = d;
	int i = 0;

	if (val > 0) {
		val = val + 0.5;
		i = floor(val);
	}
	else if (val < 0) {
		val = val - 0.5;
		i = ceil(val);
	}

	return i;
}
#endif
