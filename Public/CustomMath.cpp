// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001
// and is subject to Federal Aviation Administration Acquisition Management System
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE
// Corporation and do not necessarily reflect the views of the Federal Aviation
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include <math.h>
#include <stdlib.h>
#include <stdexcept>
#include "public/CustomMath.h"
#include "utility/UtilityConstants.h"

using namespace aaesim::open_source::constants;

// generate a uniform random number between 0 and 1
// From "Numerical Recipe"

double atan3(double x, double y) {

   // returns arc tangent as an angle measured from north in the range 0, 2pi

   double temp;

   temp = (double)atan2(x, y);

   if (temp < 0.0) {
      temp = temp + 2.0 * PI;
   }

   return (temp);

}  // atan3

double quantize(double value, double lsb) {
   // quantizes value to lsb (least significant bit)
   if (lsb == 0) return value;
   double r = round(value / lsb);
   if (r == -0) {
      r = 0;
   }
   return (lsb * r);
}  // quantize

Units::Length quantize(Units::Length value, Units::Length lsb) {
   // quantizes value to lsb (least significant bit)
   if (lsb == Units::zero()) return value;
   double r = round(value / lsb);
   if (r == -0) {
      r = 0;
   }
   return (lsb * r);
}

Units::Speed quantize(Units::Speed value, Units::Speed lsb) {
   // quantizes value to lsb (least significant bit)
   if (lsb == Units::zero()) return value;
   double r = round(value / lsb);
   if (r == -0) {
      r = 0;
   }
   return (lsb * r);
}

Units::Time quantize(Units::Time value, Units::Time lsb) {
   // quantizes value to lsb (least significant bit)
   if (lsb == Units::zero()) return value;
   double r = round(value / lsb);
   if (r == -0) {
      r = 0;
   }
   return (lsb * r);
}

double subtract_headings(double hd1, double hd2) {
   // subtract heading 2 from heading 1 with the following convention:
   // negative (counterclockwise) deltas are indicated by being greater than pi.
   // positive (clockwise) deltas are less than pi.

   double t;

   t = hd1 - hd2;

   if (t < 0.) {
      t = TWO_PI + t;
   }

   return (t);

}  // subtract_headings

// inverse = inverse(in)
/* Gauss-Jordan elimination from Numerical recipe:*/
bool inverse(DMatrix &in, int n, DMatrix &out) {
   int irow = -1, icol = -1;

   DVector indxc(1, n);
   DVector indxr(1, n);
   DVector ipiv(1, n);
   DMatrix a(1, n, 1, n);

   // copy the "in" matrix into the "a" matrix:
   int in_min_row = in.GetMinRow();
   int in_min_column = in.GetMinColumn();
   for (int i = 1; i <= n; i++) {
      for (int j = 1; j <= n; j++) {
         a.Set(i, j, in.Get(i - 1 + in_min_row, j - 1 + in_min_column));
      }
   }

   for (int j = 1; j <= n; j++) {
      ipiv.Set(j, 0.);
   }

   for (int i = 1; i <= n; i++) {
      double big = 0.0;
      for (int j = 1; j <= n; j++) {
         if (ipiv.Get(j) != 1.) {
            for (int k = 1; k <= n; k++) {
               if (ipiv.Get(k) == 0.0) {
                  if (fabs(a.Get(j, k)) >= big) {
                     big = fabs(a.Get(j, k));
                     irow = j;
                     icol = k;
                  }
               } else if (ipiv.Get(k) > 1.) {
                  // singular matrix
                  printf("\nWarning: Inversion of a singular matrix in the inverse() function (> 1 val).\n");
                  return false;
               }
            }  // end for(int k=1; k<=n; k++)
         }     // end if(ipiv.get(j) != 1.)
      }        // end for(int j=1; i<=n; j++)
      ipiv.Set(icol, ipiv.Get(icol) + 1);
      if (irow != icol) {
         // swap
         for (int l = 1; l <= n; l++) {
            double temp_swap;
            temp_swap = a.Get(irow, l);
            a.Set(irow, l, a.Get(icol, l));
            a.Set(icol, l, temp_swap);
         }  // end for(int l=1; l<=n; l++)
      }     // end if(irow != icol)
      indxr.Set(i, (double)irow);
      indxc.Set(i, (double)icol);
      if (a.Get(icol, icol) == 0.0) {
         // singular matrix
         printf("\nWarning: Inversion of a singular matrix in the inverse() function (0 val).\n");
         return false;
      }
      double pivinv = 1.0 / a.Get(icol, icol);
      a.Set(icol, icol, 1.);
      for (int l = 1; l <= n; l++) {
         a.Set(icol, l, pivinv * a.Get(icol, l));
      }  // end for(int l=1; l<=n; l++)

      for (int ll = 1; ll <= n; ll++) {
         if (ll != icol) {
            double dum = a.Get(ll, icol);
            a.Set(ll, icol, 0.);
            for (int l = 1; l <= n; l++) {
               a.Set(ll, l, a.Get(ll, l) - dum * a.Get(icol, l));
            }  // end for(int l=1; l<=n; l++)
         }     // end if(ll != icol)
      }        // end for(int ll=1; ll<=n; ll++)
   }           // end for(int i=1; i<=n; i++)

   for (int l = n; l >= 1; l--) {
      if (indxr.Get(l) != indxc.Get(l)) {
         for (int k = 1; k <= n; k++) {
            // swap:
            double temp;
            temp = a.Get(k, (int)indxr.Get(l));
            a.Set(k, (int)indxr.Get(l), a.Get(k, (int)indxc.Get(l)));
            a.Set(k, (int)indxc.Get(l), temp);
         }
      }  // end if(indxr.get(l) != indxc.get(l))
   }     // end for(int l=n; l>=1; l--)

   // copy the "a" matrix into the "out" matrix:
   int out_min_row = out.GetMinRow();
   int out_min_column = out.GetMinColumn();
   for (int i = 1; i <= n; i++) {
      for (int j = 1; j <= n; j++) {
         out.Set(i - 1 + out_min_row, j - 1 + out_min_column, a.Get(i, j));
      }
   }
   return true;
}

void matrix_times_vector(DMatrix &matrix_in, DVector &vector_in, int n, DVector &vector_out) {

   for (int i = 0; i < n; i++) {
      int ii = i + vector_out.GetMin();
      vector_out[ii] = 0.0;
      for (int j = 0; j < n; j++) {
         vector_out[ii] +=
               matrix_in[i + matrix_in.GetMinRow()][j + matrix_in.GetMinColumn()] * vector_in[j + vector_in.GetMin()];
      }
   }
}

/**
 * Create a matrix which executes a 3-D rotation of a
 * point around a vector <l,m,n> when a single-row
 * matrix [x y z] is post-multiplied by the rotation
 * matrix.
 */
DMatrix &createRotationMatrix(double l, double m, double n, const Units::Angle theta) {

   // basic formula acquired from:
   // https://en.wikipedia.org/wiki/Transformation_matrix#Rotation_2
   // Wikipedia uses T * coord_column, while we use coord_row * T.
   // Therefore, we must transpose the matrix.

   // we need a unit vector
   double mag2 = l * l + m * m + n * n;
   if (mag2 != 1) {
      double mag = sqrt(mag2);
      l /= mag;
      m /= mag;
      n /= mag;
   }

   double cosT = cos(theta);
   double sinT = sin(theta);
   double cosT1 = 1 - cosT;

   double a[3][3] = {{l * l * cosT1 + cosT, m * l * cosT1 + n * sinT, n * l * cosT1 - m * sinT},
                     {l * m * cosT1 - n * sinT, m * m * cosT1 + cosT, n * m * cosT1 + l * sinT},
                     {l * n * cosT1 + m * sinT, m * n * cosT1 - l * sinT, n * n * cosT1 + cosT}};
   DMatrix *result = new DMatrix((double **)&a, 0, 2, 0, 2);
   return *result;
}
