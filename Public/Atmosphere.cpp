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

#include "public/Atmosphere.h"
#include "math/CustomMath.h"
#include <list>

using namespace std;

Atmosphere::Atmosphere() {
}

Atmosphere::~Atmosphere() {
}

// Inputs:
//   altitude, h (meters)
// Outputs:
//   air density, rho (kg/m^3)
//   air pressure, P (N/m^2)

void Atmosphere::airDensity(const Units::Length h,
                            Units::Density &rho,
                            Units::Pressure &P) const {

   //global h_trop G T0 RHO0 P0 R K_T rho_trop P_trop;

   // Find air temperature (Kelvin), density (kg/m^3), and pressure (kg/m^2)
   Units::KelvinTemperature T = getTemp(h);

   if (h < H_TROP) {
      rho = RHO0 * pow((T / T0), (-Units::ONE_G_ACCELERATION / (K_T * R) - 1));
      P = P0 * pow((T / T0), (-Units::ONE_G_ACCELERATION / (K_T * R)));
   } else {
      rho = RHO_TROP * exp(-Units::ONE_G_ACCELERATION / (R * T) * (h - H_TROP));
      P = P_TROP * exp(-Units::ONE_G_ACCELERATION / (R * T) * (h - H_TROP));
   }

}

void Atmosphere::calcWindGrad(const Units::Length h_star_,
                              const WindStack &wind,
                              Units::Speed &w_dir,
                              Units::Frequency &w_dir_grad) const {
   // Computes wind gradients from input winds for a given altitude.
   //
   // h_star_:altitude in meters.
   // wind:wind data consisting of altitudes (feet) and wind speeds (knots).
   //      Column 1 is altitude, column 2 is the wind speed.  Wind sorted
   //      in ascending sequence by altitude.
   // w_dir:wind direction.
   // w_dir_grad:wind gradient with respect to altitude.


   // Set altitude in feet.

   // so we can use value()
   Units::FeetLength h_star(h_star_);

   // TODO:Determine effect if following h_star if block removed.

   // LAW - if h_star exceeds max altitude, set it to max altitude
   Units::FeetLength maxAlt(wind.getAltitude(wind.get_max_row()));
   Units::FeetLength minAlt(wind.getAltitude(wind.get_min_row()));
   if (h_star > maxAlt) {
      h_star = maxAlt;
   } else if (h_star < minAlt) {
      h_star = minAlt;
   }

   // Find 5 data points for calculating the wind and wind gradient.

   // Compute low index into wind to take wind data from.  The wind
   // data used will be from the low index through the low index + 4.

   int lowIx;
   int number_of_rows_of_wind_matrix = wind.get_max_row() - wind.get_min_row() + 1;
   if (number_of_rows_of_wind_matrix == 5) {
      // Take whole of wind matrix.

      lowIx = wind.get_min_row();
   } else if (h_star <= wind.getAltitude(wind.get_min_row() + 2)) {
      // Altitude at low end-take bottom 5 winds.

      lowIx = wind.get_min_row();
   } else if (h_star >= wind.getAltitude(wind.get_max_row() - 2)) {
      // Altitude at high end-take top 5 winds.

      lowIx = wind.get_max_row() - 4;
   } else {
      // Altitude somewhere in the middle-compute index to take.

      // Find first wind entry >= to altitude.

      // TODO binary search here would be more efficient
      lowIx = wind.get_min_row();
      while ((lowIx < wind.get_max_row()) &&
             (wind.getAltitude(lowIx) < h_star)) {
         lowIx++;
      }

      // Error check which shouldn't occur here because these conditions
      // are covered above.  Adjust the low index and output an error
      // message.  The condition here could be a symptom of an error with
      // the input data, (particularly the wind data).

      if ((lowIx - 2) < wind.get_min_row()) {
         cout << "calcWindGrad:Unexpected low index calculation for altitude "
              << h_star << endl << "index set to take lowest 5 wind values"
              << endl;
         lowIx = wind.get_min_row();
      } else if ((lowIx - 2) > (wind.get_max_row() - 4)) {
         cout << "calcWindGrad:Unexpected high index calculation for altitude "
              << h_star << endl << "index set to take highest 5 wind values"
              << endl;
         lowIx = wind.get_max_row() - 4;
      } else {

         // Compute final low index.

         // Adjust low index downward 1 if altitude closer to lower wind entry
         // than the one at the low index.

         if ((h_star - wind.getAltitude(lowIx - 1)) <
             (wind.getAltitude(lowIx) - h_star)) {
            lowIx = lowIx - 1;
         }

         // Low index now set to altitude closest to h_star.  Take the
         // data from the two below, data low index, and the two above.

         lowIx = lowIx - 2;

      }

   }


   // Set altitude and velocity vectors.

   DVector alt(1, 5);   // feet
   DVector vel(1, 5);   // knots

   int rowIx = 1;

   for (int ix = lowIx; ix <= (lowIx + 4); ix++) {
      alt.set(rowIx, wind.getAltitude(ix) / Units::FeetLength(1));
      vel.set(rowIx, wind.getSpeed(ix) / Units::KnotsSpeed(1));
      rowIx++;
   }

   //double h = alt[2]-alt[1];

   // Calculate the Second Derivatives calculated at internal points
   //A is a 3x3 matrix; the first row is 5 1 0; the 2nd row is 1 4 1; etc.

   //Change A to a DMatrix:
   //double **A_original = new double[3][3];
   //double A_original[3][3] = {{5, 1, 0}, {1, 4, 1}, {0, 1, 5}};
   double A_original[3][3] = {{0, 0, 0},
                              {0, 0, 0},
                              {0, 0, 0}};
   DMatrix A((double **) &A_original, 1, 3, 1, 3);


   DVector B(1, 3);
   DVector M_(1, 3);

   //
   double h2 = alt[2] - alt[1];
   double h3 = alt[3] - alt[2];
   double h4 = alt[4] - alt[3];
   double h5 = alt[5] - alt[4];

   // 1st Row of A Matrix
   A.set(1, 1, -1.0 / 2.0 * h2 - 1.0 / 3.0 * h3);
   A.set(1, 2, -1.0 / 6.0 * h3);
   A.set(1, 3, 0);
   B.set(1, (vel[2] - vel[1]) / h2 - (vel[3] - vel[2]) / h3);

   // 2nd Row of A Matrix
   A.set(2, 1, -1.0 / 6.0 * h3);
   A.set(2, 2, -1.0 / 3.0 * h3 - 1.0 / 3.0 * h4);
   A.set(2, 3, -1.0 / 6.0 * h4);
   B.set(2, (vel[3] - vel[2]) / h3 - (vel[4] - vel[3]) / h4);

   // 3rd Row of A Matrix
   A.set(3, 1, 0);
   A.set(3, 2, -1.0 / 6.0 * h4);
   A.set(3, 3, -1.0 / 3.0 * h4 - 1.0 / 2.0 * h5);
   B.set(3, (vel[4] - vel[3]) / h4 - (vel[5] - vel[4]) / h5);

   /*
   for(int loop = 2; loop < 5; loop++)
   {
      double h1 = alt[loop] - alt[loop-1];
      double h2 = alt[loop+1] - alt[loop];

      A.set(loop-1,1,);
      A.set(loop-1,2,);
      A.set(loop-1,3,);

      double tmp = (vel[loop]-vel[loop-1])/h1 - (vel[loop+1]-vel[loop])/h2;
      B.set(loop-1, tmp);
   }*/
   /*
   printf("\nB:\n");
   for(int i= B.get_min(); i<= B.get_max(); i++)
   {
      printf("%f,%f,%f\t", A[i][1], A[i][2], A[i][3]);
      printf("%f\t", B[i]);
      printf("\n");

   }
   printf("\n");
   */

   //B is a 3x1 vector
   //M_ is a 3x1 vector

   //M_ = inv(A)*6/h^2*B; //M = [M(1); M; M(end)];

   DMatrix A_inverse(1, 3, 1, 3);
   inverse(A, 3, A_inverse);

   /*
   printf("\nA_inverse\n");
   for(int i= A_inverse.get_min_row(); i<= A_inverse.get_max_row(); i++)
   {
      for(int j= A_inverse.get_min_colomn(); j<= A_inverse.get_max_colomn(); j++)
      {
         printf("%f\t", A_inverse[i][j]);
      }
      printf("\n");
   }
   */

   DVector tmp(1, 3);

   //tmp = A_inverse*B:
   matrix_times_vector(A_inverse, B, 3, M_);
/*
	printf("\ntmp:\n");
	for(int i= tmp.get_min(); i<= tmp.get_max(); i++)
	{
		printf("%f\t", tmp[i]);

	}
*/
   //M_ = A_inverse*6/h^2*B; //M = [M(1); M; M(end)];

   //vector_times_scalar( tmp, 6/SQR(h), M_ );
/*
	printf("\nM_ :\n");
	for(int i= M_ .get_min(); i<= M_ .get_max(); i++)
	{
		printf("%f\t", M_ [i]);

	}
*/
   //M is a 5x1 vector
   DVector M(1, 5); // M = [M_(1);M_;M_(end)];
   M.set(1, M_[1]);
   M.set(2, M_[1]);
   M.set(3, M_[2]);
   M.set(4, M_[3]);
   M.set(5, M_[3]);

   //debug

   //// Generate Curve through Points
   DVector a(1, 5);
   DVector b(1, 5);
   DVector c(1, 5);
   DVector d(1, 5);
   DVector x(1, 5);
   /*
   printf("\nM:\n");
   for(int i= M.get_min(); i<= M.get_max(); i++)
   {
      printf("%f\t", M[i]);

   }
   */


   for (int ind1 = 1; ind1 < 5; ind1++) {
      //4x1 vectors
      double h = alt[ind1 + 1] - alt[ind1];

      a.set(ind1, (M[ind1 + 1] - M[ind1]) / (6 * h));

      b.set(ind1, M[ind1] / 2);

      c.set(ind1, (vel[ind1 + 1] - vel[ind1]) / h - (M[ind1 + 1] + 2 * M[ind1]) / 6 * h);

      //double tmp = vel[ind1];
      d.set(ind1, vel[ind1]);

      x.set(ind1, alt[ind1]);
   }

   // Calculate gradient at h_star:
   // loop to load new altitude


   bool found = false;
   DVector new_alt(alt.get_min(), alt.get_max() + 1);
   for (int loop = alt.get_min(); loop <= alt.get_max(); loop++) {
      // if/else to add the h_star data in the correct place for the data to be sorted
      if (alt[loop] <= h_star.value()) {
         new_alt.set(loop, alt[loop]);
         /*
         printf("%d\t%f\n", loop, alt[loop]);
         */
      } else if (alt[loop] > h_star.value() && found == true) {
         new_alt.set(loop, alt[loop - 1]);
         //printf("%d\t%f\n", loop, alt[loop-1]);
      } else {
         found = true;
         new_alt.set(loop, h_star.value());
         //new_alt.set(loop+1,alt[loop]);
         //printf("%d\t%f\n", loop, h_star);
//			printf("%d\t%f\n", loop+1, alt[loop]);
//			loop++;
      }
   }
   if (found == false) {
      new_alt.set(new_alt.get_max(), h_star.value());
      //printf("%d\t%f\n", new_alt.get_max(), h_star);
   } else {
      new_alt.set(new_alt.get_max(), alt[alt.get_max()]);
      //printf("%d\t%f\n", new_alt.get_max(), new_alt[new_alt.get_max()]);
   }

/*
	printf("\nalt :\n");
	for(int i= alt.get_min(); i<= alt.get_max(); i++)
	{
		printf("%f\t", alt[i]);

	}


	printf("\nnew_alt :\n");
	for(int i= new_alt.get_min(); i<= new_alt.get_max(); i++)
	{
		printf("%f\t", new_alt[i]);

	}
*/

   //alt = new_alt;

   //ind = find(alt == h_star);
   //find the indexes  where alt==h_star; ind is a vector
   // loop to go through altitude list and find index that match


   int index = new_alt.get_min();
   list<int> ind;
   ind.clear();
   for (int loop = new_alt.get_min(); loop < new_alt.get_max(); loop++) {
      if (new_alt[loop] == h_star.value()) {
         ind.push_back(index);
      }

      index++;
   }

   if (ind.size() == 1 && (*ind.begin()) > 1) {
      // NOTE! Switched to using the new ind, formula itself needs to be tinkered with
      //use pow():
      w_dir = Units::KnotsSpeed(a[(*ind.begin()) - 1] * pow((h_star.value() - x[(*ind.begin()) - 1]), 3) +
                                b[(*ind.begin()) - 1] * pow(h_star.value() - x[(*ind.begin()) - 1], 2) +
                                c[(*ind.begin()) - 1] * (h_star.value() - x[(*ind.begin()) - 1]) +
                                d[(*ind.begin()) - 1]);
      w_dir_grad = Units::KnotsPerFootFrequency(
            3 * a[(*ind.begin()) - 1] * pow(h_star.value() - x[(*ind.begin()) - 1], 2) +
            2 * b[(*ind.begin()) - 1] * (h_star.value() - x[(*ind.begin()) - 1]) + c[(*ind.begin()) - 1]);
   } else if (ind.size() != 0) {
      // NOTE! Switched to using the new ind, formula itself needs to be tinkered with
/*		double tmp1 = a[(*ind.begin())];
		double tmp2 = x[(*ind.begin())];
		double tmp3 = b[(*ind.begin())];
		double tmp4 = c[(*ind.begin())];
		double tmp5 = d[(*ind.begin())];
		double tmp6 = a[(*ind.begin())]*pow((h_star - x[(*ind.begin())]),3);
		double tmp10 = SQR(h_star - x[(*ind.begin())]);
		double tmp7 = b[(*ind.begin())]*SQR(h_star - x[(*ind.begin())]);
		double tmp8 = c[(*ind.begin())]*(h_star - x[(*ind.begin())]);
		double tmp9 = d[(*ind.begin())];*/
      w_dir = Units::KnotsSpeed(a[(*ind.begin())] * pow((h_star.value() - x[(*ind.begin())]), 3) +
                                b[(*ind.begin())] * pow(h_star.value() - x[(*ind.begin())], 2) +
                                c[(*ind.begin())] * (h_star.value() - x[(*ind.begin())]) + d[(*ind.begin())]);
      w_dir_grad = Units::KnotsPerFootFrequency(3 * a[(*ind.begin())] * pow(h_star.value() - x[(*ind.begin())], 2) +
                                                2 * b[(*ind.begin())] * (h_star.value() - x[(*ind.begin())]) +
                                                c[(*ind.begin())]);
   }


   w_dir_grad = -w_dir_grad;   // Wind gradient is negated for sign convention

   // Convert to metric units
   //w_dir = w_dir*KTS_MPS;  //(meter/second)
   //w_dir_grad = w_dir_grad*KTS_MPS/FT_M; //(/second)
} // calcWindGrad


// Inputs:
//   calibrate air speed, vcas
//   altitude, alt
// Outputs:
//   true air speed, Vtas
Units::Speed Atmosphere::CAS2TAS(const Units::Speed vcas,
                                 const Units::Length alt) const {
   // TODO:Conversion of guts of mathematics in this?

   // global RHO0 P0 mu;

   // Get the air density
   Units::KilogramsMeterDensity rho;
   Units::Pressure p;

   airDensity(alt, rho, p);

   // Terms in the conversion
   double temp1 = 1 + MU / 2 * (RHO0.value() / P0.value()) * pow(Units::MetersPerSecondSpeed(vcas).value(), 2);

   double temp2 = pow(temp1, (1 / MU)) - 1;

   double temp3 = pow((1 + P0 / p * temp2), MU);

   Units::Speed speed = sqrt(2 / MU * p / rho * (temp3 - 1));

   return speed;
}


//--------------------------------
// Inputs:
//   true air speed, vtas.
//   altitude, alt.
// Outputs:
//   calibrated air speed.
Units::Speed Atmosphere::TAS2CAS(const Units::Speed vtas,
                                 const Units::Length alt) const {
   //global RHO0 P0 MU;

   // Get the air density
   Units::KilogramsMeterDensity rho;
   Units::Pressure p;

   airDensity(alt, rho, p);

   // Terms in the conversion
   double temp1 = 1 + MU / 2 * (rho / p) * Units::sqr(vtas);

   double temp2 = pow(temp1, (1 / MU)) - 1;

   double temp3 = pow((1 + (p / P0) * temp2), MU);

   Units::Speed speed = sqrt(2 / MU * (P0 / RHO0) * (temp3 - 1));

   return speed;
}


// method to calculate the MACH to IAS transition.
Units::Length Atmosphere::GetMachIASTransition(const Units::Speed &ias,
                                               const double &mach) const {

   // temp values to store the MACH IAS transition calculations
   double temp1, temp2, temp3, temp4;

   double deltaTrans, thetaTrans;

   // calculates the delta transition dividend
   temp1 = 1 + ((GAMMA - 1) / 2) * pow((Units::MetersPerSecondSpeed(ias).value() / A0.value()), 2);
   temp2 = pow(temp1, (GAMMA / (GAMMA - 1))) - 1;

   // calculates the delta transition divisor
   temp3 = 1 + ((GAMMA - 1) / 2) * pow(mach, 2);
   temp4 = pow(temp3, (GAMMA / (GAMMA - 1))) - 1;

   // calculate delta transition
   deltaTrans = temp2 / temp4;

   // calculate theta transition
   thetaTrans = pow(deltaTrans, (-K_T.value() * R.value() / GRAV_MPS));

   // calculates Mach IAS Transition (Transition Altitude)

   return Units::FeetLength(
         (Units::FeetLength(Units::MetersLength(1000.0)).value() / 6.5) * T0.value() * (1 - thetaTrans));

}

Units::Speed Atmosphere::machToIAS(const double mach,
                                   const Units::Length alt) const {
   // Converts mach to ias.
   //
   // mach:speed in mach.
   // alt:to calculate ias at.
   //
   // return:ias.


   Units::Speed tas = Units::MetersPerSecondSpeed(mach *
                                                  sqrt(GAMMA * R * getTemp(alt).value()));

   return TAS2CAS(tas, alt);
}
