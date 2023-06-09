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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/Atmosphere.h"
#include "math/CustomMath.h"
#include <list>
#include <log4cplus/loggingmacros.h>

using namespace std;

log4cplus::Logger Atmosphere::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("Atmosphere"));

Atmosphere::Atmosphere() {}

Atmosphere::~Atmosphere() = default;

// Inputs:
//   altitude, h (meters)
// Outputs:
//   air density, rho (kg/m^3)
//   air pressure, P (N/m^2)

void Atmosphere::AirDensity(const Units::Length h, Units::Density &rho, Units::Pressure &P) const {

   // global h_trop G T0 RHO0 P0 R K_T rho_trop P_trop;

   // Find air temperature (Kelvin), density (kg/m^3), and pressure (kg/m^2)
   Units::KelvinTemperature T = GetTemperature(h);
   Units::KelvinTemperature T0 = GetSeaLevelTemperature();

   Units::MetersLength H_TROP(GetTropopauseHeight());
   if (h < H_TROP) {
      Units::Density RHO0 = GetSeaLevelDensity();
      rho = RHO0 * pow((T / T0), RHO_T_EXPONENT);  // BADA_37_USER_MANUAL eq. 3.2-6
      P = P0_ISA * pow((T / T0), P_T_EXPONENT);    // BADA_37_USER_MANUAL eq. 3.2-15
   } else {
      const double factor(exp(-Units::ONE_G_ACCELERATION / (R * T) * (h - H_TROP)));
      rho = GetTropopauseDensity() * factor;  // BADA_37_USER_MANUAL eq. 3.2-9
      P = GetTropopausePressure() * factor;   // BADA_37_USER_MANUAL eq. 3.2-16
   }
}

void Atmosphere::CalculateWindGradientAtAltitude(const Units::Length altitude_in, const WindStack &wind_stack,
                                                 Units::Speed &wind_speed, Units::Frequency &wind_gradient) const {
   Units::FeetLength altitude = altitude_in;

   Units::FeetLength maximum_altitude = wind_stack.GetAltitude(wind_stack.GetMaxRow());
   Units::FeetLength minimum_altitude = wind_stack.GetAltitude(wind_stack.GetMinRow());
   if (altitude > maximum_altitude) {
      altitude = maximum_altitude;
   } else if (altitude < minimum_altitude) {
      altitude = minimum_altitude;
   }

   // Find appropriate altitudes to sample for wind gradient. The wind data used will be from the low index through
   // low index + 4.
   int low_index;
   int number_of_rows_of_wind_matrix = wind_stack.GetMaxRow() - wind_stack.GetMinRow() + 1;
   if (number_of_rows_of_wind_matrix == 5) {
      low_index = wind_stack.GetMinRow();
   } else if (altitude <= wind_stack.GetAltitude(wind_stack.GetMinRow() + 2)) {
      // Altitude at low end-take bottom 5 winds.
      low_index = wind_stack.GetMinRow();
   } else if (altitude >= wind_stack.GetAltitude(wind_stack.GetMaxRow() - 2)) {
      // Altitude at high end-take top 5 winds.
      low_index = wind_stack.GetMaxRow() - 4;
   } else {
      // Altitude somewhere in the middle-compute index to take.
      low_index = wind_stack.GetMinRow();
      while ((low_index < wind_stack.GetMaxRow()) && (wind_stack.GetAltitude(low_index) < altitude)) {
         low_index++;
      }

      Units::FeetLength upper_bounding_altitude = wind_stack.GetAltitude(low_index);
      Units::FeetLength lower_bounding_altitude = wind_stack.GetAltitude(low_index - 1);

      if ((altitude - lower_bounding_altitude) < (upper_bounding_altitude - altitude)) {
         low_index = low_index - 1;
      }
      low_index = low_index - 2;
   }

   DVector wind_altitudes_ft(1, 5);
   DVector wind_velocities_knots(1, 5);

   int row_index = 1;
   for (int i = low_index; i <= (low_index + 4); i++) {
      wind_altitudes_ft.Set(row_index, wind_stack.GetAltitude(i) / Units::FeetLength(1));
      wind_velocities_knots.Set(row_index, wind_stack.GetSpeed(i) / Units::KnotsSpeed(1));
      row_index++;
   }

   double A_original[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
   DMatrix A((double **)&A_original, 1, 3, 1, 3);

   DVector B(1, 3);
   DVector M_(1, 3);

   double h2 = wind_altitudes_ft[2] - wind_altitudes_ft[1];
   double h3 = wind_altitudes_ft[3] - wind_altitudes_ft[2];
   double h4 = wind_altitudes_ft[4] - wind_altitudes_ft[3];
   double h5 = wind_altitudes_ft[5] - wind_altitudes_ft[4];

   // 1st Row of A Matrix
   A.Set(1, 1, -1.0 / 2.0 * h2 - 1.0 / 3.0 * h3);
   A.Set(1, 2, -1.0 / 6.0 * h3);
   A.Set(1, 3, 0);
   B.Set(1, (wind_velocities_knots[2] - wind_velocities_knots[1]) / h2 -
                  (wind_velocities_knots[3] - wind_velocities_knots[2]) / h3);

   // 2nd Row of A Matrix
   A.Set(2, 1, -1.0 / 6.0 * h3);
   A.Set(2, 2, -1.0 / 3.0 * h3 - 1.0 / 3.0 * h4);
   A.Set(2, 3, -1.0 / 6.0 * h4);
   B.Set(2, (wind_velocities_knots[3] - wind_velocities_knots[2]) / h3 -
                  (wind_velocities_knots[4] - wind_velocities_knots[3]) / h4);

   // 3rd Row of A Matrix
   A.Set(3, 1, 0);
   A.Set(3, 2, -1.0 / 6.0 * h4);
   A.Set(3, 3, -1.0 / 3.0 * h4 - 1.0 / 2.0 * h5);
   B.Set(3, (wind_velocities_knots[4] - wind_velocities_knots[3]) / h4 -
                  (wind_velocities_knots[5] - wind_velocities_knots[4]) / h5);

   DMatrix A_inverse(1, 3, 1, 3);
   inverse(A, 3, A_inverse);

   DVector tmp(1, 3);

   matrix_times_vector(A_inverse, B, 3, M_);

   DVector M(1, 5);
   M.Set(1, M_[1]);
   M.Set(2, M_[1]);
   M.Set(3, M_[2]);
   M.Set(4, M_[3]);
   M.Set(5, M_[3]);

   DVector a(1, 5);
   DVector b(1, 5);
   DVector c(1, 5);
   DVector d(1, 5);
   DVector x(1, 5);
   for (int ind1 = 1; ind1 < 5; ind1++) {
      double h = wind_altitudes_ft[ind1 + 1] - wind_altitudes_ft[ind1];
      a.Set(ind1, (M[ind1 + 1] - M[ind1]) / (6 * h));
      b.Set(ind1, M[ind1] / 2);
      c.Set(ind1,
            (wind_velocities_knots[ind1 + 1] - wind_velocities_knots[ind1]) / h - (M[ind1 + 1] + 2 * M[ind1]) / 6 * h);
      d.Set(ind1, wind_velocities_knots[ind1]);
      x.Set(ind1, wind_altitudes_ft[ind1]);
   }

   bool found = false;
   DVector new_altitudes(wind_altitudes_ft.GetMin(), wind_altitudes_ft.GetMax() + 1);
   for (int loop = wind_altitudes_ft.GetMin(); loop <= wind_altitudes_ft.GetMax(); loop++) {
      if (wind_altitudes_ft[loop] <= altitude.value()) {
         new_altitudes.Set(loop, wind_altitudes_ft[loop]);
      } else if (wind_altitudes_ft[loop] > altitude.value() && found) {
         new_altitudes.Set(loop, wind_altitudes_ft[loop - 1]);
      } else {
         found = true;
         new_altitudes.Set(loop, altitude.value());
      }
   }

   if (!found) {
      new_altitudes.Set(new_altitudes.GetMax(), altitude.value());
   } else {
      new_altitudes.Set(new_altitudes.GetMax(), wind_altitudes_ft[wind_altitudes_ft.GetMax()]);
   }

   int index = new_altitudes.GetMin();
   list<int> ind;
   for (int loop = new_altitudes.GetMin(); loop < new_altitudes.GetMax(); loop++) {
      if (new_altitudes[loop] == altitude.value()) {
         ind.push_back(index);
      }
      index++;
   }

   if (ind.size() == 1 && (*ind.begin()) > 1) {
      wind_speed = Units::KnotsSpeed(a[(*ind.begin()) - 1] * pow((altitude.value() - x[(*ind.begin()) - 1]), 3) +
                                     b[(*ind.begin()) - 1] * pow(altitude.value() - x[(*ind.begin()) - 1], 2) +
                                     c[(*ind.begin()) - 1] * (altitude.value() - x[(*ind.begin()) - 1]) +
                                     d[(*ind.begin()) - 1]);
      wind_gradient = Units::KnotsPerFootFrequency(
            3 * a[(*ind.begin()) - 1] * pow(altitude.value() - x[(*ind.begin()) - 1], 2) +
            2 * b[(*ind.begin()) - 1] * (altitude.value() - x[(*ind.begin()) - 1]) + c[(*ind.begin()) - 1]);
   } else if (!ind.empty()) {
      wind_speed = Units::KnotsSpeed(a[(*ind.begin())] * pow((altitude.value() - x[(*ind.begin())]), 3) +
                                     b[(*ind.begin())] * pow(altitude.value() - x[(*ind.begin())], 2) +
                                     c[(*ind.begin())] * (altitude.value() - x[(*ind.begin())]) + d[(*ind.begin())]);
      wind_gradient = Units::KnotsPerFootFrequency(
            3 * a[(*ind.begin())] * pow(altitude.value() - x[(*ind.begin())], 2) +
            2 * b[(*ind.begin())] * (altitude.value() - x[(*ind.begin())]) + c[(*ind.begin())]);
   }

   if (wind_gradient != Units::zero()) {
      wind_gradient = -wind_gradient;
   }
}

// Inputs:
//   calibrate air speed, vcas
//   altitude, alt
// Outputs:
//   true air speed, Vtas
Units::Speed Atmosphere::CAS2TAS(const Units::Speed vcas, const Units::Length alt) const {
   // global RHO0 P0 mu;

   // Get the air density
   Units::KilogramsMeterDensity rho;
   Units::Pressure p;

   AirDensity(alt, rho, p);

   Units::Speed vtas = CAS2TAS(vcas, p, rho);
   return vtas;
}

Units::Speed Atmosphere::CAS2TAS(const Units::Speed vcas, const Units::Pressure p, const Units::Density rho) {

   // All calculations are from BADA_37_USER_MANUAL eq. 3.2-12
   // expression inside innermost parentheses
   double temp1 = 1 + MU / 2 * (RHO0_ISA / P0_ISA) * Units::sqr(vcas);

   // expression inside inner square brackets
   double temp2 = pow(temp1, (1 / MU)) - 1;

   // expression inside curly braces
   double temp3 = pow((1 + P0_ISA / p * temp2), MU) - 1;

   // full equation
   Units::Speed speed = sqrt(2 / MU * p / rho * temp3);

   LOG4CPLUS_TRACE(m_logger, "CAS " << Units::KnotsSpeed(vcas) << " at " << Units::PascalsPressure(p) << " and "
                                    << Units::KilogramsMeterDensity(rho) << " is TAS " << Units::KnotsSpeed(speed));

   return speed;
}

//--------------------------------
// Inputs:
//   true air speed, vtas.
//   altitude, alt.
// Outputs:
//   calibrated air speed.
Units::Speed Atmosphere::TAS2CAS(const Units::Speed vtas, const Units::Length alt) const {
   // global RHO0 P0 MU;

   // Get the air density
   Units::KilogramsMeterDensity rho;
   Units::Pressure p;

   AirDensity(alt, rho, p);

   Units::Speed vcas = TAS2CAS(vtas, p, rho);
   return vcas;
}

Units::Speed Atmosphere::TAS2CAS(const Units::Speed vtas, const Units::Pressure p, const Units::Density rho) {

   // All calculations are from BADA_37_USER_MANUAL eq. 3.2-13
   // expression inside innermost parentheses
   double temp1 = 1 + MU / 2 * (rho / p) * Units::sqr(vtas);

   // expression inside inner square brackets
   double temp2 = pow(temp1, (1 / MU)) - 1;

   // expression inside curly braces
   double temp3 = pow((1 + (p / P0_ISA) * temp2), MU) - 1;

   // full equation
   Units::Speed speed = sqrt(2 / MU * (P0_ISA / RHO0_ISA) * temp3);

   return speed;
}

// method to calculate the MACH to IAS transition.
Units::Length Atmosphere::GetMachIASTransition(const Units::Speed &ias, const double &mach) const {

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
   thetaTrans = pow(deltaTrans, (-K_T.value() * R.value() / GRAVITY_METERS_PER_SECOND));

   // calculates Mach IAS Transition (Transition Altitude)

   return Units::FeetLength((Units::FeetLength(Units::MetersLength(1000.0)).value() / 6.5) *
                            GetSeaLevelTemperature().value() * (1 - thetaTrans));
}

Units::Speed Atmosphere::MachToIAS(const double mach, const Units::Length alt) const {
   Units::Speed tas = Units::MetersPerSecondSpeed(mach * sqrt(GAMMA * R * GetTemperature(alt).value()));

   return TAS2CAS(tas, alt);
}

double Atmosphere::ESFconstantCAS(const Units::Speed true_airspeed, const Units::Length altitude_msl,
                                  const Units::KelvinTemperature temperature) {

   double esf;
   double mach;
   double temp1, temp2, temp3;

   mach = true_airspeed / sqrt(GAMMA * R * temperature);

   temp1 = 1.0 + (GAMMA - 1.0) / 2 * pow(mach, 2);
   temp2 = (pow(temp1, (-1.0 / (GAMMA - 1)))) * (pow(temp1, (GAMMA / (GAMMA - 1))) - 1.0);

   if (altitude_msl <= GetTropopauseHeight()) {
      temp3 = 1.0 + (GAMMA * R * K_T) / (Units::ONE_G_ACCELERATION * 2) * pow(mach, 2) + temp2;
   } else {
      temp3 = 1.0 + temp2;
   }

   esf = 1.0 / temp3;
   return esf;
}

Units::Speed Atmosphere::SpeedOfSound(Units::KelvinTemperature temperature) {
   // based on BADA_37_USER_MANUAL eq. 3.2-17 with M=1
   Units::KnotsSpeed speed_of_sound = sqrt(GAMMA * R * temperature);
   return speed_of_sound;
}

Units::Speed Atmosphere::SpeedOfSound(Units::Length altitude) const {
   Units::KelvinTemperature temperature = GetTemperature(altitude);
   return SpeedOfSound(temperature);
}
