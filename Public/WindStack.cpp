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

#include "public/WindStack.h"

#include <algorithm>
#include <list>

#include "utility/CustomUnits.h"
#include "public/CustomMath.h"

using namespace aaesim::open_source;
using namespace std;

WindStack::WindStack() : m_altitude(), m_speed(), m_minimum_data_index(0), m_maximum_data_index(0) {
   SetBounds(m_minimum_data_index, m_maximum_data_index);
}

WindStack::WindStack(const int min, const int max)
   : m_altitude(), m_speed(), m_minimum_data_index(min), m_maximum_data_index(max) {
   SetBounds(min, max);
}

WindStack::WindStack(const WindStack &in) { Copy(in); }

WindStack &WindStack::operator=(const WindStack &in) {
   Copy(in);
   return *this;
}

void WindStack::Copy(const WindStack &in) {
   m_minimum_data_index = in.m_minimum_data_index;
   m_maximum_data_index = in.m_maximum_data_index;
   m_altitude = in.m_altitude;
   m_speed = in.m_speed;
}

bool WindStack::operator==(const WindStack &obj) const {
   bool match = ((GetMinRow() == obj.GetMinRow()) && (GetMaxRow() == obj.GetMaxRow()));
   if (match && m_minimum_data_index != m_maximum_data_index) {
      for (auto ix = GetMinRow(); (match && (ix <= GetMaxRow())); ++ix) {
         match = match && (GetAltitude(ix) == obj.GetAltitude(ix)) && (GetSpeed(ix) == obj.GetSpeed(ix));
      }
   }
   return match;
}

bool WindStack::operator!=(const WindStack &obj) const { return !operator==(obj); }

Units::FeetLength WindStack::GetAltitude(const int index) const { return m_altitude.at(index); }

Units::KnotsSpeed WindStack::GetSpeed(const int index) const { return m_speed.at(index); }

void WindStack::SetBounds(int min, int max) {
   m_minimum_data_index = min;
   m_maximum_data_index = max;

   m_altitude.clear();
   m_speed.clear();

   for (int i = 0; i <= max; ++i) {
      m_altitude.push_back(Units::Infinity());
      m_speed.push_back(Units::Infinity());
   }
}

void WindStack::Insert(const int index, const Units::Length altitude, const Units::Speed speed) {
   m_altitude.at(index) = altitude;
   m_speed.at(index) = speed;
}

void WindStack::SortAltitudesAscending() {
   if (m_minimum_data_index == m_maximum_data_index) return;

   std::vector<std::pair<Units::Length, Units::Speed>> zipped_data;
   for (auto idx = m_minimum_data_index; idx <= m_maximum_data_index; ++idx) {
      zipped_data.push_back(std::make_pair(m_altitude.at(idx), m_speed.at(idx)));
   }
   std::sort(zipped_data.begin(), zipped_data.end(), AltitudeComparator);

   auto insert_index = m_minimum_data_index;
   auto vector_inserter = [this, &insert_index](const std::pair<Units::Length, Units::Speed> item) {
      m_altitude.at(insert_index) = item.first;
      m_speed.at(insert_index) = item.second;
      ++insert_index;
   };
   std::for_each(zipped_data.cbegin(), zipped_data.cend(), vector_inserter);
}

WindStack WindStack::CreateZeroSpeedStack() {
   WindStack wind_stack(0, 4);
   Units::FeetLength altitude(1000), altitude_step(10000);
   for (auto i = wind_stack.GetMinRow(); i <= wind_stack.GetMaxRow(); ++i) {
      wind_stack.Insert(i, altitude, Units::zero());
      altitude += altitude_step;
   }
   return wind_stack;
}

void WindStack::CalculateWindGradientAtAltitude(const Units::Length altitude_in, Units::Speed &wind_speed,
                                                Units::Frequency &wind_gradient) const {
   const WindStack &wind_stack(*this);  // was a parameter to a static function
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
