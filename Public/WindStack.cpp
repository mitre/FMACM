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

#include "utility/CustomUnits.h"

using namespace aaesim::open_source;

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
