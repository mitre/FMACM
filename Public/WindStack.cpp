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
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/WindStack.h"

WindStack::WindStack()
      : m_altitude(),
        m_speed() {
}

WindStack::WindStack(const int min,
                     const int max)
      : m_altitude(),
        m_speed() {
   SetBounds(min, max);
}

WindStack::~WindStack() = default;

bool WindStack::operator==(const WindStack &obj) const {
   bool match = ((GetMinRow() == obj.GetMinRow()) &&
                 (GetMaxRow() == obj.GetMaxRow()));

   for (auto ix = GetMinRow(); (match && (ix <= GetMaxRow())); ix++) {
      match = match && (GetAltitude(ix) == obj.GetAltitude(ix)) &&
              (GetSpeed(ix) == obj.GetSpeed(ix));
   }

   return match;

}

bool WindStack::operator!=(const WindStack &obj) const {
   return !operator==(obj);
}

Units::FeetLength WindStack::GetAltitude(const int index) const {
   return m_altitude.get(index);
}

Units::KnotsSpeed WindStack::GetSpeed(const int index) const {
   return m_speed.get(index);
}

void WindStack::SetBounds(int min,
                          int max) {
   m_altitude.setBounds(min, max);
   m_speed.setBounds(min, max);

   for (int i = min; i <= max; ++i) {
      m_altitude.set(i, Units::FeetLength(Units::Infinity()));
      m_speed.set(i, Units::KnotsSpeed(Units::Infinity()));
   }
}

void WindStack::Set(const int index,
                    const Units::Length altitude,
                    const Units::Speed speed) {
   m_altitude.set(index, altitude);
   m_speed.set(index, speed);
}

void WindStack::AscendSort() {
   bool dirty = true;
   for (int top = m_altitude.get_max(); dirty && (top > m_altitude.get_min()); top--) {
      dirty = false;
      for (int i = m_altitude.get_min(); i < top; i++) {
         if (m_altitude[i] > m_altitude[i + 1]) {
            dirty = true;
            Units::Length tempA = m_altitude[i];
            Units::Speed tempS = m_speed[i];
            m_altitude[i] = m_altitude[i + 1];
            m_speed[i] = m_speed[i + 1];
            m_altitude[i + 1] = tempA;
            m_speed[i + 1] = tempS;
         }
      }
   }
}
