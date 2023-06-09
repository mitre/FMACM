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

#include "public/WindStack.h"

WindStack::WindStack() : m_altitude(), m_speed() {}

WindStack::WindStack(const int min, const int max) : m_altitude(), m_speed() { SetBounds(min, max); }

WindStack::~WindStack() = default;

bool WindStack::operator==(const WindStack &obj) const {
   bool match = ((GetMinRow() == obj.GetMinRow()) && (GetMaxRow() == obj.GetMaxRow()));

   for (auto ix = GetMinRow(); (match && (ix <= GetMaxRow())); ix++) {
      match = match && (GetAltitude(ix) == obj.GetAltitude(ix)) && (GetSpeed(ix) == obj.GetSpeed(ix));
   }

   return match;
}

bool WindStack::operator!=(const WindStack &obj) const { return !operator==(obj); }

Units::FeetLength WindStack::GetAltitude(const int index) const { return m_altitude.get(index); }

Units::KnotsSpeed WindStack::GetSpeed(const int index) const { return m_speed.get(index); }

void WindStack::SetBounds(int min, int max) {
   m_altitude.setBounds(min, max);
   m_speed.setBounds(min, max);

   for (int i = min; i <= max; ++i) {
      m_altitude.set(i, Units::FeetLength(Units::Infinity()));
      m_speed.set(i, Units::KnotsSpeed(Units::Infinity()));
   }
}

void WindStack::Set(const int index, const Units::Length altitude, const Units::Speed speed) {
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

WindStack WindStack::CreateZeroSpeedStack() {
   WindStack wind_stack(0, 4);
   Units::FeetLength altitude(1000), altitude_step(10000);
   for (auto i = wind_stack.GetMinRow(); i <= wind_stack.GetMaxRow(); ++i) {
      wind_stack.Set(i, altitude, Units::zero());
      altitude += altitude_step;
   }
   return wind_stack;
}
