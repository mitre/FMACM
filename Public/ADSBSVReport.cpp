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

#include "public/ADSBSVReport.h"
#include "math/CustomMath.h"

namespace Sensor {

namespace ADSB {
const ADSBSVReport ADSBSVReport::blank_report;

ADSBSVReport::ADSBSVReport() { Clear(); }

ADSBSVReport::~ADSBSVReport() {}

// Copy constructor
ADSBSVReport::ADSBSVReport(const ADSBSVReport &in) {
   m_id = in.m_id;
   m_time = in.m_time;
   m_x = in.m_x;
   m_y = in.m_y;
   m_z = in.m_z;
   m_xd = in.m_xd;
   m_yd = in.m_yd;
   m_zd = in.m_zd;
   m_nacp = in.m_nacp;
   m_nacv = in.m_nacv;
   m_nicp = in.m_nicp;
   m_nicv = in.m_nicv;
   m_has_position = in.m_has_position;
   m_has_velocity = in.m_has_velocity;
   m_horizontal_position_quantum = in.m_horizontal_position_quantum;
   m_horizontal_velocity_quantum = in.m_horizontal_velocity_quantum;
   m_vertical_position_quantum = in.m_vertical_position_quantum;
   m_vertical_velocity_quantum = in.m_vertical_velocity_quantum;
   m_time_quantum = in.m_time_quantum;
}

// Assignment operator
ADSBSVReport &ADSBSVReport::operator=(const ADSBSVReport &in) {
   if (this != &in) {
      m_id = in.m_id;
      m_time = in.m_time;
      m_x = in.m_x;
      m_y = in.m_y;
      m_z = in.m_z;
      m_xd = in.m_xd;
      m_yd = in.m_yd;
      m_zd = in.m_zd;
      m_nacp = in.m_nacp;
      m_nacv = in.m_nacv;
      m_nicp = in.m_nicp;
      m_nicv = in.m_nicv;
      m_has_position = in.m_has_position;
      m_has_velocity = in.m_has_velocity;
      m_horizontal_position_quantum = in.m_horizontal_position_quantum;
      m_horizontal_velocity_quantum = in.m_horizontal_velocity_quantum;
      m_vertical_position_quantum = in.m_vertical_position_quantum;
      m_vertical_velocity_quantum = in.m_vertical_velocity_quantum;
      m_time_quantum = in.m_time_quantum;
   }
   return *this;
}

// Clear
void ADSBSVReport::Clear(void) {
   // Reset reports to original condition
   m_id = -1;
   m_time = Units::SecondsTime(-1);
   m_x = m_y = m_z = Units::FeetLength(0);
   m_xd = m_yd = m_zd = Units::FeetPerSecondSpeed(0);
   m_nacp = m_nacv = m_nicp = m_nicv = 0;
   m_has_position = m_has_velocity = false;
   m_horizontal_position_quantum = m_vertical_position_quantum = Units::FeetLength(-1);
   m_horizontal_velocity_quantum = m_vertical_velocity_quantum = Units::FeetPerSecondSpeed(-1);
}

bool ADSBSVReport::operator==(const ADSBSVReport &in) {
   // rewritten 4/14/16 to ignore fields not included in message

   // fields which must always match
   if (m_id != in.m_id || m_time != in.m_time || m_has_position != in.m_has_position ||
       m_has_velocity != in.m_has_velocity) {
      return false;
   }

   // position fields
   if (m_has_position) {
      if (m_x != in.m_x || m_y != in.m_y || m_z != in.m_z || m_nacp != in.m_nacp || m_nicp != in.m_nicp ||
          m_horizontal_position_quantum != in.m_horizontal_position_quantum ||
          m_vertical_position_quantum != in.m_vertical_position_quantum) {
         return false;
      }
   }

   // velocity fields
   if (this->m_has_velocity) {
      if (m_xd != in.m_xd || m_yd != in.m_yd || m_zd != in.m_zd || m_nacv != in.m_nacv || m_nicv != in.m_nicv ||
          m_horizontal_velocity_quantum != in.m_horizontal_velocity_quantum ||
          m_vertical_velocity_quantum != in.m_vertical_velocity_quantum) {
         return false;
      }
   }

   return true;
}

bool ADSBSVReport::IsHasPosition() const { return m_has_position; }

void ADSBSVReport::SetHasPosition(const bool has_position) { m_has_position = has_position; }

bool ADSBSVReport::IsHasVelocity() const { return m_has_velocity; }

void ADSBSVReport::SetHasVelocity(const bool has_velocity) { m_has_velocity = has_velocity; }

Units::FeetLength ADSBSVReport::GetHorizontalPositionQuantum() const { return m_horizontal_position_quantum; }

Units::FeetPerSecondSpeed ADSBSVReport::GetHorizontalVelocityQuantum() const { return m_horizontal_velocity_quantum; }

void ADSBSVReport::SetId(int id) { m_id = id; }

void ADSBSVReport::SetTime(Units::Time time) { m_time = time; }

int ADSBSVReport::GetId() const { return m_id; }

int ADSBSVReport::GetNacp() const { return m_nacp; }

int ADSBSVReport::GetNacv() const { return m_nacv; }

int ADSBSVReport::GetNicp() const { return m_nicp; }

int ADSBSVReport::GetNicv() const { return m_nicv; }

Units::SecondsTime ADSBSVReport::GetTime() const { return m_time; }

Units::SecondsTime ADSBSVReport::GetTimeQuantum() const { return m_time_quantum; }

Units::FeetLength ADSBSVReport::GetVerticalPositionQuantum() const { return m_vertical_position_quantum; }

Units::FeetPerSecondSpeed ADSBSVReport::GetVerticalVelocityQuantum() const { return m_vertical_velocity_quantum; }

Units::FeetLength ADSBSVReport::GetX() const { return m_x; }

Units::FeetPerSecondSpeed ADSBSVReport::GetXd() const { return m_xd; }

Units::FeetLength ADSBSVReport::GetY() const { return m_y; }

Units::FeetPerSecondSpeed ADSBSVReport::GetYd() const { return m_yd; }

Units::FeetLength ADSBSVReport::GetZ() const { return m_z; }

Units::FeetPerSecondSpeed ADSBSVReport::GetZd() const { return m_zd; }

void ADSBSVReport::SetNacp(int nacp) { m_nacp = nacp; }

void ADSBSVReport::SetNacv(int nacv) { m_nacv = nacv; }

void ADSBSVReport::SetNicp(int nicp) { m_nicp = nicp; }

void ADSBSVReport::SetNicv(int nicv) { nicv = nicv; }

void ADSBSVReport::SetPosition(const Units::Length x, const Units::Length y, const Units::Length z,
                               const Units::Length horizontalQuantum, const Units::Length verticalQuantum) {
   m_horizontal_position_quantum = horizontalQuantum;
   m_x = quantize(x, m_horizontal_position_quantum);
   m_y = quantize(y, m_horizontal_position_quantum);
   m_vertical_position_quantum = verticalQuantum;
   m_z = quantize(z, m_vertical_position_quantum);
}

void ADSBSVReport::SetVelocity(const Units::Speed xd, const Units::Speed yd, const Units::Speed zd,
                               const Units::Speed horizontalQuantum, const Units::Speed verticalQuantum) {
   m_horizontal_velocity_quantum = horizontalQuantum;
   m_xd = quantize(xd, m_horizontal_velocity_quantum);
   m_yd = quantize(yd, m_horizontal_velocity_quantum);
   m_vertical_velocity_quantum = verticalQuantum;
   m_zd = quantize(zd, m_vertical_velocity_quantum);
}
}  // namespace ADSB
}  // namespace Sensor
