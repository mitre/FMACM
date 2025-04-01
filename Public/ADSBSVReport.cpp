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

#include "public/ADSBSVReport.h"
#include "public/CustomMath.h"

namespace aaesim::open_source {
const ADSBSVReport ADSBSVReport::EMPTY_REPORT{};

bool ADSBSVReport::operator==(const ADSBSVReport &in) {
   if (m_id != in.m_id || m_time != in.m_time || m_has_position != in.m_has_position ||
       m_has_velocity != in.m_has_velocity) {
      return false;
   }

   if (m_has_position) {
      if (m_enu_x != in.m_enu_x || m_enu_y != in.m_enu_y || m_altitude_msl != in.m_altitude_msl ||
          m_nacp != in.m_nacp || m_nicp != in.m_nicp ||
          m_horizontal_position_quantum != in.m_horizontal_position_quantum ||
          m_vertical_position_quantum != in.m_vertical_position_quantum) {
         return false;
      }
   }

   if (this->m_has_velocity) {
      if (m_enu_xd != in.m_enu_xd || m_enu_yd != in.m_enu_yd || m_altitude_rate != in.m_altitude_rate ||
          m_nacv != in.m_nacv || m_nicv != in.m_nicv ||
          m_horizontal_velocity_quantum != in.m_horizontal_velocity_quantum ||
          m_vertical_velocity_quantum != in.m_vertical_velocity_quantum) {
         return false;
      }
   }

   return true;
}

ADSBSVReport::ADSBSVReport(const Builder &builder) {
   m_id = builder.GetUniqueId();
   m_time = builder.GetTimestamp();
   m_nacp = builder.GetNACp();
   m_nacv = builder.GetNACv();
   m_nicp = builder.GetNICp();
   m_nicv = builder.GetNICv();
   m_horizontal_position_quantum = builder.GetHorizontalPositionQuantum();
   m_vertical_position_quantum = builder.GetVerticalPositionQuantum();
   m_horizontal_velocity_quantum = builder.GetHorizontalVelocityQuantum();
   m_vertical_velocity_quantum = builder.GetVerticalVelocityQuantum();
   if (builder.HasPosition()) {
      m_has_position = true;
      m_enu_x = quantize(builder.GetPositionEnuX(), m_horizontal_position_quantum);
      m_enu_y = quantize(builder.GetPositionEnuY(), m_horizontal_position_quantum);
      m_latitude = builder.GetLatitude();
      m_longitude = builder.GetLongitude();
      m_altitude_msl = quantize(builder.GetAltitudeMsl(), m_vertical_position_quantum);
   }
   if (builder.HasVelocity()) {
      m_has_velocity = true;
      m_enu_xd = quantize(builder.GetGroundSpeedEnuXd(), m_horizontal_velocity_quantum);
      m_enu_yd = quantize(builder.GetGroundSpeedEnuYd(), m_horizontal_velocity_quantum);
      m_altitude_rate = quantize(builder.GetAltitudeRate(), m_vertical_velocity_quantum);
   }
}

ADSBSVReport::Builder::Builder(int unique_acid, Units::Time timestamp) {
   id_ = unique_acid;
   timestamp_ = timestamp;
}

ADSBSVReport ADSBSVReport::Builder::Build() { return ADSBSVReport{*this}; }

ADSBSVReport::Builder *ADSBSVReport::Builder::NACp(int nacp) {
   nacp_ = nacp;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::NACv(int nacv) {
   nacv_ = nacv;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::NICp(int nicp) {
   nicp_ = nicp;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::NICv(int nicv) {
   nicv_ = nicv;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::Position(Units::FeetLength enu_x, Units::FeetLength enu_y) {
   position_x_ = enu_x;
   position_y_ = enu_y;
   has_position_ = true;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::GeodeticPosition(Units::Angle latitude, Units::Angle longitude) {
   latitude_ = latitude;
   longitude_ = longitude;
   has_position_ = true;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::AltitudeMsl(Units::FeetLength altitude_msl) {
   altitude_msl_ = altitude_msl;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::GroundSpeed(Units::FeetPerSecondSpeed enu_xd,
                                                          Units::FeetPerSecondSpeed enu_yd) {
   xd_ = enu_xd;
   yd_ = enu_yd;
   has_velocity_ = true;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::AltitudeRate(Units::FeetPerSecondSpeed altitude_rate) {
   altitude_rate_ = altitude_rate;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::HorizontalPositionQuantum(Units::Length quantum) {
   horizontal_position_quantum_ = quantum;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::VerticalPositionQuantum(Units::Length quantum) {
   vertical_position_quantum_ = quantum;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::HorizontalVelocityQuantum(Units::Speed quantum) {
   horizontal_velocity_quantum_ = quantum;
   return this;
}

ADSBSVReport::Builder *ADSBSVReport::Builder::VerticalVelocityQuantum(Units::Speed quantum) {
   vertical_velocity_quantum_ = quantum;
   return this;
}

}  // namespace aaesim::open_source
