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

#pragma once

#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Time.h>
#include <scalar/Angle.h>

namespace aaesim::open_source {
class ADSBSVReport final {
  public:
   ADSBSVReport() = default;
   ~ADSBSVReport() = default;
   static const ADSBSVReport EMPTY_REPORT;

   bool operator==(const ADSBSVReport &in);

   bool HasPosition() const;
   bool HasVelocity() const;
   int GetId() const;
   int GetNacp() const;
   int GetNacv() const;
   int GetNicp() const;
   int GetNicv() const;
   Units::SecondsTime GetTime() const;
   Units::FeetLength GetX() const;
   Units::FeetPerSecondSpeed GetXd() const;
   Units::FeetLength GetY() const;
   Units::FeetPerSecondSpeed GetYd() const;
   Units::FeetLength GetAltitudeMsl() const;
   Units::FeetPerSecondSpeed GetVerticalSpeed() const;
   Units::Angle GetLatitude() const;
   Units::Angle GetLongitude() const;

   class Builder {
     private:
      int id_{-1};
      Units::Time timestamp_{Units::SecondsTime(-1)};
      int nacp_{0}, nacv_{0}, nicp_{0}, nicv_{0};
      bool has_position_{false}, has_velocity_{false};
      Units::FeetLength position_x_{Units::zero()}, position_y_{Units::zero()}, altitude_msl_{Units::zero()};
      Units::FeetPerSecondSpeed xd_{Units::zero()}, yd_{Units::zero()}, altitude_rate_{Units::zero()};
      Units::Length horizontal_position_quantum_{Units::zero()}, vertical_position_quantum_{Units::zero()};
      Units::Speed horizontal_velocity_quantum_{Units::zero()}, vertical_velocity_quantum_{Units::zero()};
      Units::Angle latitude_{Units::zero()}, longitude_{Units::zero()};

     public:
      Builder(int unique_acid, Units::Time timestamp);
      ~Builder() = default;
      ADSBSVReport Build();
      Builder *NACp(int nacp);
      Builder *NACv(int nacv);
      Builder *NICp(int nicp);
      Builder *NICv(int nicv);
      Builder *Position(Units::FeetLength enu_x, Units::FeetLength enu_y);
      Builder *GeodeticPosition(Units::Angle latitude, Units::Angle longitude);
      Builder *AltitudeMsl(Units::FeetLength altitude_msl);
      Builder *GroundSpeed(Units::FeetPerSecondSpeed enu_xd, Units::FeetPerSecondSpeed enu_yd);
      Builder *AltitudeRate(Units::FeetPerSecondSpeed altitude_rate);
      Builder *HorizontalPositionQuantum(Units::Length quantum);
      Builder *VerticalPositionQuantum(Units::Length quantum);
      Builder *HorizontalVelocityQuantum(Units::Speed quantum);
      Builder *VerticalVelocityQuantum(Units::Speed quantum);
      int GetNACp() const { return nacp_; }
      int GetNACv() const { return nacv_; }
      int GetNICp() const { return nicp_; }
      int GetNICv() const { return nicv_; }
      int GetUniqueId() const { return id_; }
      Units::Time GetTimestamp() const { return timestamp_; }
      Units::Length GetPositionEnuX() const { return position_x_; }
      Units::Length GetPositionEnuY() const { return position_y_; }
      Units::Length GetAltitudeMsl() const { return altitude_msl_; }
      Units::Speed GetGroundSpeedEnuXd() const { return xd_; }
      Units::Speed GetGroundSpeedEnuYd() const { return yd_; }
      Units::Speed GetAltitudeRate() const { return altitude_rate_; }
      Units::Length GetHorizontalPositionQuantum() const { return horizontal_position_quantum_; }
      Units::Length GetVerticalPositionQuantum() const { return vertical_position_quantum_; }
      Units::Speed GetHorizontalVelocityQuantum() const { return horizontal_velocity_quantum_; }
      Units::Speed GetVerticalVelocityQuantum() const { return vertical_velocity_quantum_; }
      bool HasPosition() const { return has_position_; }
      bool HasVelocity() const { return has_velocity_; }
      Units::Angle GetLatitude() const { return latitude_; }
      Units::Angle GetLongitude() const { return longitude_; }
   };

  private:
   ADSBSVReport(const Builder &builder);

   int m_id{-1};
   Units::SecondsTime m_time{Units::SecondsTime(-1)};
   int m_nacp{0}, m_nacv{0}, m_nicp{0}, m_nicv{0};
   bool m_has_position{false}, m_has_velocity{false};
   Units::FeetLength m_enu_x{Units::zero()}, m_enu_y{Units::zero()}, m_altitude_msl{Units::zero()};
   Units::Angle m_latitude{Units::zero()}, m_longitude{Units::zero()};
   Units::FeetPerSecondSpeed m_enu_xd{Units::zero()}, m_enu_yd{Units::zero()}, m_altitude_rate{Units::zero()};
   Units::FeetLength m_horizontal_position_quantum{Units::zero()}, m_vertical_position_quantum{Units::zero()};
   Units::FeetPerSecondSpeed m_horizontal_velocity_quantum{Units::zero()}, m_vertical_velocity_quantum{Units::zero()};
};

inline bool ADSBSVReport::HasPosition() const { return m_has_position; }
inline bool ADSBSVReport::HasVelocity() const { return m_has_velocity; }
inline int ADSBSVReport::GetId() const { return m_id; }
inline int ADSBSVReport::GetNacp() const { return m_nacp; }
inline int ADSBSVReport::GetNacv() const { return m_nacv; }
inline int ADSBSVReport::GetNicp() const { return m_nicp; }
inline int ADSBSVReport::GetNicv() const { return m_nicv; }
inline Units::SecondsTime ADSBSVReport::GetTime() const { return m_time; }
inline Units::FeetLength ADSBSVReport::GetX() const { return m_enu_x; }
inline Units::FeetPerSecondSpeed ADSBSVReport::GetXd() const { return m_enu_xd; }
inline Units::FeetLength ADSBSVReport::GetY() const { return m_enu_y; }
inline Units::FeetPerSecondSpeed ADSBSVReport::GetYd() const { return m_enu_yd; }
inline Units::FeetLength ADSBSVReport::GetAltitudeMsl() const { return m_altitude_msl; }
inline Units::FeetPerSecondSpeed ADSBSVReport::GetVerticalSpeed() const { return m_altitude_rate; }
inline Units::Angle ADSBSVReport::GetLatitude() const { return m_latitude; }
inline Units::Angle ADSBSVReport::GetLongitude() const { return m_longitude; }

}  // namespace aaesim::open_source
