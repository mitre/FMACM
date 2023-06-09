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

#pragma once

#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Time.h>

namespace Sensor {

namespace ADSB {

class ADSBSVReport {
  public:
   ADSBSVReport();

   ~ADSBSVReport();

   // Copy constructor and overload operator
   ADSBSVReport(const ADSBSVReport &in);

   ADSBSVReport &operator=(const ADSBSVReport &in);

   bool operator==(const ADSBSVReport &in);

   void Clear();

   bool IsHasPosition() const;

   void SetHasPosition(bool hasPosition);

   bool IsHasVelocity() const;

   void SetHasVelocity(bool hasVelocity);

   Units::FeetLength GetHorizontalPositionQuantum() const;

   Units::FeetPerSecondSpeed GetHorizontalVelocityQuantum() const;

   int GetId() const;

   int GetNacp() const;

   int GetNacv() const;

   int GetNicp() const;

   int GetNicv() const;

   Units::SecondsTime GetTime() const;

   Units::SecondsTime GetTimeQuantum() const;

   Units::FeetLength GetVerticalPositionQuantum() const;

   Units::FeetPerSecondSpeed GetVerticalVelocityQuantum() const;

   Units::FeetLength GetX() const;

   Units::FeetPerSecondSpeed GetXd() const;

   Units::FeetLength GetY() const;

   Units::FeetPerSecondSpeed GetYd() const;

   Units::FeetLength GetZ() const;

   Units::FeetPerSecondSpeed GetZd() const;

   void SetId(const int id);

   void SetTime(const Units::Time time);

   void SetPosition(const Units::Length x, const Units::Length y, const Units::Length z,
                    const Units::Length horizontalQuantum, const Units::Length verticalQuantum);

   void SetVelocity(const Units::Speed xd, const Units::Speed yd, const Units::Speed zd,
                    const Units::Speed horizontalQuantum, const Units::Speed verticalQuantum);

   void SetNacp(const int nacp);

   void SetNacv(const int nacv);

   void SetNicp(const int nicp);

   void SetNicv(const int nicv);

   static const ADSBSVReport blank_report;

   // Input Data:
   // none

   Units::FeetLength m_x, m_y, m_z;
   Units::FeetPerSecondSpeed m_xd, m_yd, m_zd;
   // Other Data:
  private:
   int m_id;
   Units::SecondsTime m_time;
   //         Units::FeetLength m_x, m_y, m_z;
   //         Units::FeetPerSecondSpeed m_xd, m_yd, m_zd;
   int m_nacp, m_nacv, m_nicp, m_nicv;
   bool m_has_position, m_has_velocity;
   Units::FeetLength m_horizontal_position_quantum, m_vertical_position_quantum;
   Units::FeetPerSecondSpeed m_horizontal_velocity_quantum, m_vertical_velocity_quantum;
   Units::SecondsTime m_time_quantum;
};

}  // namespace ADSB
}  // namespace Sensor
