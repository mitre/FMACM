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

#include <vector>
#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Frequency.h>

namespace aaesim {
namespace open_source {
class WindStack {
  public:
   WindStack();

   WindStack(const WindStack &in);

   WindStack(const int min, const int max);

   ~WindStack() = default;

   WindStack &operator=(const WindStack &in);

   bool operator==(const WindStack &obj) const;

   bool operator!=(const WindStack &obj) const;

   Units::FeetLength GetAltitude(const int index) const;

   Units::KnotsSpeed GetSpeed(const int index) const;

   int GetMinRow() const;

   int GetMaxRow() const;

   void SetBounds(const int min, const int max);

   void Insert(const int index, const Units::Length altitude, const Units::Speed speed);

   void SortAltitudesAscending();

   void CalculateWindGradientAtAltitude(const Units::Length altitude_in, Units::Speed &wind_speed,
                                        Units::Frequency &wind_gradient) const;

   static WindStack CreateZeroSpeedStack();

  private:
   static bool AltitudeComparator(std::pair<Units::Length, Units::Speed> item1,
                                  std::pair<Units::Length, Units::Speed> item2);
   void Copy(const WindStack &in);
   std::vector<Units::Length> m_altitude;
   std::vector<Units::Speed> m_speed;
   int m_minimum_data_index, m_maximum_data_index;
};

inline int WindStack::GetMinRow() const { return m_minimum_data_index; }

inline int WindStack::GetMaxRow() const { return m_maximum_data_index; }

inline bool WindStack::AltitudeComparator(std::pair<Units::Length, Units::Speed> item1,
                                          std::pair<Units::Length, Units::Speed> item2) {
   return item1.first < item2.first;
}
}  // namespace open_source
}  // namespace aaesim
