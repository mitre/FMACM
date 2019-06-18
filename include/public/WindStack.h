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

#pragma once

#include "Length.h"
#include "Speed.h"
#include "math/UVector.h"

class WindStack
{
public:
   WindStack();

   WindStack(const int min,
             const int max);

   virtual ~WindStack();

   bool operator==(const WindStack &obj) const;

   bool operator!=(const WindStack &obj) const;

   UVector<Units::FeetLength> GetAltitudeVector();

   UVector<Units::KnotsSpeed> GetSpeedVector();

   Units::FeetLength GetAltitude(const int index) const;

   Units::KnotsSpeed GetSpeed(const int index) const;

   int GetMinRow() const;

   int GetMaxRow() const;

   void SetBounds(const int min,
                  const int max);

   void Set(const int index,
            const Units::Length altitude,
            const Units::Speed speed);

   void AscendSort();

private:
   UVector<Units::FeetLength> m_altitude;
   UVector<Units::KnotsSpeed> m_speed;
};

inline UVector<Units::FeetLength> WindStack::GetAltitudeVector() {
   return m_altitude;
}

inline UVector<Units::KnotsSpeed> WindStack::GetSpeedVector() {
   return m_speed;
}

inline int WindStack::GetMinRow() const {
   return m_altitude.get_min();
}

inline int WindStack::GetMaxRow() const {
   return m_altitude.get_max();
}