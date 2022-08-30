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

   static WindStack CreateZeroSpeedStack();

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
