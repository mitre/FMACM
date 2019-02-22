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

   //WindStack(const DMatrix &dMatrix);
   //void load(const DMatrix &dMatrix);
   UVector<Units::FeetLength> getAltitudeVector();

   UVector<Units::KnotsSpeed> getSpeedVector();

   Units::FeetLength getAltitude(const int index) const;

   Units::KnotsSpeed getSpeed(const int index) const;

   int get_min_row() const;

   int get_max_row() const;

   void setBounds(const int min,
                  const int max);

   void set(const int index,
            const Units::Length altitude,
            const Units::Speed speed);

   void ascendSort();

private:
   UVector<Units::FeetLength> altitude;
   UVector<Units::KnotsSpeed> speed;
};

