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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once


#include <Angle.h>
#include <Force.h>
#include <Speed.h>

class ControlCommands
{
public:
   ControlCommands(const Units::Angle &phi,
                   const Units::Force &thrust,
                   const Units::Angle &gamma,
                   const Units::Speed &trueAirspeed,
                   const double speedBrake,
                   const int flapMode)
         : phi(phi),
           thrust(thrust),
           gamma(gamma),
           trueAirspeed(trueAirspeed),
           speedBrake(speedBrake),
           flapMode(flapMode) {
   }

   const Units::Angle &getPhi() const {
      return phi;
   }

   const Units::Force &getThrust() const {
      return thrust;
   }

   const Units::Angle &getGamma() const {
      return gamma;
   }

   const Units::Speed &getTrueAirspeed() const {
      return trueAirspeed;
   }


   double getSpeedBrake() const {
      return speedBrake;
   }

   int getFlapMode() const {
      return flapMode;
   }

private:
   Units::Angle phi;
   Units::Force thrust;
   Units::Angle gamma;
   Units::Speed trueAirspeed;
   double speedBrake;
   int flapMode;
};


