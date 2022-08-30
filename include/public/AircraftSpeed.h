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

#include <scalar/Speed.h>

enum speed_type {
   UNSPECIFIED_SPEED,
   INDICATED_AIR_SPEED,
   MACH_SPEED,
   //TRUE_AIR_SPEED,   // not used in AAESim
   //GROUND_SPEED,     // not used in AAESim
};

/*
 * This class holds a speed type and value.
 * For types other than MACH, the unit is assumed to be knots.
 */
class AircraftSpeed {
public:
   AircraftSpeed();
   AircraftSpeed(const speed_type type, const double value);
   AircraftSpeed(const speed_type type, const Units::Speed value);
   virtual ~AircraftSpeed();
   void SetSpeed(const speed_type type, const double value);
   speed_type GetSpeedType() const;
   double GetValue() const;

private:
   speed_type m_speed_type;
   double m_value;
};

