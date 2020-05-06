#pragma once

#include <Speed.h>

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

