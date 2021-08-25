#include <stdexcept>
#include "public/AircraftSpeed.h"

AircraftSpeed::AircraftSpeed() {
   SetSpeed(UNSPECIFIED_SPEED, 0);
}

AircraftSpeed::AircraftSpeed(const speed_type type, const double value) {
   SetSpeed(type, value);
}

/** Convenience constructor for using Units::Speed IAS, stores speed as knots */
AircraftSpeed::AircraftSpeed(const speed_type type, const Units::Speed speed) {
   if (type == MACH_SPEED) {
      throw std::logic_error("Cannot construct MACH_SPEED AircraftSpeed using traditional speed units.");
   }
   SetSpeed(type, Units::KnotsSpeed(speed).value());
}

AircraftSpeed::~AircraftSpeed() {
}

speed_type AircraftSpeed::GetSpeedType() const {
   return m_speed_type;
}

void AircraftSpeed::SetSpeed(const speed_type type, const double value) {
   m_speed_type = type;
   m_value = value;
}

double AircraftSpeed::GetValue() const {
   return m_value;
}
