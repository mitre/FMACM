#include "public/StandardAtmosphere.h"

StandardAtmosphere::StandardAtmosphere(const Units::Temperature temperatureOffset)
      :
      m_temperature_offset(temperatureOffset) {
}

StandardAtmosphere::~StandardAtmosphere() {
   // nothing to do
}

Units::Temperature StandardAtmosphere::GetTemperatureOffset() const {
   return m_temperature_offset;
}

/**
 * getTemp calculates the temperature at a given altitude.
 *
 * Note:  Only the Troposphere and Tropopause layers are implemented.
 * Calculations above 65,000 feet would require adding Stratosphere.
 */
Units::KelvinTemperature StandardAtmosphere::GetTemp(const Units::Length h) const {
   Units::KelvinTemperature T;
   if (h < H_TROP) {
      T = T0 + m_temperature_offset - Units::KelvinPerMeter(6.5 / 1000) * h;
   } else {
      T = T_TROP + m_temperature_offset;
   }

   return T;
}
