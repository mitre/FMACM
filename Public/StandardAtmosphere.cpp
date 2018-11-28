#include "public/StandardAtmosphere.h"

StandardAtmosphere::StandardAtmosphere(const Units::Temperature temperatureOffset)
      :
      mTemperatureOffset(temperatureOffset) {
}

StandardAtmosphere::~StandardAtmosphere() {
   // nothing to do
}

Units::Temperature StandardAtmosphere::getTemperatureOffset() const {
   return mTemperatureOffset;
}

/**
 * getTemp calculates the temperature at a given altitude.
 *
 * Note:  Only the Troposphere and Tropopause layers are implemented.
 * Calculations above 65,000 feet would require adding Stratosphere.
 */
Units::KelvinTemperature StandardAtmosphere::getTemp(const Units::Length h) const {
   Units::KelvinTemperature T;
   if (h < H_TROP) {
      T = T0 + mTemperatureOffset - Units::KelvinPerMeter(6.5 / 1000) * h;
   } else {
      T = T_TROP + mTemperatureOffset;
   }

   return T;
}
