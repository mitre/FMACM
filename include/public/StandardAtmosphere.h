/**
 * StandardAtmosphere is an implementation of the ISA
 * (International Standard Atmosphere) concept, which
 * defines straightforward approximations for calculating
 * pressure, temperature, density, and viscosity as a
 * function of altitude.
 *
 * https://en.wikipedia.org/wiki/International_Standard_Atmosphere
 */

#pragma once

#include <public/Atmosphere.h>

class StandardAtmosphere : public Atmosphere
{
public:
   StandardAtmosphere(const Units::Temperature temperatureOffset);

   virtual ~StandardAtmosphere();

   Units::KelvinTemperature getTemp(const Units::Length h) const;

   Units::Temperature getTemperatureOffset() const;

private:
   const Units::Temperature mTemperatureOffset;
};

