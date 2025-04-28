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
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <memory>
#include "public/Atmosphere.h"
#include "public/WindStack.h"
#include "public/Logging.h"

class Wind;

namespace aaesim {
namespace open_source {
class WeatherEstimate {
   friend class Wind_populate_predicted_wind_matrices_Test;

   friend class TrajectoryPredictor_updateWeatherPrediction_Test;

   friend class TrajectoryPredictor_startAltitudeInDescentAltList_Test;

   friend class TrajectoryPredictor_startAndEndAltitudeInDescentAltList_Test;

   struct shared_members {
      shared_members() : east_west(), north_south(), m_atmosphere(nullptr) {}
      shared_members(std::shared_ptr<Atmosphere> atmosphere) : east_west(), north_south(), m_atmosphere(atmosphere) {}
      aaesim::open_source::WindStack east_west;
      aaesim::open_source::WindStack north_south;
      std::shared_ptr<Atmosphere> m_atmosphere;
   };

  protected:
   std::shared_ptr<shared_members> m_shared_members;

  private:
   inline static log4cplus::Logger m_logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("WeatherEstimate"))};

  public:
   aaesim::open_source::WindStack &east_west() const { return m_shared_members->east_west; }
   aaesim::open_source::WindStack &north_south() const { return m_shared_members->north_south; }

   std::shared_ptr<Wind> getWind() const;

   std::shared_ptr<Atmosphere> getAtmosphere() const;

   virtual void LoadConditionsAt(const Units::Angle latitude, const Units::Angle longitude,
                                 const Units::Length altitude);
   Units::Density GetDensity() const;
   Units::Pressure GetPressure() const;
   Units::KelvinTemperature GetTemperature() const;

   Units::Speed MachToTAS(const double mach, const Units::Length altitude) const;
   Units::Speed MachToCAS(const double mach, const Units::Length altitude) const;
   Units::Speed TAS2CAS(const Units::Speed true_airspeed, const Units::Length altitude) const;
   Units::Speed CAS2TAS(const Units::Speed calibrated_airspeed, const Units::Length altitude) const;
   double CAS2Mach(const Units::Speed calibrated_airspeed, const Units::Length altitude) const;
   double TAS2Mach(const Units::Speed true_airspeed, const Units::Length altitude) const;
   double ESFconstantCAS(const Units::Speed true_airspeed, const Units::Length altitude) const;
   std::pair<std::pair<Units::Angle, Units::Angle>, Units::Length> GetCurrentLocationOfWeather() const;
   void SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere);

  protected:
   // Constructors are protected to prevent bare instantiation.
   // Callers should use WeatherPrediction or WeatherTruth.
   WeatherEstimate();

   WeatherEstimate(std::shared_ptr<Wind> wind, std::shared_ptr<Atmosphere> atmosphere);

   virtual ~WeatherEstimate();

   bool IsTemperatureAvailable(const Units::Angle latitude, const Units::Angle longitude,
                               const Units::Length altitude) const;

   void SetLocation(const Units::Angle latitude, const Units::Angle longitude, const Units::Length altitude);

   std::shared_ptr<Wind> m_wind{};
   Units::KelvinTemperature m_temperature{};
   Units::Pressure m_pressure{};
   Units::Density m_density{};
   mutable bool m_temperature_checked{true}, m_temperature_available{false};
   std::pair<std::pair<Units::Angle, Units::Angle>, Units::Length> m_location_of_current_conditions{};
};

inline void WeatherEstimate::SetAtmosphere(std::shared_ptr<Atmosphere> atmosphere) {
   m_shared_members->m_atmosphere = atmosphere;
}

inline void WeatherEstimate::SetLocation(const Units::Angle latitude, const Units::Angle longitude,
                                         const Units::Length altitude) {
   m_location_of_current_conditions.first = std::pair<Units::Angle, Units::Angle>(latitude, longitude);
   m_location_of_current_conditions.second = altitude;
}

inline std::pair<std::pair<Units::Angle, Units::Angle>, Units::Length> WeatherEstimate::GetCurrentLocationOfWeather()
      const {
   return m_location_of_current_conditions;
}

}  // namespace open_source
}  // namespace aaesim
