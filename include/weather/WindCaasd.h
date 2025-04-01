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
#include <filesystem>

#include "public/Wind.h"
#include "GridFactory.h"
#include "LinearInterpolator3D.h"
#include "WeatherGrid.h"

namespace aaesim::weather {
class WindCaasd final : public Wind {
  private:
   static log4cplus::Logger m_logger;
   wind::GridFactory m_grid_factory;
   int m_pressure_index, m_height_index;
   int m_wind_query_variables[2];
   int m_temp_query_variables[1];
   int m_pressure_query_variables[1];
   wind::LinearInterpolator3D *m_interpolator;  // NOT threadsafe!
   // If this application is ever changed to have multiple threads
   // accessing the same WindCaasd object, it will need to provide
   // a separate m_interpolator object for each thread.
   // They are relatively lightweight, so creating a new one for
   // each query is feasible. -- klewis
   std::shared_ptr<wind::WeatherGrid> m_weather_grid;

  public:
   WindCaasd(const std::filesystem::path &grb_file_name);

   virtual ~WindCaasd();

   // implementation methods
   void InterpolateWind(Units::Angle latitude, Units::Angle longitude, Units::Length alt, Units::Speed &east_west,
                        Units::Speed &north_south) override;

   void InterpolateWindScalar(Units::Angle latitude, Units::Angle longitude, Units::Length altitude,
                              Units::Speed &east_west, Units::Speed &north_south) override;

   void InterpolateWindMatrix(Units::Angle latitude, Units::Angle longitude, Units::Length altitude,
                              aaesim::open_source::WindStack &east_west,
                              aaesim::open_source::WindStack &north_south) override;

   Units::KelvinTemperature InterpolateTemperature(Units::Angle latitude, Units::Angle longitude,
                                                   Units::Length altitude) override;

   Units::Pressure InterpolatePressure(Units::Angle latitude, Units::Angle longitude, Units::Length altitude) override;

  private:
   // Logging methods
   void InterpolateWind_LogWindQueryError(const double lat_deg, const double lon_deg, const double alt_meters,
                                          const int status);
   void InterpolateWindMatrix_LogWindQueryError(const double lat_deg, const double lon_deg, const double alt_meters,
                                                const int status);
   void InterpolateTemperature_LogWindQueryError(const double lat_deg, const double lon_deg, const double alt_meters,
                                                 const int status);
   void InterpolatePressure_LogWindQueryError(const double lat_deg, const double lon_deg, const double alt_meters,
                                              const int status);
};
}  // namespace aaesim::weather
