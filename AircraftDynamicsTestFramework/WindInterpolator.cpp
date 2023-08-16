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

#include "framework/WindInterpolator.h"

using namespace fmacm;

Units::KelvinTemperature WindInterpolator::InterpolateTemperature(const Units::Angle latitude_in,
                                                                  const Units::Angle longitude_in,
                                                                  const Units::Length altitude) {
   /** Ignore parameters, return temp from set weather */
   return Units::KelvinTemperature(m_weather_data_point.temperature.value());
}

Units::Pressure WindInterpolator::InterpolatePressure(const Units::Angle latitude_in, const Units::Angle longitude_in,
                                                      const Units::Length altitude) {
   /** Ignore parameters, return temp from set weather */
   throw std::logic_error("WindInterpolator::InterpolatePressure is not implemented.");
   return Units::MillibarPressure(0);
}

void WindInterpolator::InterpolateWind(Units::Angle latitude_in, Units::Angle longitude_in, Units::Length altitude,
                                       Units::Speed &u, Units::Speed &v) {
   /** Ignore parameters, return wind components from set weather */
   u = m_weather_data_point.Vwx;
   v = m_weather_data_point.Vwy;
}

void WindInterpolator::InterpolateWindScalar(Units::Angle lat_in, Units::Angle lon_in, Units::Length altitude,
                                             Units::Speed &east_west, Units::Speed &north_south) {
   /** Ignore parameters, return wind components from set weather */
   east_west = m_weather_data_point.Vwx;
   north_south = m_weather_data_point.Vwy;
}

void WindInterpolator::InterpolateWindMatrix(Units::Angle lat_in, Units::Angle lon_in, Units::Length alt_in,
                                             aaesim::open_source::WindStack &east_west,
                                             aaesim::open_source::WindStack &north_south) {
   /** Ignore lat/lon, return wind matrix from set weather, centered on alt_in */
   east_west.SetBounds(1, 5);
   north_south.SetBounds(1, 5);
   const Units::FeetLength alt_step(1);
   for (int i = 1; i <= 5; i++) {
      double offset = 3 - i;  // zero at center, descending
      Units::Length altitude = alt_in + offset * alt_step;
      Units::Speed u = m_weather_data_point.Vwx + offset * alt_step * -m_weather_data_point.dVwx_dh;
      Units::Speed v = m_weather_data_point.Vwy + offset * alt_step * -m_weather_data_point.dVwy_dh;
      east_west.Insert(i, altitude, u);
      north_south.Insert(i, altitude, v);
   }

   east_west.SortAltitudesAscending();
   north_south.SortAltitudesAscending();
}

void WindInterpolator::UpdateWindDataPoint(const fmacm::WindInterpolator::WeatherDataPoint &data_point) {
   m_weather_data_point = data_point;
}
