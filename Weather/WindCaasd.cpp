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

#include "weather/WindCaasd.h"
#include <nlohmann/json.hpp>

using namespace std;
using namespace wind;
using namespace aaesim::weather;

log4cplus::Logger WindCaasd::m_logger = log4cplus::Logger::getInstance("WindCaasd");

WindCaasd::WindCaasd(const std::filesystem::path &grb_file_name) {
   m_height_index = m_grid_factory.includeVariable("gh");
   m_wind_query_variables[0] = m_grid_factory.includeVariable("u");
   m_wind_query_variables[1] = m_grid_factory.includeVariable("v");
   m_temp_query_variables[0] = m_grid_factory.includeVariable("t");
   m_pressure_query_variables[0] = m_grid_factory.includeVariable("pres");
   m_weather_grid = m_grid_factory.createGridFromGribFile(grb_file_name);
   m_interpolator = new LinearInterpolator3D(m_height_index, m_weather_grid);
   m_pressure_index = 0;
}

WindCaasd::~WindCaasd() { delete m_interpolator; }

void WindCaasd::InterpolateWind(Units::Angle latitude, Units::Angle longitude, Units::Length alt,
                                Units::Speed &east_west, Units::Speed &north_south) {

   const double lat_deg(Units::DegreesAngle(latitude).value());
   const double lon_deg(Units::DegreesAngle(longitude).value());
   const double alt_meters(Units::MetersLength(alt).value());

   double result[2];
   int status = m_interpolator->getInterpolatedValues(lat_deg, lon_deg, alt_meters, m_wind_query_variables, result, 2);
   if (status) {
      InterpolateWind_LogWindQueryError(lat_deg, lon_deg, alt_meters, status);
   }
   east_west = Units::MetersPerSecondSpeed(result[0]);
   north_south = Units::MetersPerSecondSpeed(result[1]);
}

void WindCaasd::InterpolateWind_LogWindQueryError(const double lat_deg, const double lon_deg, const double alt_meters,
                                                  const int status) {
   using json = nlohmann::json;

   if (m_logger.getLogLevel() <= log4cplus::ERROR_LOG_LEVEL) {
      json j, queryPoint;
      j["message"] = "Error in wind query";
      queryPoint["latitude"] = lat_deg;
      queryPoint["longitude"] = lon_deg;
      queryPoint["altitude_meters"] = alt_meters;
      j["query_point"] = queryPoint;
      j["status"] = status;
      LOG4CPLUS_ERROR(m_logger, j);
   }
}

void WindCaasd::InterpolateWindScalar(Units::Angle latitude, Units::Angle longitude, Units::Length altitude,
                                      Units::Speed &east_west, Units::Speed &north_south) {
   InterpolateWind(latitude, longitude, altitude, east_west, north_south);
}

void WindCaasd::InterpolateWindMatrix(Units::Angle latitude, Units::Angle longitude, Units::Length altitude,
                                      aaesim::open_source::WindStack &east_west,
                                      aaesim::open_source::WindStack &north_south) {

   // return closest 5 integer layer numbers
   const double lat_deg(Units::DegreesAngle(latitude).value());
   const double lon_deg(Units::DegreesAngle(longitude).value());
   const double ONE_THOUSAND_FEET(Units::MetersLength(Units::FeetLength(1000)).value());

   // set the WindStack bounds based on altitude
   int middle_thousand = round(altitude / Units::FeetLength(1000));
   middle_thousand = max(middle_thousand, 2);

   double result[3];

   east_west.SetBounds(middle_thousand - 1, middle_thousand + 3);
   north_south.SetBounds(middle_thousand - 1, middle_thousand + 3);

   for (int i = east_west.GetMinRow(); i <= east_west.GetMaxRow(); i++) {
      double alt_meters = (i - 1) * ONE_THOUSAND_FEET;
      int status =
            m_interpolator->getInterpolatedValues(lat_deg, lon_deg, alt_meters, m_wind_query_variables, result, 2);
      if (status) {
         InterpolateWindMatrix_LogWindQueryError(lat_deg, lon_deg, alt_meters, status);
      }
      east_west.Insert(i, Units::MetersLength(alt_meters), Units::MetersPerSecondSpeed(result[0]));
      north_south.Insert(i, Units::MetersLength(alt_meters), Units::MetersPerSecondSpeed(result[1]));
      LOG4CPLUS_TRACE(m_logger, "True wind at (" << Units::DegreesAngle(latitude) << ","
                                                 << Units::DegreesAngle(longitude) << "," << (i - 1) << ",000 ft) is ("
                                                 << result[0] << "," << result[1] << ")");
   }
}

void WindCaasd::InterpolateWindMatrix_LogWindQueryError(const double lat_deg, const double lon_deg,
                                                        const double alt_meters, const int status) {
   using json = nlohmann::json;

   if (m_logger.getLogLevel() <= log4cplus::ERROR_LOG_LEVEL) {
      json j, queryPoint;
      j["message"] = "Error in wind query";
      queryPoint["latitude"] = lat_deg;
      queryPoint["longitude"] = lon_deg;
      queryPoint["altitude_meters"] = alt_meters;
      j["query_point"] = queryPoint;
      j["status"] = status;
      LOG4CPLUS_ERROR(m_logger, j);
   }
}

Units::KelvinTemperature WindCaasd::InterpolateTemperature(Units::Angle latitude, Units::Angle longitude,
                                                           Units::Length altitude) {

   const double lat_deg(Units::DegreesAngle(latitude).value());
   const double lon_deg(Units::DegreesAngle(longitude).value());
   const double alt_meters(Units::MetersLength(altitude).value());

   double result[1];
   int status = m_interpolator->getInterpolatedValues(lat_deg, lon_deg, alt_meters, m_temp_query_variables, result, 1);
   if (status) {
      InterpolateTemperature_LogWindQueryError(lat_deg, lon_deg, alt_meters, status);
   }
   Units::KelvinTemperature t(result[0]);

   return t;
}

void WindCaasd::InterpolateTemperature_LogWindQueryError(const double lat_deg, const double lon_deg,
                                                         const double alt_meters, const int status) {
   using json = nlohmann::json;

   if (m_logger.getLogLevel() <= log4cplus::ERROR_LOG_LEVEL) {
      json j, queryPoint;
      j["message"] = "Error in temperature query";
      queryPoint["latitude"] = lat_deg;
      queryPoint["longitude"] = lon_deg;
      queryPoint["altitude_meters"] = alt_meters;
      j["query_point"] = queryPoint;
      j["status"] = status;
      LOG4CPLUS_ERROR(m_logger, j);
   }
}

Units::Pressure WindCaasd::InterpolatePressure(Units::Angle latitude, Units::Angle longitude, Units::Length altitude) {

   const double lat_deg(Units::DegreesAngle(latitude).value());
   const double lon_deg(Units::DegreesAngle(longitude).value());
   const double alt_meters(Units::MetersLength(altitude).value());

   double result[1];
   int status =
         m_interpolator->getInterpolatedValues(lat_deg, lon_deg, alt_meters, m_pressure_query_variables, result, 1);
   if (status) {
      InterpolatePressure_LogWindQueryError(lat_deg, lon_deg, alt_meters, status);
   }
   Units::PascalsPressure pressure(result[0]);

   return pressure;
}

void WindCaasd::InterpolatePressure_LogWindQueryError(const double lat_deg, const double lon_deg,
                                                      const double alt_meters, const int status) {
   using json = nlohmann::json;

   if (m_logger.getLogLevel() <= log4cplus::ERROR_LOG_LEVEL) {
      json j, queryPoint;
      j["message"] = "Error in pressure query";
      queryPoint["latitude"] = lat_deg;
      queryPoint["longitude"] = lon_deg;
      queryPoint["altitude_meters"] = alt_meters;
      j["query_point"] = queryPoint;
      j["status"] = status;
      LOG4CPLUS_ERROR(m_logger, j);
   }
}
