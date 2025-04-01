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

#include "weather/WindLegacy.h"
#include <cstring>

using namespace std;
using namespace aaesim::weather;

// needed for WindSpeedUtils friend class
namespace aaesim {
namespace test {
namespace utils {
class WindSpeedUtils;
}
}  // namespace test
}  // namespace aaesim

log4cplus::Logger WindLegacy::m_logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("WindLegacy"));

void WindLegacy::readRUCWindFile(const string &file_name) {

   // Reads and stores RUC wind data.
   //
   // fType:file type being read-
   //       FORECAST_FILE
   //       TRUTH_FILE

   int k1;
   int k2;
   int k3;

   int lct;

   int dx1;
   int dx2;
   int dx3;

   double latin;
   double lgin;
   double tempin;
   double uin;
   double vin;
   double mbin;
   char linearr[300];

   FILE *fru1;

   if ((fru1 = fopen(file_name.c_str(), "r")) == NULL) {
      LOG4CPLUS_ERROR(m_logger, "Unable to open RUC wind file " << file_name);
      return;
   }

   fgets(linearr, 200, fru1);
   fgets(linearr, 200, fru1);

   /* FLIGHT LEVEL*/

   for (k1 = static_cast<int>(MINIMUM_ALTITUDE_LIMIT.value()); k1 <= MAXIMUM_ALTITUDE_LIMIT.value(); k1++) {
      fgets(linearr, 200, fru1);
      fgets(linearr, 200, fru1);

      lct = 0;

      /* 40 latitudes 18 to 57	*/
      dx3 = 0;

      for (k3 = 1; k3 <= NUM_LAT_GRID_POINTS; k3++) {
         dx3++;

         /*	70 longitude	-129 to -60 	*/
         dx2 = 0;

         for (k2 = 1; k2 <= NUM_LON_GRID_POINTS; k2++) {
            fgets(linearr, 200, fru1);
            sscanf(linearr, "%lf,%lf,%lf,%lf,%lf,%lf", &lgin, &latin, &tempin, &uin, &vin, &mbin);
#ifdef WIND_FLOAT
            // altin = (float) altin;
            latin = (float)latin;
            lgin = (float)lgin;
            tempin = (float)tempin;
            uin = (float)uin;
            vin = (float)vin;
            mbin = (float)mbin;
#endif
            /* cut the grid	flight on flight level 	*/

            if ((k1 >= MINIMUM_ALTITUDE_LIMIT.value()) && (k1 <= MAXIMUM_ALTITUDE_LIMIT.value())) {
               /* cut the grid	flight on longitude 	*/

               if ((lgin >= ULHLG) && (lgin <= LRHLG)) {
                  lct++;
                  // dx1 = k1-24;
                  dx1 = k1;
                  dx2++;

                  m_wind_data[dx1].wu[dx3][dx2] = Units::MetersPerSecondSpeed(uin);
                  m_wind_data[dx1].wv[dx3][dx2] = Units::MetersPerSecondSpeed(vin);
                  m_wind_data[dx1].temp[dx3][dx2] = Units::KelvinTemperature(tempin);
               }
            }
         }
      } /* k3 rows	*/
   }

   fclose(fru1);

}  // readRUCWindFile

void WindLegacy::readRAPWindFile(const string &file_name) {

   // Reads and stores RAP wind data.
   //
   // fType:file type being read-
   //       FORECAST_FILE
   //       TRUTH_FILE

   int numAlt = NUMALT;  // number of altitudes
   int numLat = NUMLAT;  // number of latitudes
   int numLon = NUMLON;  // number of longitudes

   double altin;
   double latin;
   double lgin;
   double tempin;
   double uin;
   double vin;
   double mbin;

   char linearr[300];

   FILE *fru1;

   if ((fru1 = fopen(file_name.c_str(), "r")) == NULL) {
      LOG4CPLUS_ERROR(m_logger, "Unable to open RAP wind file " << file_name);
      return;
   }

   // read the first 3 header lines
   fgets(linearr, 200, fru1);
   fgets(linearr, 200, fru1);
   fgets(linearr, 200, fru1);

   // read the content:

   for (int i = 0; i < numAlt; i++) {
      for (int j = 0; j < numLat; j++) {
         for (int k = 0; k < numLon; k++) {
            // read alt(i), lat(j), lon(k), ...;
            fgets(linearr, 200, fru1);
            sscanf(linearr, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &altin, &latin, &lgin, &tempin, &uin, &vin, &mbin);
#ifdef WIND_FLOAT
            altin = (float)altin;
            latin = (float)latin;
            lgin = (float)lgin;
            tempin = (float)tempin;
            uin = (float)uin;
            vin = (float)vin;
            mbin = (float)mbin;
#endif
            m_wind_data[i].alt[j][k] = Units::FeetLength(altin);
            m_wind_data[i].lat[j][k] = Units::DegreesAngle(latin);
            m_wind_data[i].lon[j][k] = Units::DegreesAngle(lgin);
            /*	  Units::Speed u_true_north,v_true_north;
                    WindRotation(m_wind_data[i].lon[j][k],
                              Units::MetersPerSecondSpeed(uin),
                              Units::MetersPerSecondSpeed(vin),
                              u_true_north,v_true_north);
                    m_wind_data[i].wu[j][k] = u_true_north;
                    m_wind_data[i].wv[j][k] = v_true_north;*/
            // data from Atmospheric Toolbox is in knots and already rotated.
            m_wind_data[i].wu[j][k] = Units::KnotsSpeed(uin);
            m_wind_data[i].wv[j][k] = Units::KnotsSpeed(vin);

         }  // for (int k=0;k<numLon;k++)
      }     // for (int j=0;j<numLat;j++)
   }        // for (int i=0;i<numAlt;i++)

   fclose(fru1);

}  // readRAPWindFile

void WindLegacy::LambertProjectionUsgs(const Units::Angle latitude_in, const Units::Angle longitude_in,
                                       Units::Length &x_out, Units::Length &y_out) {
   /*use Lambert conformal projection to convert latitude_in, longitude_in into x_out and y_out
     The equations are from USGS Map Projection Manual
     Input:
     Units::Angle latitude_in, Units::Angle longitude_in: all in radians

     output:
     Units::Length& x_out, Units::Length& y_out: all in meters

     USAGE:
   */

   // constants: put here for now for testing. From Table 1 of Reference [10]

   Units::MetersLength a(6378206.4);  // The Earth's semi-major axis (m)
   double e = 0.0822719;
   // double e_square = 0.00676866;

   // origins

   Units::DegreesAngle phi0(23.);
   Units::DegreesAngle lambda0(-96.);

   // standard parallels

   Units::DegreesAngle phi1(20.);
   Units::DegreesAngle phi2(30.);

   // intermediate variables

   Units::UnsignedAngle phi_in = latitude_in;
   Units::Angle lambda_in = longitude_in + Units::PI_RADIANS_ANGLE * 2;

   Units::Angle phi[4];  // phi[0] = phi0, phi[1] = phi1, phi[2] = phi2, phi[3] = phi_in
   phi[0] = phi0;
   phi[1] = phi1;
   phi[2] = phi2;
   phi[3] = phi_in;

   // calculate t0, t1, t2, and t:

   double t[4];  // t0 = t[0], t1 = t[1], t2 = t[2], t = t[3] (corresponding to the order in the phi array)
   for (int i = 0; i < 4; i++) {
      t[i] = CalculateTerm_t(phi[i], e);
   }

   // calculate m1, m2:

   double m[3];  // m[0] is unused. m[1] = m1, m[2] = m2
   for (int i = 1; i < 3; i++) {
      m[i] = CalculateTerm_m(phi[i], e);
   }

   // calculate n:

   double n;
   n = (log(m[1]) - log(m[2])) / (log(t[1]) - log(t[2]));  // The base log() is the Euler number--verified

   // calculate F:

   double F;
   F = m[1] / (n * pow(t[1], n));

   // calculate rho:

   Units::FeetLength rho;
   rho = a * F * pow(t[3], n);  // rho = aFt^n

   // calculate rho0:

   Units::FeetLength rho0;
   rho0 = a * F * pow(t[0], n);  // rho0 = aFT0^n

   // calculate theta:

   Units::SignedAngle delta;
   delta = lambda_in - lambda0;

   Units::Angle theta;
   theta = n * delta;

   // calculate x, y:

   x_out = rho * sin(theta);
   y_out = rho0 - rho * cos(theta);

}  // LambertProjectionUsgs

double WindLegacy::CalculateTerm_t(const Units::Angle phi, const double e) {

   // helper function for LambertProjectionUsgs

   double t = 0.;

   double sin_phi = sin(phi);

   t = sqrt((1 - sin_phi) / (1 + sin_phi) * pow((1 + e * sin_phi) / (1 - e * sin_phi), e));

   return t;
}  // CalculateTerm_t

double WindLegacy::CalculateTerm_m(const Units::Angle phi, const double e) {

   // helper function for LambertProjectionUsgs

   double m = 0.;

   double sin_phi = sin(phi);
   double cos_phi = cos(phi);

   m = cos_phi / sqrt(1 - pow(e, 2) * pow(sin_phi, 2));

   return m;
}  // static double CalculateTerm_m()

void WindLegacy::FindIndicesLambertProjectionUsgs(const Units::Angle latitude_in, const Units::Angle longitude_in,
                                                  double &i_out, double &j_out) {
   /*Input:
     Units::Angle latitude_in, Units::Angle longitude_in: all in radians

     output:
     double& i_out, double& j_out: grid index for the point (no unit)
   */

   // constants:

   Units::MetersLength dx(13512.3651);  // x_direction_grid_length (meters)
   Units::MetersLength dy(13512.3651);  // 13467.25351; // y_direction_grid_length (meters)

   // lower left corner pixel: lat, lon

   Units::DegreesAngle lat1(16.281), lon1(-126.138);

   // lower left corner pixel: Lambert

   Units::Length x1, y1;
   LambertProjectionUsgs(lat1, lon1, x1, y1);

   // calculate (x, y) of Lambert coordinate for (lat_in, lon_in):

   Units::Length x, y;
   LambertProjectionUsgs(latitude_in, longitude_in, x, y);

   // calculate the indices:

   i_out = (x - x1) / dx;
   j_out = (y - y1) / dy;

   // limit the indices within range: i in [0, 450], and j in [0, 336]

   if (i_out < 0.) {
      i_out = 0.;
   }
   if (i_out > 450.) {
      i_out = 450.;
   }
   if (j_out < 0.) {
      j_out = 0.;
   }
   if (j_out > 336.) {
      j_out = 336.;
   }
}  // FindIndicesLambertProjectionUsgs

void WindLegacy::FindLowerLeftCornerIndicesLambertProjectionUsgs(const Units::Angle latitude_in,
                                                                 const Units::Angle longitude_in, int &i_out,
                                                                 int &j_out) {

   /*Input:
     Units::Angle latitude_in, Units::Angle longitude_in: all in radians

     output:
     int& i_out, int& j_out: grid indices for the  lower-left corner point of the grid that the point falls in
   */

   double point_index_x, point_index_y;
   FindIndicesLambertProjectionUsgs(latitude_in, longitude_in, point_index_x, point_index_y);

   i_out = (int)floor(point_index_x);
   j_out = (int)floor(point_index_y);

}  // FindLowerLeftCornerIndicesLambertProjectionUsgs

void WindLegacy::InterpolateWindMatrix(const Units::Angle lat_in, const Units::Angle lon_in, const Units::Length alt_in,
                                       aaesim::open_source::WindStack &east_west,
                                       aaesim::open_source::WindStack &north_south) {

   /*
     Method: Interpolate wind at a (latitude, longitude) at different altitudes
     // lat_in: radians
     // lon_in: radians
     // x: feet
     // y: feet
     // The return values are east_west and north_south. Both are two-dimensional WindStack: the first column contains
     the altitude (ft),
     // and the second column contains the wind speed (knots).
     */

   // set the WindStack bounds based on altitude
   int middle_thousand = round(alt_in / Units::FeetLength(1000));
   middle_thousand = min(middle_thousand, WindLegacy::GetFlightLevelUpperBound() - 2);
   middle_thousand = max(middle_thousand, WindLegacy::GetFlightLevelLowerBound() + 2);
   east_west.SetBounds(middle_thousand - 1, middle_thousand + 3);
   north_south.SetBounds(middle_thousand - 1, middle_thousand + 3);

   if (!UseWind()) {
      for (int i = east_west.GetMinRow(); i <= east_west.GetMaxRow(); i++) {
         east_west.Insert(i, Units::FeetLength((i - 1) * 1000), Units::KnotsSpeed(0));
      }

      for (int i = north_south.GetMinRow(); i <= north_south.GetMaxRow(); i++) {
         north_south.Insert(i, Units::FeetLength((i - 1) * 1000), Units::KnotsSpeed(0));
      }

      return;
   }

   for (int i = east_west.GetMinRow(); i <= east_west.GetMaxRow(); i++) {
      Units::Length alt = Units::FeetLength((i - 1) * 1000);

      Units::Speed u, v;
      InterpolateWind(lat_in, lon_in, alt, u, v);

      east_west.Insert(i, alt, u);
      north_south.Insert(i, alt, v);
   }

}  // InterpolateWindMatrix

void WindLegacy::InterpolateWindScalar(const Units::Angle lat_in, const Units::Angle lon_in,
                                       const Units::Length altitude, Units::Speed &east_west,
                                       Units::Speed &north_south) {

   // Method: Interpolate wind at (x_in, y_in, altitude)
   // lat_in: radians
   // lon_in: radians
   // x: feet
   // y: feet
   // altitude: feet
   // The return values are *east_west and *north_south: wind speeds (in knots).

   if (!UseWind()) {
      throw logic_error("This class should not be used if use_wind is false.");
   }

   InterpolateWind(lat_in, lon_in, altitude, east_west, north_south);
}  // InterpolateWindScalar

void WindLegacy::InterpolateWind(const Units::Angle latitude_in, const Units::Angle longitude_in,
                                 const Units::Length alt, Units::Speed &u, Units::Speed &v) {

   /*
     Method: Interpolate the (u, v) wind at the position (latitude_in, longitude_in, alt)
     Input:
     struct WindData wind_array_input[]: the structure that contains the wind matrix
     Units::Angle latitude_in, Units::Angle longitude_in: aircraft position (all in radians)
     Units::Length alt:

     output:
     Units::Speed u: interpolated wind u (m/s)
     Units::Speed v: interpolated wind v (m/s)
   */

   int lower_left_x_index, lower_left_y_index;
   FindLowerLeftCornerIndicesLambertProjectionUsgs(latitude_in, longitude_in, lower_left_x_index, lower_left_y_index);

   // Lambert Projection of the Aircraft's Current Position
   Units::MetersLength x_aircraft, y_aircraft;
   LambertProjectionUsgs(latitude_in, longitude_in, x_aircraft, y_aircraft);

   // Lambert Projection of Grid Locations
   Units::MetersLength x_lower_left, y_lower_left;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index][lower_left_x_index],
                         m_wind_data[0].lon[lower_left_y_index][lower_left_x_index], x_lower_left, y_lower_left);

   Units::MetersLength x_lower_right, y_lower_right;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index][lower_left_x_index + 1],
                         m_wind_data[0].lon[lower_left_y_index][lower_left_x_index + 1], x_lower_right, y_lower_right);

   Units::MetersLength x_upper_left, y_upper_left;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index + 1][lower_left_x_index],
                         m_wind_data[0].lon[lower_left_y_index + 1][lower_left_x_index], x_upper_left, y_upper_left);

   Units::MetersLength x_upper_right, y_upper_right;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index + 1][lower_left_x_index + 1],
                         m_wind_data[0].lon[lower_left_y_index + 1][lower_left_x_index + 1], x_upper_right,
                         y_upper_right);

   int new_lower_left_x_index, new_lower_left_y_index;
   Units::MetersLength new_x_lower_left, new_y_lower_left, new_x_lower_right, new_y_lower_right, new_x_upper_left,
         new_y_upper_left, new_x_upper_right, new_y_upper_right;

   CheckBox(lower_left_x_index, lower_left_y_index, x_aircraft, y_aircraft, x_lower_left, y_lower_left, x_lower_right,
            y_lower_right, x_upper_left, y_upper_left, x_upper_right, y_upper_right, new_lower_left_x_index,
            new_lower_left_y_index, new_x_lower_left, new_y_lower_left, new_x_lower_right, new_y_lower_right,
            new_x_upper_left, new_y_upper_left, new_x_upper_right, new_y_upper_right);

   // do four vertical interpolations;then do horizontal linear interpolation
   // vertical interpolation at lower-left corner of the unit square
   Units::Speed u11, v11;
   VerticallyInterpolateWind(alt, new_lower_left_x_index, new_lower_left_y_index, u11, v11);

   // vertical interpolation at lower-right corner of one unit square
   Units::Speed u21, v21;
   VerticallyInterpolateWind(alt, new_lower_left_x_index + 1, new_lower_left_y_index, u21, v21);

   // vertical interpolation at upper-left corner of the unit square
   Units::Speed u12, v12;
   VerticallyInterpolateWind(alt, new_lower_left_x_index, new_lower_left_y_index + 1, u12, v12);

   // vertical interpolation at upper-right corner of one unit square
   Units::Speed u22, v22;
   VerticallyInterpolateWind(alt, new_lower_left_x_index + 1, new_lower_left_y_index + 1, u22, v22);

   // do horizontal bilinear interpolation
   BilinearInterpolation(x_aircraft, y_aircraft, new_x_lower_left, new_x_lower_right, new_y_lower_left,
                         new_y_upper_left, u11, u21, u12, u22, u);
   BilinearInterpolation(x_aircraft, y_aircraft, new_x_lower_left, new_x_lower_right, new_y_lower_left,
                         new_y_upper_left, v11, v21, v12, v22, v);
}  // InterpolateWind

Units::Pressure WindLegacy::InterpolatePressure(Units::Angle latitude_in, Units::Angle longitude_in,
                                                Units::Length alt) {

   int lower_left_x_index, lower_left_y_index;
   FindLowerLeftCornerIndicesLambertProjectionUsgs(latitude_in, longitude_in, lower_left_x_index, lower_left_y_index);

   // Lambert Projection of the Aircraft's Current Position
   Units::MetersLength x_aircraft, y_aircraft;
   LambertProjectionUsgs(latitude_in, longitude_in, x_aircraft, y_aircraft);

   // Lambert Projection of Grid Locations
   Units::MetersLength x_lower_left, y_lower_left;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index][lower_left_x_index],
                         m_wind_data[0].lon[lower_left_y_index][lower_left_x_index], x_lower_left, y_lower_left);

   Units::MetersLength x_lower_right, y_lower_right;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index][lower_left_x_index + 1],
                         m_wind_data[0].lon[lower_left_y_index][lower_left_x_index + 1], x_lower_right, y_lower_right);

   Units::MetersLength x_upper_left, y_upper_left;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index + 1][lower_left_x_index],
                         m_wind_data[0].lon[lower_left_y_index + 1][lower_left_x_index], x_upper_left, y_upper_left);

   Units::MetersLength x_upper_right, y_upper_right;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index + 1][lower_left_x_index + 1],
                         m_wind_data[0].lon[lower_left_y_index + 1][lower_left_x_index + 1], x_upper_right,
                         y_upper_right);

   int new_lower_left_x_index, new_lower_left_y_index;
   Units::MetersLength new_x_lower_left, new_y_lower_left, new_x_lower_right, new_y_lower_right, new_x_upper_left,
         new_y_upper_left, new_x_upper_right, new_y_upper_right;

   CheckBox(lower_left_x_index, lower_left_y_index, x_aircraft, y_aircraft, x_lower_left, y_lower_left, x_lower_right,
            y_lower_right, x_upper_left, y_upper_left, x_upper_right, y_upper_right, new_lower_left_x_index,
            new_lower_left_y_index, new_x_lower_left, new_y_lower_left, new_x_lower_right, new_y_lower_right,
            new_x_upper_left, new_y_upper_left, new_x_upper_right, new_y_upper_right);

   // do four vertical interpolations;then do horizontal linear interpolation
   // vertical interpolation at lower-left corner of the unit square
   Units::Pressure t11, t21, t12, t22, t;
   t11 = VerticallyInterpolatePressure(alt, new_lower_left_x_index, new_lower_left_y_index);

   // vertical interpolation at lower-right corner of one unit square
   t21 = VerticallyInterpolatePressure(alt, new_lower_left_x_index + 1, new_lower_left_y_index);

   // vertical interpolation at upper-left corner of the unit square
   t12 = VerticallyInterpolatePressure(alt, new_lower_left_x_index, new_lower_left_y_index + 1);

   // vertical interpolation at upper-right corner of one unit square
   t22 = VerticallyInterpolatePressure(alt, new_lower_left_x_index + 1, new_lower_left_y_index + 1);

   // do horizontal bilinear interpolation
   BilinearInterpolation(x_aircraft, y_aircraft, new_x_lower_left, new_x_lower_right, new_y_lower_left,
                         new_y_upper_left, t11, t21, t12, t22, t);
   return t;
}

Units::KelvinTemperature WindLegacy::InterpolateTemperature(Units::Angle latitude_in, Units::Angle longitude_in,
                                                            Units::Length alt) {

   // disable the temperature feature because we don't support it for Legacy
   return Units::KelvinTemperature(-9999);

   int lower_left_x_index, lower_left_y_index;
   FindLowerLeftCornerIndicesLambertProjectionUsgs(latitude_in, longitude_in, lower_left_x_index, lower_left_y_index);

   // Lambert Projection of the Aircraft's Current Position
   Units::MetersLength x_aircraft, y_aircraft;
   LambertProjectionUsgs(latitude_in, longitude_in, x_aircraft, y_aircraft);

   // Lambert Projection of Grid Locations
   Units::MetersLength x_lower_left, y_lower_left;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index][lower_left_x_index],
                         m_wind_data[0].lon[lower_left_y_index][lower_left_x_index], x_lower_left, y_lower_left);

   Units::MetersLength x_lower_right, y_lower_right;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index][lower_left_x_index + 1],
                         m_wind_data[0].lon[lower_left_y_index][lower_left_x_index + 1], x_lower_right, y_lower_right);

   Units::MetersLength x_upper_left, y_upper_left;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index + 1][lower_left_x_index],
                         m_wind_data[0].lon[lower_left_y_index + 1][lower_left_x_index], x_upper_left, y_upper_left);

   Units::MetersLength x_upper_right, y_upper_right;
   LambertProjectionUsgs(m_wind_data[0].lat[lower_left_y_index + 1][lower_left_x_index + 1],
                         m_wind_data[0].lon[lower_left_y_index + 1][lower_left_x_index + 1], x_upper_right,
                         y_upper_right);

   int new_lower_left_x_index, new_lower_left_y_index;
   Units::MetersLength new_x_lower_left, new_y_lower_left, new_x_lower_right, new_y_lower_right, new_x_upper_left,
         new_y_upper_left, new_x_upper_right, new_y_upper_right;

   CheckBox(lower_left_x_index, lower_left_y_index, x_aircraft, y_aircraft, x_lower_left, y_lower_left, x_lower_right,
            y_lower_right, x_upper_left, y_upper_left, x_upper_right, y_upper_right, new_lower_left_x_index,
            new_lower_left_y_index, new_x_lower_left, new_y_lower_left, new_x_lower_right, new_y_lower_right,
            new_x_upper_left, new_y_upper_left, new_x_upper_right, new_y_upper_right);

   // do four vertical interpolations;then do horizontal linear interpolation
   // vertical interpolation at lower-left corner of the unit square
   Units::Temperature t11, t21, t12, t22, t;
   t11 = VerticallyInterpolateTemp(alt, new_lower_left_x_index, new_lower_left_y_index);

   // vertical interpolation at lower-right corner of one unit square
   t21 = VerticallyInterpolateTemp(alt, new_lower_left_x_index + 1, new_lower_left_y_index);

   // vertical interpolation at upper-left corner of the unit square
   t12 = VerticallyInterpolateTemp(alt, new_lower_left_x_index, new_lower_left_y_index + 1);

   // vertical interpolation at upper-right corner of one unit square
   t22 = VerticallyInterpolateTemp(alt, new_lower_left_x_index + 1, new_lower_left_y_index + 1);

   // do horizontal bilinear interpolation
   BilinearInterpolation(x_aircraft, y_aircraft, new_x_lower_left, new_x_lower_right, new_y_lower_left,
                         new_y_upper_left, t11, t21, t12, t22, t);
   return t;
}

void WindLegacy::VerticallyInterpolateWind(const Units::Length alt, const int x_index, const int y_index,
                                           Units::Speed &u, Units::Speed &v) {

   /*
     Method: Vertically interpolate the (u, v) wind at altitude (alt), and at the horizontal grid point indexed with
     (x_index, y_index) Input: struct WindData wind_array_input[]: the matrix containing the wind data int x_index, int
     y_index: the indices for the grid point Units::Length alt:

     output:
     Units::Speed u: interpolated wind u (m/s)
     Units::Speed v: interpolated wind v (n/s)
   */

   Units::Length maxmin = Units::FeetLength(-1.);
   Units::Length minmax = Units::FeetLength(Units::infinity());
   bool do_interpolation = true;
   int alt_index1 = -1;
   int alt_index2 = -2;

   bool minmax_found = false;
   bool maxmin_found = false;

   for (int i = 0; i < NUMALT; i++) {
      Units::Length alt_tmp = m_wind_data[i].alt[y_index][x_index];

      // if there is an alt_tmp that is equal to alt, then use the (u,v) at that i index:

      if (fabs(Units::FeetLength(alt_tmp - alt).value()) < 0.1) {

         // essentially if (alt_tmp == alt)

         minmax_found = false;
         maxmin_found = false;
         alt_index1 = i;
         alt_index2 = i;
         break;
      }

      // find the i index for the max altitude among those altitudes that are less than alt

      if (alt_tmp < alt) {
         if (alt_tmp >= maxmin) {
            maxmin = alt_tmp;
            alt_index1 = i;
            maxmin_found = true;
         }
      }

      // find the i index for the min altitude among those altitudes that are greater than alt

      if (alt_tmp > alt) {
         if (alt_tmp <= minmax) {
            minmax = alt_tmp;
            alt_index2 = i;
            minmax_found = true;
         }
      }
   }

   // determine whether to use interpolation

   int index = -1;  // the index for non-interpolation

   if (!maxmin_found && !minmax_found) {

      index = alt_index1;
      do_interpolation = false;

   } else if (maxmin_found && !minmax_found) {

      index = alt_index1;
      do_interpolation = false;

   } else if (!maxmin_found && minmax_found) {

      index = alt_index2;
      do_interpolation = false;

   } else {

      // maxmin_found && minmax_found

      do_interpolation = true;
   }

   if (do_interpolation) {

      // get the altitudes and winds at those two indices

      Units::Length alt1, alt2;
      Units::Speed u1, u2;
      Units::Speed v1, v2;

      alt1 = maxmin;
      alt2 = minmax;

      assert(alt_index1 >= 0);
      assert(alt_index2 >= 0);
      u1 = m_wind_data[alt_index1].wu[y_index][x_index];
      u2 = m_wind_data[alt_index2].wu[y_index][x_index];

      v1 = m_wind_data[alt_index1].wv[y_index][x_index];
      v2 = m_wind_data[alt_index2].wv[y_index][x_index];

      // interpolate

      LinearInterpolation(alt, alt1, alt2, u1, u2, u);
      LinearInterpolation(alt, alt1, alt2, v1, v2, v);
   } else {

      // use the (u, v) at the alt corresponding to the index: no interpolation

      u = m_wind_data[index].wu[y_index][x_index];
      v = m_wind_data[index].wv[y_index][x_index];
   }

}  // VerticallyInterpolateWind

Units::Temperature WindLegacy::VerticallyInterpolateTemp(const Units::Length alt, const int x_index,
                                                         const int y_index) {

   Units::Temperature t;
   Units::Length maxmin = Units::FeetLength(-1.);
   Units::Length minmax = Units::FeetLength(Units::infinity());
   bool do_interpolation = true;
   int alt_index1 = -1;
   int alt_index2 = -2;

   bool minmax_found = false;
   bool maxmin_found = false;

   for (int i = 0; i < NUMALT; i++) {
      Units::Length alt_tmp = m_wind_data[i].alt[y_index][x_index];

      // if there is an alt_tmp that is equal to alt, then use the (u,v) at that i index:

      if (fabs(Units::FeetLength(alt_tmp - alt).value()) < 0.1) {

         // essentially if (alt_tmp == alt)

         minmax_found = false;
         maxmin_found = false;
         alt_index1 = i;
         alt_index2 = i;
         break;
      }

      // find the i index for the max altitude among those altitudes that are less than alt

      if (alt_tmp < alt) {
         if (alt_tmp >= maxmin) {
            maxmin = alt_tmp;
            alt_index1 = i;
            maxmin_found = true;
         }
      }

      // find the i index for the min altitude among those altitudes that are greater than alt

      if (alt_tmp > alt) {
         if (alt_tmp <= minmax) {
            minmax = alt_tmp;
            alt_index2 = i;
            minmax_found = true;
         }
      }
   }

   // determine whether to use interpolation

   int index = -1;  // the index for non-interpolation

   if (!maxmin_found && !minmax_found) {

      index = alt_index1;
      do_interpolation = false;

   } else if (maxmin_found && !minmax_found) {

      index = alt_index1;
      do_interpolation = false;

   } else if (!maxmin_found && minmax_found) {

      index = alt_index2;
      do_interpolation = false;

   } else {

      // maxmin_found && minmax_found

      do_interpolation = true;
   }

   if (do_interpolation) {

      // get the altitudes and winds at those two indices

      Units::Length alt1, alt2;
      Units::Temperature t1, t2;

      alt1 = maxmin;
      alt2 = minmax;

      assert(alt_index1 >= 0);
      assert(alt_index2 >= 0);
      t1 = m_wind_data[alt_index1].temp[y_index][x_index];
      t2 = m_wind_data[alt_index2].temp[y_index][x_index];

      // interpolate

      LinearInterpolation(alt, alt1, alt2, t1, t2, t);
   } else {

      // use the temp at the alt corresponding to the index: no interpolation

      t = m_wind_data[index].temp[y_index][x_index];
   }
   return t;

}  // VerticallyInterpolateTemp

Units::Pressure WindLegacy::VerticallyInterpolatePressure(const Units::Length alt, const int x_index,
                                                          const int y_index) {

   Units::Pressure p;
   Units::Length maxmin = Units::FeetLength(-1.);
   Units::Length minmax = Units::FeetLength(Units::infinity());
   bool do_interpolation = true;
   int alt_index1 = -1;
   int alt_index2 = -2;

   bool minmax_found = false;
   bool maxmin_found = false;

   for (int i = 0; i < NUMALT; i++) {
      Units::Length alt_tmp = m_wind_data[i].alt[y_index][x_index];

      // if there is an alt_tmp that is equal to alt, then use the (u,v) at that i index:

      if (fabs(Units::FeetLength(alt_tmp - alt).value()) < 0.1) {

         // essentially if (alt_tmp == alt)

         minmax_found = false;
         maxmin_found = false;
         alt_index1 = i;
         alt_index2 = i;
         break;
      }

      // find the i index for the max altitude among those altitudes that are less than alt

      if (alt_tmp < alt) {
         if (alt_tmp >= maxmin) {
            maxmin = alt_tmp;
            alt_index1 = i;
            maxmin_found = true;
         }
      }

      // find the i index for the min altitude among those altitudes that are greater than alt

      if (alt_tmp > alt) {
         if (alt_tmp <= minmax) {
            minmax = alt_tmp;
            alt_index2 = i;
            minmax_found = true;
         }
      }
   }

   // determine whether to use interpolation

   int index = -1;  // the index for non-interpolation

   if (!maxmin_found && !minmax_found) {

      index = alt_index1;
      do_interpolation = false;

   } else if (maxmin_found && !minmax_found) {

      index = alt_index1;
      do_interpolation = false;

   } else if (!maxmin_found && minmax_found) {

      index = alt_index2;
      do_interpolation = false;

   } else {

      // maxmin_found && minmax_found

      do_interpolation = true;
   }

   if (do_interpolation) {

      // get the altitudes and winds at those two indices

      Units::Length alt1, alt2;
      Units::Pressure t1, t2;

      alt1 = maxmin;
      alt2 = minmax;

      assert(alt_index1 >= 0);
      assert(alt_index2 >= 0);
      t1 = m_wind_data[alt_index1].pressure[y_index][x_index];
      t2 = m_wind_data[alt_index2].pressure[y_index][x_index];

      // interpolate

      LinearInterpolation(alt, alt1, alt2, t1, t2, p);
   } else {

      // use the temp at the alt corresponding to the index: no interpolation

      p = m_wind_data[index].pressure[y_index][x_index];
   }
   return p;

}  // VerticallyInterpolatePressure

void WindLegacy::LinearInterpolation(const Units::Length x, const Units::Length x1, const Units::Length x2,
                                     const Units::Speed y1, const Units::Speed y2, Units::Speed &out) {

   /*
     Method: linear interpolation
     Input:
     Units::Length x, Units::Length x1, Units::Length x2, Units::Speed y1, Units::Speed y2

     output:
     Units::Speed& out: interpolated value
   */

   Units::Length min = Units::min(x1, x2);
   Units::Length max = Units::max(x1, x2);

   if (x < min || x > max) {

      // x is ouside of the line segment between x1 and x2: do not interpolate;
      // instead, use y value at the closest x

      if (abs(x - x1) <= abs(x - x2)) {

         out = y1;

      } else {

         out = y2;
      }

   } else {

      // interpolate

      out = y1 + (y2 - y1) / (x2 - x1) * (x - x1);
   }

}  // LinearInterpolation

void WindLegacy::LinearInterpolation(const Units::Length x, const Units::Length x1, const Units::Length x2,
                                     const Units::Temperature y1, const Units::Temperature y2,
                                     Units::Temperature &out) {

   /*
     Method: linear interpolation
     Input:
     Units::Length x, Units::Length x1, Units::Length x2, Units::Temperature y1, Units::Temperature y2

     output:
     Units::Temperature& out: interpolated value
   */

   Units::Length min = Units::min(x1, x2);
   Units::Length max = Units::max(x1, x2);

   if (x < min || x > max) {

      // x is ouside of the line segment between x1 and x2: do not interpolate;
      // instead, use y value at the closest x

      if (abs(x - x1) <= abs(x - x2)) {

         out = y1;

      } else {

         out = y2;
      }

   } else {

      // interpolate

      out = y1 + (y2 - y1) / (x2 - x1) * (x - x1);
   }

}  // LinearInterpolation

void WindLegacy::LinearInterpolation(const Units::Length x, const Units::Length x1, const Units::Length x2,
                                     const Units::Pressure y1, const Units::Pressure y2, Units::Pressure &out) {

   /*
     Method: linear interpolation
     Input:
     Units::Length x, Units::Length x1, Units::Length x2, Units::Pressure y1, Units::Pressure y2

     output:
     Units::Pressure& out: interpolated value
   */

   Units::Length min = Units::min(x1, x2);
   Units::Length max = Units::max(x1, x2);

   if (x < min || x > max) {

      // x is ouside of the line segment between x1 and x2: do not interpolate;
      // instead, use y value at the closest x

      if (abs(x - x1) <= abs(x - x2)) {

         out = y1;

      } else {

         out = y2;
      }

   } else {

      // interpolate

      out = y1 + (y2 - y1) / (x2 - x1) * (x - x1);
   }

}  // LinearInterpolation

void WindLegacy::BilinearInterpolation(Units::Length x, Units::Length y, Units::Length x1, Units::Length x2,
                                       Units::Length y1, Units::Length y2, Units::Speed f11, Units::Speed f21,
                                       Units::Speed f12, Units::Speed f22, Units::Speed &out) {

   /*
     Method: bilinear interpolation
     Input:
     Units::Length x, Units::Length y, Units::Length x1, Units::Length x2, Units::Length y1, Units::Length y2,
     Units::Speed f11, Units::Speed f21, Units::Speed f12, Units::Speed f22

     output:
     Units::Speed& out: interpolated value
   */

   Units::Speed fr1, fr2;
   LinearInterpolation(x, x1, x2, f11, f21, fr1);
   LinearInterpolation(x, x1, x2, f12, f22, fr2);
   LinearInterpolation(y, y1, y2, fr1, fr2, out);

}  // bilinear_interpolation

void WindLegacy::BilinearInterpolation(Units::Length x, Units::Length y, Units::Length x1, Units::Length x2,
                                       Units::Length y1, Units::Length y2, Units::Temperature f11,
                                       Units::Temperature f21, Units::Temperature f12, Units::Temperature f22,
                                       Units::Temperature &out) {

   /*
     Method: bilinear interpolation
     Input:
     Units::Length x, Units::Length y, Units::Length x1, Units::Length x2, Units::Length y1, Units::Length y2,
     Units::Temperature f11, Units::Temperature f21, Units::Temperature f12, Units::Temperature f22

     output:
     Units::Temperature& out: interpolated value
   */

   Units::Temperature fr1, fr2;
   LinearInterpolation(x, x1, x2, f11, f21, fr1);
   LinearInterpolation(x, x1, x2, f12, f22, fr2);
   LinearInterpolation(y, y1, y2, fr1, fr2, out);

}  // bilinear_interpolation

void WindLegacy::BilinearInterpolation(Units::Length x, Units::Length y, Units::Length x1, Units::Length x2,
                                       Units::Length y1, Units::Length y2, Units::Pressure f11, Units::Pressure f21,
                                       Units::Pressure f12, Units::Pressure f22, Units::Pressure &out) {

   /*
     Method: bilinear interpolation
     Input:
     Units::Length x, Units::Length y, Units::Length x1, Units::Length x2, Units::Length y1, Units::Length y2,
     Units::Pressure f11, Units::Pressure f21, Units::Pressure f12, Units::Pressure f22

     output:
     Units::Pressure& out: interpolated value
   */

   Units::Pressure fr1, fr2;
   LinearInterpolation(x, x1, x2, f11, f21, fr1);
   LinearInterpolation(x, x1, x2, f12, f22, fr2);
   LinearInterpolation(y, y1, y2, fr1, fr2, out);

}  // bilinear_interpolation

void WindLegacy::WindRotation(const Units::Angle longitude, const Units::Speed u, const Units::Speed v,
                              Units::Speed &u_true_north, Units::Speed &v_true_north) {

   /*
     Method: rotate wind (u, v) from Lambert coordinate system into wind in true north
     Input:
     Units::Angle longitude: the longitude of the position of the wind
     Units::Speed u, Units::Speed v: wind in Lambert coordinate system

     output:
     Units::Speed& u_true_north, Units::Speed& v_true_north: wind in true north
   */

   Units::Angle lat0 = Units::DegreesAngle(25.);
   Units::Angle lon0 = Units::DegreesAngle(-95.);
   Units::Angle alpha = sin(lat0) * (longitude - lon0);

   double sin_alpha = sin(alpha);
   double cos_alpha = cos(alpha);

   u_true_north = cos_alpha * u + sin_alpha * v;
   v_true_north = -sin_alpha * u + cos_alpha * v;

}  // WindRotation

void WindLegacy::CheckBox(const int lower_left_x_index, const int lower_left_y_index, const Units::Length x_aircraft,
                          const Units::Length y_aircraft, const Units::Length x_lower_left,
                          const Units::Length y_lower_left, const Units::Length x_lower_right,
                          const Units::Length y_lower_right, const Units::Length x_upper_left,
                          const Units::Length y_upper_left, const Units::Length x_upper_right,
                          const Units::Length y_upper_right, int &new_lower_left_x_index, int &new_lower_left_y_index,
                          Units::Length &new_x_lower_left, Units::Length &new_y_lower_left,
                          Units::Length &new_x_lower_right, Units::Length &new_y_lower_right,
                          Units::Length &new_x_upper_left, Units::Length &new_y_upper_left,
                          Units::Length &new_x_upper_right, Units::Length &new_y_upper_right) {

   // local-version variables:

   Units::MetersLength x_ll = x_lower_left;
   Units::MetersLength y_ll = y_lower_left;
   Units::MetersLength x_lr = x_lower_right;
   Units::MetersLength y_lr = y_lower_right;
   Units::MetersLength x_ul = x_upper_left;
   Units::MetersLength y_ul = y_upper_left;
   Units::MetersLength x_ur = x_upper_right;
   Units::MetersLength y_ur = y_upper_right;
   int ll_x_index = lower_left_x_index;
   int ll_y_index = lower_left_y_index;

   // "default" return values if the aircraft is already within the box

   new_x_lower_left = x_lower_left;
   new_y_lower_left = y_lower_left;
   new_x_lower_right = x_lower_right;
   new_y_lower_right = y_lower_right;
   new_x_upper_left = x_upper_left;
   new_y_upper_left = y_upper_left;
   new_x_upper_right = x_upper_right;
   new_y_upper_right = y_upper_right;

   // check and correct:

   Units::MetersLength tmp1, tmp2, tmp3, tmp4;
   tmp1 = y_aircraft - LinearFunctionOfX(x_aircraft, x_ll, y_ll, x_ul, y_ul);
   tmp2 = y_aircraft - LinearFunctionOfX(x_aircraft, x_lr, y_lr, x_ur, y_ur);
   tmp3 = y_aircraft - LinearFunctionOfX(x_aircraft, x_ll, y_ll, x_lr, y_lr);
   tmp4 = y_aircraft - LinearFunctionOfX(x_aircraft, x_ul, y_ul, x_ur, y_ur);
   new_lower_left_x_index = ll_x_index;
   new_lower_left_y_index = ll_y_index;

   const Units::MetersLength MARGIN(4000.);

   if (tmp1 * tmp2 > Units::MetersArea(0)  // the aircraft is to the same side of both lines;thus it is outside the box
       && abs(tmp1) > MARGIN && abs(tmp2) > MARGIN) {  // reduce sensitivity

      // increase or decrease ll_x_index to find the correct index

      Units::Length d1 = DistancePointToLine(x_aircraft, y_aircraft, x_ll, y_ll, x_ul, y_ul);
      Units::Length d2 = DistancePointToLine(x_aircraft, y_aircraft, x_lr, y_lr, x_ur, y_ur);

      if (d1 > d2) {

         // the aircraft is to the right of the box

         new_lower_left_x_index = ll_x_index + 1;
      } else {

         new_lower_left_x_index = ll_x_index - 1;
      }
   }

   if (tmp3 * tmp4 > Units::MetersArea(0)  // the aircraft is to the same side of both lines; thus it is outside the box
       && abs(tmp3) > MARGIN && abs(tmp4) > MARGIN) {  // reduce sensitivity

      // increase or devrease lower_left_y_index to find the correct index

      Units::Length d1 = DistancePointToLine(x_aircraft, y_aircraft, x_ll, y_ll, x_lr, y_lr);
      Units::Length d2 = DistancePointToLine(x_aircraft, y_aircraft, x_ul, y_ul, x_ur, y_ur);

      if (d1 > d2) {

         // the aircraft is above the box

         new_lower_left_y_index = ll_y_index + 1;

      } else {

         new_lower_left_y_index = ll_y_index - 1;
      }
   }

   LambertProjectionUsgs(m_wind_data[0].lat[new_lower_left_y_index][new_lower_left_x_index],
                         m_wind_data[0].lon[new_lower_left_y_index][new_lower_left_x_index], new_x_lower_left,
                         new_y_lower_left);

   LambertProjectionUsgs(m_wind_data[0].lat[new_lower_left_y_index][new_lower_left_x_index + 1],
                         m_wind_data[0].lon[new_lower_left_y_index][new_lower_left_x_index + 1], new_x_lower_right,
                         new_y_lower_right);

   LambertProjectionUsgs(m_wind_data[0].lat[new_lower_left_y_index + 1][new_lower_left_x_index],
                         m_wind_data[0].lon[new_lower_left_y_index + 1][new_lower_left_x_index], new_x_upper_left,
                         new_y_upper_left);

   LambertProjectionUsgs(m_wind_data[0].lat[new_lower_left_y_index + 1][new_lower_left_x_index + 1],
                         m_wind_data[0].lon[new_lower_left_y_index + 1][new_lower_left_x_index + 1], new_x_upper_right,
                         new_y_upper_right);

   /*
   // prepare local variables for the next possible loop
   x_ll = new_x_lower_left;		y_ll = new_y_lower_left;
   x_lr = new_x_lower_right;		y_lr = new_y_lower_right;
   x_ul = new_x_upper_left;		y_ul = new_y_upper_left;
   x_ur = new_x_upper_right;		y_ur = new_y_upper_right;
   ll_x_index = new_lower_left_x_index;
   ll_y_index = new_lower_left_y_index;

   // determine whether to keep looping:
   tmp1 = y_aircraft-f(x_aircraft,new_x_lower_left,new_y_lower_left,new_x_upper_left,new_y_upper_left);
   tmp2 = y_aircraft-f(x_aircraft,new_x_lower_right,new_y_lower_right,new_x_upper_right,new_y_upper_right);

   tmp3 = y_aircraft-f(x_aircraft,new_x_lower_left,new_y_lower_left,new_x_lower_right,new_y_lower_right);
   tmp4 =
   y_aircraft-LinearFunctionOfX(x_aircraft,new_x_upper_left,new_y_upper_left,new_x_upper_right,new_y_upper_right); }
   while(tmp1 * tmp2 > 0.0 || tmp3 * tmp4 > 0.0);
   */

}  // CheckBox

Units::Length WindLegacy::LinearFunctionOfX(const Units::Length x, const Units::Length x1, const Units::Length y1,
                                            const Units::Length x2, const Units::Length y2) {

   // the linear function (of x) representing the line passing through (x1, y1) and (x2, y2)

   return (x - x1) * (y2 - y1) / (x2 - x1) + y1;
}

Units::Length WindLegacy::DistancePointToLine(const Units::Length x0, const Units::Length y0, const Units::Length x1,
                                              const Units::Length y1, const Units::Length x2, const Units::Length y2) {

   // distance from (x0, y0) to the line passing through (x1, y1) and (x2, y2)

   // y = ax + b

   double a = (y2 - y1) / (x2 - x1);
   Units::Length b = (x2 * y1 - x1 * y2) / (x2 - x1);

   Units::Length dx = (a * a * x0 + a * b - a * y0) / (a * a + 1.);
   Units::Length dy = (y0 - a * x0 - b) / (a * a + 1.);

   return sqrt(dx * dx + dy * dy);
}

int WindLegacy::GetFlightLevelLowerBound() { return static_cast<int>(MINIMUM_ALTITUDE_LIMIT.value()); }

int WindLegacy::GetFlightLevelUpperBound() { return static_cast<int>(MAXIMUM_ALTITUDE_LIMIT.value()); }
