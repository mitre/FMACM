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

#include "public/Logging.h"
#include "public/Wind.h"

#define NUMALT (50)
#define NUMLAT (337)
#define NUMLON (451)

#define ULHLAT (57.)
#define ULHLG (-129.)
#define LRHLAT (18.)
#define LRHLG (-60.)

#define NUM_LAT_GRID_POINTS (ULHLAT - LRHLAT + 1)
#define NUM_LON_GRID_POINTS (LRHLG - ULHLG + 1)

// needed for WindSpeedUtils friend class
namespace aaesim {
namespace test {
class WindLegacy_readRAPWindFile_Test;
class Wind_interpolate_wind_Test;
class Wind_interpolate_wind_scalar_Test;
class Wind_interpolate_wind_matrix_Test;
class WindLegacy_check_box_Test;
class Wind_vertically_interpolate_wind_Test;
class Wind_readRAPTestDataWindFile_Test;
namespace utils {
class WindSpeedUtils;
}
}  // namespace test
}  // namespace aaesim

struct WindData {
   Units::Length alt[NUMLAT][NUMLON];

   Units::Angle lat[NUMLAT][NUMLON];
   Units::Angle lon[NUMLAT][NUMLON];

   Units::Speed wu[NUMLAT][NUMLON];
   Units::Speed wv[NUMLAT][NUMLON];

   Units::Temperature temp[NUMLAT][NUMLON];
   Units::PascalsPressure pressure[NUMLAT][NUMLON];
};

namespace aaesim::weather {
class WindLegacy final : public Wind {
   friend class aaesim::test::WindLegacy_readRAPWindFile_Test;

   friend class aaesim::test::Wind_interpolate_wind_Test;

   friend class aaesim::test::Wind_interpolate_wind_scalar_Test;

   friend class aaesim::test::Wind_interpolate_wind_matrix_Test;

   friend class aaesim::test::WindLegacy_check_box_Test;

   friend class aaesim::test::Wind_vertically_interpolate_wind_Test;

   friend class aaesim::test::Wind_readRAPTestDataWindFile_Test;

   friend class aaesim::test::utils::WindSpeedUtils;  //::PopulateWindMatrices(const Units::Speed windSpeedEast, const
                                                      //: Units::Speed windSpeedNorth);

  public:
   WindLegacy() = default;

   virtual ~WindLegacy() = default;

   void readRUCWindFile(const std::string &file_name);

   void readRAPWindFile(const std::string &file_name);

   void InterpolateWind(Units::Angle latitude_in, Units::Angle longitude_in, Units::Length alt, Units::Speed &u,
                        Units::Speed &v) override;

   void InterpolateWindScalar(Units::Angle lat_in, Units::Angle lon_in, Units::Length altitude, Units::Speed &east_west,
                              Units::Speed &north_south) override;

   void InterpolateWindMatrix(Units::Angle lat_in, Units::Angle lon_in, Units::Length alt_in,
                              aaesim::open_source::WindStack &east_west,
                              aaesim::open_source::WindStack &north_south) override;

   Units::KelvinTemperature InterpolateTemperature(Units::Angle latitude, Units::Angle longitude,
                                                   Units::Length altitude) override;

   Units::Pressure InterpolatePressure(Units::Angle latitude, Units::Angle longitude, Units::Length altitude) override;

  private:
   static log4cplus::Logger m_logger;
   inline static const Units::FeetLength MAXIMUM_ALTITUDE_LIMIT{45000};
   inline static const Units::FeetLength MINIMUM_ALTITUDE_LIMIT{0};

   WindData m_wind_data[50];

   void VerticallyInterpolateWind(const Units::Length alt, const int x_index, const int y_index, Units::Speed &u,
                                  Units::Speed &v);

   Units::Temperature VerticallyInterpolateTemp(const Units::Length alt, const int x_index, const int y_index);

   Units::Pressure VerticallyInterpolatePressure(const Units::Length alt, const int x_index, const int y_index);

   int GetFlightLevelLowerBound();

   int GetFlightLevelUpperBound();

   void CheckBox(const int lower_left_x_index, const int lower_left_y_index, const Units::Length x_aircraft,
                 const Units::Length y_aircraft, const Units::Length x_lower_left, const Units::Length y_lower_left,
                 const Units::Length x_lower_right, const Units::Length y_lower_right, const Units::Length x_upper_left,
                 const Units::Length y_upper_left, const Units::Length x_upper_right, const Units::Length y_upper_right,
                 int &new_lower_left_x_index, int &new_lower_left_y_index, Units::Length &new_x_lower_left,
                 Units::Length &new_y_lower_left, Units::Length &new_x_lower_right, Units::Length &new_y_lower_right,
                 Units::Length &new_x_upper_left, Units::Length &new_y_upper_left, Units::Length &new_x_upper_right,
                 Units::Length &new_y_upper_right);

   Units::Length LinearFunctionOfX(const Units::Length x, const Units::Length x1, const Units::Length y1,
                                   const Units::Length x2, const Units::Length y2);

  public:
   static void WindRotation(const Units::Angle longitude, const Units::Speed u, const Units::Speed v,
                            Units::Speed &u_true_north, Units::Speed &v_true_north);

   static void FindIndicesLambertProjectionUsgs(const Units::Angle latitude_in, const Units::Angle longitude_in,
                                                double &i_out, double &j_out);

   static void FindLowerLeftCornerIndicesLambertProjectionUsgs(const Units::Angle latitude_in,
                                                               const Units::Angle longitude_in, int &i_out, int &j_out);

   static void LambertProjectionUsgs(const Units::Angle latitude_in, const Units::Angle longitude_in,
                                     Units::Length &x_out, Units::Length &y_out);

   static double CalculateTerm_t(const Units::Angle phi, const double e);

   static double CalculateTerm_m(const Units::Angle phi, const double e);

   static Units::Length DistancePointToLine(const Units::Length x0, const Units::Length y0, const Units::Length x1,
                                            const Units::Length y1, const Units::Length x2, const Units::Length y2);

   static void LinearInterpolation(const Units::Length x, const Units::Length x1, const Units::Length x2,
                                   const Units::Speed y1, const Units::Speed y2, Units::Speed &out);

   static void LinearInterpolation(const Units::Length x, const Units::Length x1, const Units::Length x2,
                                   const Units::Temperature y1, const Units::Temperature y2, Units::Temperature &out);

   static void LinearInterpolation(const Units::Length x, const Units::Length x1, const Units::Length x2,
                                   const Units::Pressure y1, const Units::Pressure y2, Units::Pressure &out);

   static void BilinearInterpolation(const Units::Length x, const Units::Length y, const Units::Length x1,
                                     const Units::Length x2, const Units::Length y1, const Units::Length y2,
                                     const Units::Speed f11, const Units::Speed f21, const Units::Speed f12,
                                     const Units::Speed f22, Units::Speed &out);

   static void BilinearInterpolation(const Units::Length x, const Units::Length y, const Units::Length x1,
                                     const Units::Length x2, const Units::Length y1, const Units::Length y2,
                                     const Units::Temperature f11, const Units::Temperature f21,
                                     const Units::Temperature f12, const Units::Temperature f22,
                                     Units::Temperature &out);

   static void BilinearInterpolation(const Units::Length x, const Units::Length y, const Units::Length x1,
                                     const Units::Length x2, const Units::Length y1, const Units::Length y2,
                                     const Units::Pressure f11, const Units::Pressure f21, const Units::Pressure f12,
                                     const Units::Pressure f22, Units::Pressure &out);
};
}  // namespace aaesim::weather