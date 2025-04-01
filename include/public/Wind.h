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

#include <scalar/Length.h>
#include <scalar/Speed.h>
#include <scalar/Angle.h>
#include <scalar/Temperature.h>
#include "public/WeatherPrediction.h"
#include "public/AircraftIntent.h"

namespace aaesim::test {
class WindLegacy_readRAPWindFile_Test;
class Wind_interpolate_wind_Test;
class Wind_interpolate_wind_scalar_Test;
class Wind_interpolate_wind_matrix_Test;
class WindLegacy_check_box_Test;
class Wind_vertically_interpolate_wind_Test;
class Wind_readRAPTestDataWindFile_Test;
class Wind_interpolateTemp_Test;
namespace utils {
class WindSpeedUtils;
}
}  // namespace aaesim::test

enum WindDataFormat { RUC_FORMAT, RAP_FORMAT, BINARY };

enum WindFileType { FORECAST_FILE, TRUTH_FILE };

class Wind {
   friend class aaesim::test::WindLegacy_readRAPWindFile_Test;

   friend class aaesim::test::Wind_interpolateTemp_Test;

   friend class aaesim::test::Wind_interpolate_wind_Test;

   friend class aaesim::test::Wind_interpolate_wind_scalar_Test;

   friend class aaesim::test::Wind_interpolate_wind_matrix_Test;

   friend class aaesim::test::WindLegacy_check_box_Test;

   friend class aaesim::test::Wind_vertically_interpolate_wind_Test;

   friend class aaesim::test::Wind_readRAPTestDataWindFile_Test;

   friend class aaesim::test::utils::WindSpeedUtils;

  public:
   inline static const Units::NauticalMilesLength SAMPLING_DISTANCE_FROM_END_OF_ROUTE{60};

   Wind() = default;

   virtual ~Wind() = default;

   void InterpolateTrueWind(const Units::Angle lat_in, const Units::Angle lon_in, const Units::Length altitude,
                            aaesim::open_source::WindStack &east_west, aaesim::open_source::WindStack &north_south);

   void InterpolateForecastWind(const std::shared_ptr<TangentPlaneSequence> &tangentPlaneSequence,
                                const Units::Length x_in, const Units::Length y_in, const Units::Length altitude,
                                Units::Speed &east_west, Units::Speed &north_south);

   virtual void InterpolateWindScalar(Units::Angle lat_in, Units::Angle lon_in, Units::Length altitude,
                                      Units::Speed &east_west, Units::Speed &north_south) = 0;

   virtual Units::KelvinTemperature InterpolateTemperature(const Units::Angle latitude_in,
                                                           const Units::Angle longitude_in,
                                                           const Units::Length altitude) = 0;

   virtual Units::Pressure InterpolatePressure(const Units::Angle latitude_in, const Units::Angle longitude_in,
                                               const Units::Length altitude) = 0;

   static std::shared_ptr<Wind> GetWindTruthInstance();

   static void SetWindTruthInstance(std::shared_ptr<Wind> &truth_instance);

   static void SetUseWind(const bool useWind);

   static bool UseWind();

  protected:
   virtual void InterpolateWind(Units::Angle latitude_in, Units::Angle longitude_in, Units::Length altitude,
                                Units::Speed &u, Units::Speed &v) = 0;

   virtual void InterpolateWindMatrix(Units::Angle lat_in, Units::Angle lon_in, Units::Length alt_in,
                                      aaesim::open_source::WindStack &east_west,
                                      aaesim::open_source::WindStack &north_south) = 0;

  private:
   inline static bool m_use_wind{false};
   inline static log4cplus::Logger m_logger{log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("Wind"))};
   inline static std::shared_ptr<Wind> m_wind_truth_instance{};
};

inline void Wind::SetWindTruthInstance(std::shared_ptr<Wind> &truth_instance) {
   m_wind_truth_instance = truth_instance;
}

inline std::shared_ptr<Wind> Wind::GetWindTruthInstance() { return m_wind_truth_instance; }

inline void Wind::SetUseWind(const bool useWind) { m_use_wind = useWind; }

inline bool Wind::UseWind() { return m_use_wind; }
