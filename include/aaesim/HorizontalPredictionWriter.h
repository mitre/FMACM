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

#include "aaesim/AircraftDataIterationWriter.h"
#include "aaesim/KineticPredictionDataIterationWriter.h"

#include "aaesim/AircraftEntity.h"
#include "public/Logging.h"

namespace aaesim {
class HorizontalPredictionWriter final : public AircraftDataIterationWriter,
                                         public KineticPredictionDataIterationWriter {
  public:
   HorizontalPredictionWriter() : AircraftDataIterationWriter("", "_predicted_kinetic_horizontal_trajectory.csv") {}
   void Gather(const int &iteration_number, const aaesim::AircraftEntity &aircraft) override;
   void Gather(const int &iteration_number, const aaesim::KineticPredictionEntity &entity) override;
   void Finish() override;
   std::string GetFilename() const override { return GetOutputFilename(); }

  private:
   static log4cplus::Logger m_logger;
   static std::vector<std::string> COLUMN_NAMES;
   inline static std::shared_ptr<HorizontalPredictionWriter> m_instance = nullptr;
   struct DataToWrite {
      DataToWrite()
         : iteration_number(INT32_MIN),
           acid(),
           guidance_flight_phase(),
           simulation_time(Units::negInfinity()),
           latitude(Units::negInfinity()),
           longitude(Units::negInfinity()),
           leg_length(Units::negInfinity()),
           path_length(Units::negInfinity()),
           segment_type(),
           enu_course_start(Units::negInfinity()),
           enu_course_end(Units::negInfinity()),
           turn_radius(Units::negInfinity()),
           groundspeed(Units::negInfinity()),
           bank_angle(Units::negInfinity()),
           turn_center_latitude(Units::negInfinity()),
           turn_center_longitude(Units::negInfinity()),
           position_x(Units::negInfinity()),
           position_y(Units::negInfinity()),
           turn_center_x(Units::negInfinity()),
           turn_center_y(Units::negInfinity()) {}
      int iteration_number;
      std::string acid;
      std::string guidance_flight_phase;
      Units::SecondsTime simulation_time;
      Units::DegreesAngle latitude;
      Units::DegreesAngle longitude;
      Units::MetersLength leg_length;
      Units::MetersLength path_length;
      std::string segment_type;
      Units::DegreesAngle enu_course_start;
      Units::DegreesAngle enu_course_end;
      Units::MetersLength turn_radius;
      Units::MetersPerSecondSpeed groundspeed;
      Units::DegreesAngle bank_angle;
      Units::DegreesAngle turn_center_latitude;
      Units::DegreesAngle turn_center_longitude;
      Units::MetersLength position_x;
      Units::MetersLength position_y;
      Units::MetersLength turn_center_x;
      Units::MetersLength turn_center_y;
   };
   static EarthModel::LocalPositionEnu ConvertPosition(std::shared_ptr<TangentPlaneSequence> position_converter,
                                                       const aaesim::LatitudeLongitudePoint &wgs84_position);
   void DataCollector(int iteration_number, std::string acid, const Units::Time &prediction_time,
                      const aaesim::open_source::GuidanceFlightPhase &guidance_flight_phase,
                      const std::vector<Wgs84HorizontalPathSegment> &horizontal_path,
                      const std::shared_ptr<TangentPlaneSequence> &tangent_plane_sequence);

   std::vector<DataToWrite> m_kinetic_horizontal_prediction_data;
};

inline EarthModel::LocalPositionEnu HorizontalPredictionWriter::ConvertPosition(
      std::shared_ptr<TangentPlaneSequence> position_converter, const aaesim::LatitudeLongitudePoint &wgs84_position) {
   EarthModel::GeodeticPosition position;
   position.latitude = wgs84_position.GetLatitude();
   position.longitude = wgs84_position.GetLongitude();
   EarthModel::LocalPositionEnu enu_position;
   position_converter->ConvertGeodeticToLocal(position, enu_position);
   return enu_position;
}
}  // namespace aaesim