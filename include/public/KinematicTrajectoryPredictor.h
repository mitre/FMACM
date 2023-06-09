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
// 2022 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <vector>
#include <scalar/Angle.h>
#include "public/EuclideanTrajectoryPredictor.h"
#include "public/KinematicDescent4DPredictor.h"
#include "public/PrecalcWaypoint.h"
#include "public/HorizontalPath.h"
#include "public/AircraftIntent.h"
#include "public/Guidance.h"

namespace aaesim {
namespace open_source {
class KinematicTrajectoryPredictor : public aaesim::open_source::EuclideanTrajectoryPredictor {
  public:
   KinematicTrajectoryPredictor();

   KinematicTrajectoryPredictor(Units::Angle maximum_bank_angle, Units::Speed transition_ias, double transition_mach,
                                Units::Length transition_altitude_msl, Units::Length cruise_altitude_msl);

   KinematicTrajectoryPredictor(const KinematicTrajectoryPredictor &obj);

   virtual ~KinematicTrajectoryPredictor() = default;

   void CalculateWaypoints(const AircraftIntent &aircraft_intent,
                           const WeatherPrediction &weather_prediction) override final;

   KinematicTrajectoryPredictor &operator=(const KinematicTrajectoryPredictor &obj);

   bool operator==(const KinematicTrajectoryPredictor &obj) const;

   bool operator!=(const KinematicTrajectoryPredictor &boj) const;

   const std::vector<double> &GetVerticalPathDistances() const;

   const double GetVerticalPathDistanceByIndex(int index) const;

   const std::vector<double> &GetVerticalPathTimes() const;

   const double GetVerticalPathTimeByIndex(int index) const;

   const std::vector<double> &GetVerticalPathGroundspeeds() const;

   const std::vector<double> &GetVerticalPathVelocities() const;

   // Deprecated
   const double GetVerticalPathVelocityByIndex(int index) const;

   const Units::Speed GetVerticalPathCasByIndex(int index) const;

   const std::vector<double> &GetVerticalPathAltitudes() const;

   const double GetVerticalPathAltitudeByIndex(const int index) const;

   std::shared_ptr<aaesim::open_source::KinematicDescent4DPredictor> GetKinematicDescent4dPredictor() const;

  private:
   static log4cplus::Logger m_logger;
};
}  // namespace open_source
}  // namespace aaesim

inline const std::vector<double> &aaesim::open_source::KinematicTrajectoryPredictor::GetVerticalPathDistances() const {
   return m_vertical_predictor->GetVerticalPath().along_path_distance_m;
}

inline const double aaesim::open_source::KinematicTrajectoryPredictor::GetVerticalPathDistanceByIndex(int index) const {
   return m_vertical_predictor->GetVerticalPath().along_path_distance_m[index];
}

inline const std::vector<double> &aaesim::open_source::KinematicTrajectoryPredictor::GetVerticalPathTimes() const {
   return m_vertical_predictor->GetVerticalPath().time_to_go_sec;
}

inline const double aaesim::open_source::KinematicTrajectoryPredictor::GetVerticalPathTimeByIndex(int index) const {
   return m_vertical_predictor->GetVerticalPath().time_to_go_sec[index];
}

inline const std::vector<double> &aaesim::open_source::KinematicTrajectoryPredictor::GetVerticalPathGroundspeeds()
      const {
   return m_vertical_predictor->GetVerticalPath().gs_mps;
}

inline const std::vector<double> &aaesim::open_source::KinematicTrajectoryPredictor::GetVerticalPathVelocities() const {
   return m_vertical_predictor->GetVerticalPath().cas_mps;
}

inline const double aaesim::open_source::KinematicTrajectoryPredictor::GetVerticalPathVelocityByIndex(int index) const {
   return m_vertical_predictor->GetVerticalPath().cas_mps[index];
}

inline const Units::Speed aaesim::open_source::KinematicTrajectoryPredictor::GetVerticalPathCasByIndex(
      int index) const {
   return Units::MetersPerSecondSpeed(m_vertical_predictor->GetVerticalPath().cas_mps[index]);
}

inline const std::vector<double> &aaesim::open_source::KinematicTrajectoryPredictor::GetVerticalPathAltitudes() const {
   return m_vertical_predictor->GetVerticalPath().altitude_m;
}

inline const double aaesim::open_source::KinematicTrajectoryPredictor::GetVerticalPathAltitudeByIndex(
      const int index) const {
   return m_vertical_predictor->GetVerticalPath().altitude_m[index];
}
