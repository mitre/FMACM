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

#include <vector>
#include <scalar/Angle.h>

#include "public/EuclideanTrajectoryPredictor.h"
#include "avionics/KineticDescent4DPredictor.h"
#include "public/PrecalcWaypoint.h"
#include "public/AircraftIntent.h"
#include "public/Guidance.h"
#include "public/AircraftState.h"
#include "public/Logging.h"

class KineticTrajectoryPredictor : public aaesim::open_source::EuclideanTrajectoryPredictor {
  public:
   KineticTrajectoryPredictor();

   KineticTrajectoryPredictor(Units::Angle max_bank_below_fl195,
                              std::shared_ptr<KineticDescent4DPredictor> vertical_predictor);

   KineticTrajectoryPredictor(const KineticTrajectoryPredictor &obj);

   virtual ~KineticTrajectoryPredictor() = default;

   KineticTrajectoryPredictor &operator=(const KineticTrajectoryPredictor &obj);

   virtual void BuildTrajectoryPrediction(aaesim::open_source::WeatherPrediction &weather,
                                          const std::shared_ptr<TangentPlaneSequence> &position_converter,
                                          Units::Length start_altitude) override;

   void BuildTrajectoryPrediction(const std::string &acPerfName, aaesim::open_source::WeatherPrediction &weather,
                                  const std::shared_ptr<TangentPlaneSequence> &position_converter,
                                  Units::Length start_altitude);

   std::shared_ptr<KineticDescent4DPredictor> GetVertPredictor() const;

   const std::string &GetAircraftType() const;

   void SetAchieveWaypointName(const std::string &achieve_waypoint_name);

   const std::string &GetAchieveWaypointName() const;

  protected:
   std::string m_achieve_waypoint_name;

  private:
   std::string m_aircraft_type;

   static log4cplus::Logger logger;
};

inline void KineticTrajectoryPredictor::SetAchieveWaypointName(const std::string &achieve_waypoint_name) {
   m_achieve_waypoint_name.assign(achieve_waypoint_name);
}

inline const std::string &KineticTrajectoryPredictor::GetAchieveWaypointName() const { return m_achieve_waypoint_name; }

inline std::shared_ptr<KineticDescent4DPredictor> KineticTrajectoryPredictor::GetVertPredictor() const {
   return std::shared_ptr<KineticDescent4DPredictor>(
         std::static_pointer_cast<KineticDescent4DPredictor>(m_vertical_predictor));
}

inline const std::string &KineticTrajectoryPredictor::GetAircraftType() const { return m_aircraft_type; }
