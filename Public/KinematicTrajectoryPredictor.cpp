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

#include "public/KinematicTrajectoryPredictor.h"

using namespace std;
using namespace aaesim::open_source;

log4cplus::Logger KinematicTrajectoryPredictor::m_logger =
      log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("KinematicTrajectoryPredictor"));

KinematicTrajectoryPredictor::KinematicTrajectoryPredictor() {
   m_vertical_predictor = std::shared_ptr<VerticalPredictor>(new KinematicDescent4DPredictor());
}

KinematicTrajectoryPredictor::KinematicTrajectoryPredictor(Units::Angle maximum_bank_angle, Units::Speed transition_ias,
                                                           double transition_mach,
                                                           Units::Length transition_altitude_msl,
                                                           Units::Length cruise_altitude_msl) {
   m_bank_angle = maximum_bank_angle;
   KinematicDescent4DPredictor *kinematic_descent_predictor = new KinematicDescent4DPredictor();
   kinematic_descent_predictor->SetMembers(transition_mach, transition_ias, cruise_altitude_msl,
                                           transition_altitude_msl);
   m_vertical_predictor = std::shared_ptr<VerticalPredictor>(kinematic_descent_predictor);
}

KinematicTrajectoryPredictor::KinematicTrajectoryPredictor(const KinematicTrajectoryPredictor &obj) { operator=(obj); }

void KinematicTrajectoryPredictor::CalculateWaypoints(const AircraftIntent &aircraft_intent,
                                                      const WeatherPrediction &weather_prediction) {
   Units::Length altitude_at_faf = Units::MetersLength(
         aircraft_intent.GetRouteData().m_nominal_altitude[aircraft_intent.GetNumberOfWaypoints() - 1]);
   Units::Speed nominal_ias_at_faf = Units::FeetPerSecondSpeed(
         aircraft_intent.GetRouteData().m_nominal_ias[aircraft_intent.GetNumberOfWaypoints() - 1]);

   GetKinematicDescent4dPredictor()->SetConditionsAtEndOfRoute(altitude_at_faf, nominal_ias_at_faf);
   EuclideanTrajectoryPredictor::CalculateWaypoints(aircraft_intent, WeatherPrediction());
}

KinematicTrajectoryPredictor &KinematicTrajectoryPredictor::operator=(const KinematicTrajectoryPredictor &obj) {
   if (this != &obj) {
      EuclideanTrajectoryPredictor::operator=(obj);

      std::shared_ptr<KinematicDescent4DPredictor> kin = obj.GetKinematicDescent4dPredictor();

      if (kin == NULL) {
         m_vertical_predictor = std::shared_ptr<VerticalPredictor>((KinematicDescent4DPredictor *)NULL);
      } else {
         m_vertical_predictor = std::shared_ptr<VerticalPredictor>(new KinematicDescent4DPredictor(*kin));
      }
   }

   return *this;
}

std::shared_ptr<KinematicDescent4DPredictor> KinematicTrajectoryPredictor::GetKinematicDescent4dPredictor() const {
   return std::shared_ptr<KinematicDescent4DPredictor>(
         static_pointer_cast<KinematicDescent4DPredictor>(m_vertical_predictor));
}
