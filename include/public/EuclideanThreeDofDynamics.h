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

#include "public/ThreeDOFDynamics.h"

namespace aaesim {
   namespace open_source {
      class EuclideanThreeDofDynamics : public ThreeDOFDynamics {
       public:

         EuclideanThreeDofDynamics() = default;

         ~EuclideanThreeDofDynamics() = default;

         AircraftState Update(const Guidance &guidance,
                              const std::shared_ptr<AircraftControl> &aircraft_control) override;

         void Initialize(std::shared_ptr<const BadaPerformanceCalculator> aircraft_performance,
                         const Waypoint &initial_position,
                         std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                         Units::Length initial_altitude_msl,
                         Units::Speed initial_true_airspeed,
                         Units::Angle initial_ground_course_enu,
                         double initial_mass_fraction,
                         const WeatherTruth &true_weather,
                         const std::string &aircraft_type) override;
       protected:
         void CalculateEnvironmentalWind(WindStack &wind_east,
                                         WindStack &wind_north,
                                         Units::Frequency &dVwx_dh,
                                         Units::Frequency &dVwy_dh) override;
       private:
         static log4cplus::Logger m_logger;
         std::shared_ptr<TangentPlaneSequence> m_tangent_plane_sequence;

      };

   }
}