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

#include "scalar/Length.h"
#include "scalar/Speed.h"
#include "scalar/Mass.h"
#include "public/BadaUtils.h"
#include "utility/BoundedValue.h"

namespace aaesim {
namespace open_source {
struct FixedMassAircraftPerformance {
   /**
    * Calculate expected drag coefficients and flap settings for the incoming state values.
    */
   virtual void GetDragCoefficients(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
         const aaesim::open_source::bada_utils::FlapConfiguration &current_flap_configuration,
         double &cd0,  /** [out] parasitic drag coefficient */
         double &cd2,  /** [out] induced drag coefficient */
         double &gear, /** [out] landing gear drag coefficient */
         aaesim::open_source::bada_utils::FlapConfiguration &flap_configuration /** [out] flap configuration */
   ) const = 0;

   /**
    * Calculate drag coefficients and increments flap configuration when appropriate.
    */
   virtual void GetDragCoefficientsAndIncrementFlapConfiguration(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
         double &cd0,  /** [out] parasitic drag coefficient */
         double &cd2,  /** [out] induced drag coefficient */
         double &gear, /** [out] landing gear drag coefficient */
         aaesim::open_source::bada_utils::FlapConfiguration &updated_flap_setting /** [out] flap configuration */
         ) = 0;

   /**
    * Allow early configuration changes; use when extra drag is needed and speed is less than max configuration speed.
    */
   virtual void GetConfigurationForIncreasedDrag(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
         aaesim::open_source::bada_utils::FlapConfiguration &updated_flap_setting /** [out] flap configuration */
         ) = 0;

   /**
    * Calculate the maximum available thrust in Newtons.
    */
   virtual Units::NewtonsForce GetMaxThrust(const Units::Length &altitude_msl,
                                            aaesim::open_source::bada_utils::FlapConfiguration flap_configuration,
                                            aaesim::open_source::bada_utils::EngineThrustMode engine_thrust_mode,
                                            Units::AbsCelsiusTemperature temperature_offset) const = 0;

   virtual void GetCoefficientsForFlapConfiguration(open_source::bada_utils::FlapConfiguration flap_configuration,
                                                    double &cd0, /** [out] parasitic drag coefficient */
                                                    double &cd2, /** [out] induced drag coefficient */
                                                    double &gear /** [out] landing gear drag coefficient */
   ) const = 0;

   virtual aaesim::open_source::bada_utils::FlapConfiguration GetFlapConfigurationForState(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
         const aaesim::open_source::bada_utils::FlapConfiguration &current_flap_configuration) const = 0;

   virtual Units::Mass GetAircraftMass() const = 0;

   virtual double GetAircraftMassPercentile() const = 0;

   virtual open_source::bada_utils::FlapSpeeds GetFlapSpeeds() const = 0;

   virtual open_source::bada_utils::FlapConfiguration GetCurrentFlapConfiguration() const = 0;

   virtual void UpdateMassFraction(BoundedValue<double, 0, 1> mass_fraction) = 0;

   virtual aaesim::open_source::bada_utils::AircraftType GetAircraftTypeInformation() const = 0;

   virtual aaesim::open_source::bada_utils::Mass GetAircraftMassInformation() const = 0;

   virtual aaesim::open_source::bada_utils::FlightEnvelope GetFlightEnvelopeInformation() const = 0;

   virtual aaesim::open_source::bada_utils::Aerodynamics GetAerodynamicsInformation() const = 0;

   virtual aaesim::open_source::bada_utils::EngineThrust GetEngineThrustInformation() const = 0;

   virtual aaesim::open_source::bada_utils::FuelFlow GetFuelFlowInformation() const = 0;

   virtual aaesim::open_source::bada_utils::GroundMovement GetGroundMovementInformation() const = 0;

   virtual aaesim::open_source::bada_utils::Procedure GetProcedureInformation(unsigned int index) const = 0;

   virtual aaesim::open_source::bada_utils::AircraftPerformance GetAircraftPerformanceInformation() const = 0;

   virtual std::string GetAircraftTypeIdentifier() const = 0;
};
}  // namespace open_source
}  // namespace aaesim
