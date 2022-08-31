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

#include "aaesim/BadaPerformanceCalculator.h"

using namespace aaesim;

// Gets configuration when flying trajectory
//-----------------
// INPUTS:
//   ac: aircraft parameter structure
//   Calibrated air speed, V_cas: kts
//   alt: altitude in meters.
//   altAtFAF: altitude at FAF (or
//   last waypoint) in meters.
// OUTPUTS:
//   cd0,cd2,gear: drag coefficients for the specific configuration
void BadaPerformanceCalculator::GetDragCoefficientsAndIncrementFlapConfiguration(
   const Units::Speed &calibrated_airspeed, /** [in] Calibrated airspeed in knots */
      const Units::Length &altitude_msl,    /** [in] Altitude in meters */
      double &cd0,        /** [out] parasitic drag coefficient */
      double &cd2,        /** [out] induced drag coefficient */
      double &gear,       /** [out] landing gear drag coefficient */
      aaesim::open_source::bada_utils::FlapConfiguration &updated_flap_setting           /** [out] flap configuration */) {

}



// Gets configuration for trajectory building.
//-----------------
// INPUTS:
//   ac: aircraft parameter structure
//   Calibrated air speed, V_cas: kts
//   alt: altitude in meters.
//   altAtFAF: altitude at FAF (or
//   last waypoint) in meters.
// OUTPUTS:
//   cd0,cd2,gear: drag coefficients for the specific configuration
void BadaPerformanceCalculator::GetDragCoefficients(
      const Units::Speed &calibrated_airspeed,     /** [in] Calibrated airspeed in knots */
      const Units::Length &altitude_msl,             /** [in] Altitude in meters */
      const aaesim::open_source::bada_utils::FlapConfiguration &current_flap_configuration, /** [in] flap configuration  */
      double &cd0,            /** [out] parasitic drag coefficient */
      double &cd2,            /** [out] induced drag coefficient */
      double &gear,           /** [out] landing gear drag coefficient */
      aaesim::open_source::bada_utils::FlapConfiguration &flap_configuration /** [out] flap configuration  */
      ) const {

}

// Configures Aircraft is extra drag is needed and speed is less than max configuration speed
//-----------------
// INPUTS:
//   ac: aircraft parameter structure
//   Calibrated air speed, V_cas: kts
//   alt: altitude in meters.
//   altAtFAF: altitude at FAF (or
//   last waypoint) in meters.
// OUTPUTS:
//   mode: configuration
void BadaPerformanceCalculator::GetConfigurationForIncreasedDrag(
   const Units::Speed &calibrated_airspeed, /** [in] Calibrated airspeed in knots */
      const Units::Length &altitude,         /** [in] Altitude in meters */
      aaesim::open_source::bada_utils::FlapConfiguration &updated_flap_setting) /** [out] flap configuration */
{

}

// gets maximum aircraft thrust
Units::NewtonsForce BadaPerformanceCalculator::GetMaxThrust(const Units::Length &altitude,
                                  aaesim::open_source::bada_utils::FlapConfiguration mode,
                                  aaesim::open_source::bada_utils::EngineThrustMode engine_thrust_mode,
                                  Units::AbsCelsiusTemperature temperature_offset) const {
   return Units::NewtonsForce(0);
}

BadaPerformanceCalculator* BadaPerformanceCalculator::MakeBadaPerformance(
   std::string aircraft_type, /** Aircraft type */
   BoundedValue<double,0,1> mass_percentile, /** Mass percentile: 0.0=mass.m_min, 1.0=mass.m_max */
   Units::Length faf_altitude_msl, /** Used to ensure flaps don't deploy too early */
   open_source::bada_utils::FlapConfiguration initial_flap_configuration /** Initialize flaps configuration */) {
   std::cout << "BadaPerformanceCalculator.cpp: Implement me!" << std::endl;
   return nullptr;
}
