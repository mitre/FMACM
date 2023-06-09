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

#include <string>
#include <cstring>
#include "Bada.h"
#include "public/BadaUtils.h"
#include "utility/Logging.h"
#include "utility/BoundedValue.h"

namespace aaesim {
class BadaPerformanceCalculator : public Bada {
  public:
   BadaPerformanceCalculator() = default;

   ~BadaPerformanceCalculator() = default;

   /**
    * Calculates expected drag coefficients and flap settings for the incoming state values.
    */
   void GetDragCoefficients(
         const Units::Speed &calibrated_airspeed, /** [in] Calibrated airspeed */
         const Units::Length &altitude_msl,       /** [in] Altitude MSL */
         const aaesim::open_source::bada_utils::FlapConfiguration &current_flap_configuration, /** [in] current flap
                                                                                                  configuration */
         double &cd0,  /** [out] parasitic drag coefficient */
         double &cd2,  /** [out] induced drag coefficient */
         double &gear, /** [out] landing gear drag coefficient */
         aaesim::open_source::bada_utils::FlapConfiguration &flap_configuration /** [out] flap configuration */) const;

   /**
    * Calculates drag coefficients and increments flap configuration when appropriate.
    */
   void GetDragCoefficientsAndIncrementFlapConfiguration(
         const Units::Speed &calibrated_airspeed, /** [in] Calibrated airspeed */
         const Units::Length &altitude_msl,       /** [in] Altitude MSL */
         double &cd0,                             /** [out] parasitic drag coefficient */
         double &cd2,                             /** [out] induced drag coefficient */
         double &gear,                            /** [out] landing gear drag coefficient */
         aaesim::open_source::bada_utils::FlapConfiguration &updated_flap_setting /** [out] flap configuration */);

   /**
    * Allow early configuration changes; used when extra drag is needed and speed is less than max configuration speed.
    */
   void GetConfigurationForIncreasedDrag(
         const Units::Speed &calibrated_airspeed, /** [in] Calibrated airspeed */
         const Units::Length &altitude_msl,       /** [in] Altitude MSL */
         aaesim::open_source::bada_utils::FlapConfiguration &updated_flap_setting /** [out] flap configuration */);

   /**
    * Calculates the maximum available thrust in Newtons.
    */
   Units::NewtonsForce GetMaxThrust(
         const Units::Length &altitude_msl,                                     /** [in] altitude_msl */
         aaesim::open_source::bada_utils::FlapConfiguration flap_configuration, /** [in] flap configuration */
         aaesim::open_source::bada_utils::EngineThrustMode engine_thrust_mode,
         Units::AbsCelsiusTemperature temperature_offset /** [in] "cruise" or "descent" */) const;

   void GetCoefficientsForFlapConfiguration(open_source::bada_utils::FlapConfiguration flap_configuration, double &cd0,
                                            double &cd2, double &gear) const;

   aaesim::open_source::bada_utils::FlapConfiguration GetFlapConfigurationForState(
         const Units::Speed &calibrated_airspeed, const Units::Length &altitude_msl,
         const aaesim::open_source::bada_utils::FlapConfiguration &current_flap_configuration) const;

   Units::Length GetAltitudeAtFaf() const;

   Units::Mass GetAircraftMass() const;

   double GetAircraftMassPercentile() const;

   open_source::bada_utils::FlapSpeeds GetFlapSpeeds() const;

   open_source::bada_utils::FlapConfiguration GetCurrentFlapConfiguration() const;

   bool IsFlapProgressionReversed();

   void UpdateMassFraction(BoundedValue<double, 0, 1> mass_fraction);

   static void SetBadaDataPath(std::string path_to_data);

   static BadaPerformanceCalculator *MakeBadaPerformance(
         std::string aircraft_type,                  /** Aircraft type */
         BoundedValue<double, 0, 1> mass_percentile, /** Mass percentile: 0.0=mass.m_min, 1.0=mass.m_max */
         Units::Length faf_altitude_msl,             /** Used to ensure flaps don't deploy too early */
         open_source::bada_utils::FlapConfiguration initial_flap_configuration /** Initialize flaps configuration */);

  private:
   static log4cplus::Logger m_logger;

   BadaPerformanceCalculator(const Bada *bada, BoundedValue<double, 0, 1> mass_percentile,
                             Units::Length faf_altitude_msl,
                             open_source::bada_utils::FlapConfiguration initial_flap_configuration,
                             bool perform_reverse_flap_progression);

   bool CloseToFinalApproachFixAltitude(const Units::Length &altitude_msl) const;
   open_source::bada_utils::FlapConfiguration PerformReverseFlapProgression(
         open_source::bada_utils::FlapConfiguration current_flap_configuration, Units::Speed calibrated_airspeed,
         const bool near_final_approach_fix) const;
   open_source::bada_utils::FlapConfiguration PerformNormalFlapProgression(
         open_source::bada_utils::FlapConfiguration current_flap_configuration, Units::Speed calibrated_airspeed,
         const bool near_final_approach_fix) const;

  protected:
   open_source::bada_utils::FlapConfiguration m_flap_configuration;
   open_source::bada_utils::FlapSpeeds m_flap_speeds;
   Units::Mass m_aircraft_mass;
   double m_mass_percentile;
   Units::Length m_faf_altitude_msl;
   bool m_perform_reverse_flap_progression;
};

inline Units::Mass BadaPerformanceCalculator::GetAircraftMass() const { return m_aircraft_mass; }

inline double BadaPerformanceCalculator::GetAircraftMassPercentile() const { return m_mass_percentile; }

inline aaesim::open_source::bada_utils::FlapSpeeds BadaPerformanceCalculator::GetFlapSpeeds() const {
   return m_flap_speeds;
}

inline void BadaPerformanceCalculator::SetBadaDataPath(std::string path_to_data) {
   std::strcpy(Bada::input_path, path_to_data.c_str());
}

inline open_source::bada_utils::FlapConfiguration BadaPerformanceCalculator::GetCurrentFlapConfiguration() const {
   return m_flap_configuration;
}

inline Units::Length BadaPerformanceCalculator::GetAltitudeAtFaf() const { return m_faf_altitude_msl; }

inline bool BadaPerformanceCalculator::IsFlapProgressionReversed() { return m_perform_reverse_flap_progression; }

inline open_source::bada_utils::FlapConfiguration BadaPerformanceCalculator::PerformNormalFlapProgression(
      open_source::bada_utils::FlapConfiguration current_flap_configuration, Units::Speed calibrated_airspeed,
      const bool near_final_approach_fix) const {
   // This logic uses these transition rules:
   // -- the flap configuration is defined on the interval [TAKEOFF, GEAR_DOWN];
   // -- the flap configuration can only increment and never decrements;
   // -- the flap configuration can only increment by one step;
   // -- the flap configuration transition occurs as late in the CAS profile as possible
   //    based on the flaps speed schedule. Maximums are not used; only when
   //    calibrated_airspeed has dropped below a minimum does the logic allow a transition.
   if (calibrated_airspeed < m_flap_speeds.cas_climb_minimum &&
       current_flap_configuration == open_source::bada_utils::FlapConfiguration::TAKEOFF) {
      return open_source::bada_utils::FlapConfiguration::TAKEOFF;
   } else if (calibrated_airspeed >= m_flap_speeds.cas_takeoff_minimum &&
              calibrated_airspeed < m_flap_speeds.cas_climb_minimum &&
              current_flap_configuration == open_source::bada_utils::FlapConfiguration::TAKEOFF) {
      return open_source::bada_utils::FlapConfiguration::TAKEOFF;
   } else if (calibrated_airspeed >= m_flap_speeds.cas_climb_minimum &&
              calibrated_airspeed <= m_flap_speeds.cas_cruise_minimum &&
              (current_flap_configuration == open_source::bada_utils::FlapConfiguration::TAKEOFF ||
               current_flap_configuration == open_source::bada_utils::FlapConfiguration::INITIAL_CLIMB)) {
      return open_source::bada_utils::FlapConfiguration::INITIAL_CLIMB;
   } else if (calibrated_airspeed > m_flap_speeds.cas_cruise_minimum &&
              (current_flap_configuration == open_source::bada_utils::FlapConfiguration::INITIAL_CLIMB ||
               current_flap_configuration == open_source::bada_utils::FlapConfiguration::CRUISE)) {
      return open_source::bada_utils::FlapConfiguration::CRUISE;
   } else if ((calibrated_airspeed <= m_flap_speeds.cas_approach_minimum &&
               calibrated_airspeed > m_flap_speeds.cas_landing_minimum) &&
              near_final_approach_fix &&
              (current_flap_configuration == open_source::bada_utils::FlapConfiguration::CRUISE ||
               current_flap_configuration == open_source::bada_utils::FlapConfiguration::APPROACH)) {
      return open_source::bada_utils::FlapConfiguration::APPROACH;
   } else if ((calibrated_airspeed <= m_flap_speeds.cas_landing_minimum &&
               calibrated_airspeed > m_flap_speeds.cas_gear_out_minimum) &&
              near_final_approach_fix &&
              (current_flap_configuration == open_source::bada_utils::FlapConfiguration::APPROACH ||
               current_flap_configuration == open_source::bada_utils::FlapConfiguration::LANDING)) {
      return open_source::bada_utils::FlapConfiguration::LANDING;
   } else if (calibrated_airspeed <= m_flap_speeds.cas_gear_out_minimum && near_final_approach_fix &&
              (current_flap_configuration == open_source::bada_utils::FlapConfiguration::LANDING ||
               current_flap_configuration == open_source::bada_utils::FlapConfiguration::GEAR_DOWN)) {
      return open_source::bada_utils::FlapConfiguration::GEAR_DOWN;
   }
   return current_flap_configuration;
}

inline open_source::bada_utils::FlapConfiguration BadaPerformanceCalculator::PerformReverseFlapProgression(
      open_source::bada_utils::FlapConfiguration current_flap_configuration, Units::Speed calibrated_airspeed,
      const bool near_final_approach_fix) const {
   // This logic uses these transition rules:
   // -- the flap configuration is defined on the interval [GEAR_DOWN, CRUISE];
   // -- the flap configuration can only decrement and never increments;
   // -- the flap configuration can now decrement multiple steps;
   // -- the flap configuration transition occurs as early in the CAS profile as possible
   //    based on the flaps speed schedule.

   bool advanced(true);
   while (advanced) {
      open_source::bada_utils::FlapConfiguration previous_flap_configuration(current_flap_configuration);
      if (current_flap_configuration == open_source::bada_utils::FlapConfiguration::GEAR_DOWN &&
          calibrated_airspeed < m_flap_speeds.cas_gear_out_minimum && near_final_approach_fix) {
         current_flap_configuration = open_source::bada_utils::FlapConfiguration::GEAR_DOWN;
      } else if (current_flap_configuration == open_source::bada_utils::FlapConfiguration::GEAR_DOWN &&
                 (calibrated_airspeed >= m_flap_speeds.cas_gear_out_maximum ||
                  calibrated_airspeed < m_flap_speeds.cas_landing_minimum) &&
                 near_final_approach_fix) {
         current_flap_configuration = open_source::bada_utils::FlapConfiguration::LANDING;
      } else if (near_final_approach_fix &&
                 (current_flap_configuration == open_source::bada_utils::FlapConfiguration::LANDING ||
                  current_flap_configuration == open_source::bada_utils::FlapConfiguration::APPROACH) &&
                 (calibrated_airspeed >= m_flap_speeds.cas_landing_maximum ||
                  calibrated_airspeed < m_flap_speeds.cas_approach_minimum)) {
         current_flap_configuration = open_source::bada_utils::FlapConfiguration::APPROACH;
      } else if ((current_flap_configuration == open_source::bada_utils::FlapConfiguration::APPROACH ||
                  current_flap_configuration == open_source::bada_utils::FlapConfiguration::CRUISE) &&
                 calibrated_airspeed >= m_flap_speeds.cas_approach_minimum) {
         current_flap_configuration = open_source::bada_utils::FlapConfiguration::CRUISE;
      } else {
         // None of the above rules fired:  Exit the loop.
         advanced = false;
      }
      if (current_flap_configuration == previous_flap_configuration) {
         // Flap configuration did not change:  Exit the loop.
         advanced = false;
      }
   }  // end while (advanced)
   return current_flap_configuration;
}

inline void BadaPerformanceCalculator::UpdateMassFraction(BoundedValue<double, 0, 1> mass_fraction) {
   m_mass_percentile = mass_fraction;
   m_aircraft_mass = (m_mass.m_max - m_mass.m_min) * m_mass_percentile + m_mass.m_min;
}
}  // namespace aaesim
