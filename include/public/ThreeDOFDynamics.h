// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2019 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include "aaesim/BadaWithCalc.h"
#include "public/EquationsOfMotionStateDeriv.h"
#include "public/AircraftControl.h"
#include "public/AircraftState.h"
#include "public/ControlCommands.h"
#include "public/Guidance.h"
#include "public/EquationsOfMotionState.h"
#include "public/InternalObserver.h"
#include "public/LoggingLoadable.h"
#include <string>
#include <map>
#include <Angle.h>
#include <Force.h>
#include <Frequency.h>
#include <Length.h>
#include <Speed.h>


typedef struct
{
   //This structure is not used at all within the EOM function. It only serves to
   //represent the state outside the EOM function. Within the EOM function, the state is
   //represented with the X vector.
   int id;
   double x; //aircraft position east coordinate (m)
   double y; //aircraft position north coordinate (m)
   double h; //aircraft altitude (m)
   double V; // True airspeed (m/s)
   Units::KnotsSpeed v_cas;   // calibrated/indicated airspeed
   double psi; //aircraft heading angle measured from east counter-clockwise (rad)
   double phi;  //aircraft roll angle (rad)
   double gamma; //aircraft flight-path angle (rad) NOTE: for gamma, heading down is positive; heading up is negative
   double thrust;  //thrust (N)
   double speed_brake; //speed brake (% of deployment)-currently unused
   double xd; //ground speed x component (m/s)
   double yd; //ground speed y component (m/s)
   int m_flap_configuration; // 0, 1, 2, or 3 for flaps speed.
} DynamicsState;


// State in State out replacement for AircraftEOM aircraft dynamics model
class ThreeDOFDynamics : public LoggingLoadable
{
public:
   ThreeDOFDynamics(void);

   /**
    * Aircraft update method that calculates the new aircraft state from the given command state
    *
    * @param aircraft_state
    * @param guidance
    * @param aircraft_control
    * @return
    */
   virtual AircraftState Update(const AircraftState &aircraft_state,
                                const Guidance &guidance,
                                std::shared_ptr<AircraftControl> aircraft_control);

   // load method to read in the Dynamics values
   bool load(DecodedStream *input);

   // method to check if the model loaded properly
   bool IsLoaded() const;

   void Initialize(const double mass_percentile,
                   Units::Length altitude_msl_at_faf,
                   Units::Speed initial_tas,
                   std::shared_ptr<TangentPlaneSequence> tangent_plane_sequence,
                   const EarthModel::LocalPositionEnu &initial_enu_position,
                   const double initial_heading,
                   const WeatherTruth &true_weather);

   const std::string &GetBadaAcType() const;

   void SetBadaAcType(const std::string &acType);

   const std::pair<Units::Speed, Units::Speed> GetWindComponents() const;

   DynamicsState m_dynamics_state;

private:
   static log4cplus::Logger m_logger;

   EquationsOfMotionState m_equations_of_motion_state;

   // integrate method to integrate the command vector into the aircraft state
   AircraftState Integrate(const Guidance &guidance,
                           std::shared_ptr<AircraftControl> aircraft_controller);

   static std::string GetTrueWindAsCsvString(const WindStack &wind_x,
                                             const WindStack &wind_y);

   // Calculate the trim angle correction necessary and provides an updated state
   Units::UnsignedRadiansAngle CalculateTrimmedPsiForWind(Units::Angle psi);

   EquationsOfMotionStateDeriv StatePropagation(const Units::Frequency dVwx_dh,
                                                const Units::Frequency dVwy_dh,
                                                const Units::Frequency k_gamma,
                                                const Units::Frequency k_t,
                                                const Units::Frequency k_phi,
                                                const double k_speedBrake,
                                                const ControlCommands commands);

   void CalculateKineticForces(Units::Force &lift, Units::Force &drag);

   void CalculateEnvironmentalWind(WindStack &wind_x,
                                   WindStack &wind_y,
                                   Units::Frequency &dVwx_dh,
                                   Units::Frequency &dVwy_dh);

   std::string m_ac_type;

   bool m_model_loaded;

   BadaWithCalc m_bada_calculator;

   Units::Speed m_wind_velocity_x, m_wind_velocity_y;  // True wind direction values computed by dynamics and speed_on_pitch_control_dynamics

   double m_max_thrust_percent;
   double m_min_thrust_percent;
   Units::Length m_altitude_msl_at_final_waypoint;
   std::shared_ptr<TangentPlaneSequence> m_tangent_plane_sequence;
   WeatherTruth m_true_weather;
};

inline const std::pair<Units::Speed, Units::Speed> ThreeDOFDynamics::GetWindComponents() const {
   return std::make_pair(m_wind_velocity_x, m_wind_velocity_y);
};