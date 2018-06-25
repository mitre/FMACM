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
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once
#include "framework/TestFrameworkFMS.h"
#include <Angle.h>
#include <Force.h>
#include <Frequency.h>
#include <Length.h>
#include <Speed.h>
#include "AngularSpeed.h"
#include "aaesim/BadaWithCalc.h"


// State in State out replacement for AircraftEOM aircraft dynamics model
class TestFrameworkDynamics : public Loadable
{
public:
	TestFrameworkDynamics(void);
	~TestFrameworkDynamics(void);

	// sets the Dynamics FMS
	void setFms(TestFrameworkFMS *Fms_in);

	// Aircraft update method that calculates the new aircraft state from the given command state
	AircraftState update(AircraftState state_in, Guidance guidance_in);

	// load method to read in the Dynamics values
	bool load(DecodedStream *input);

	// method to check if the model loaded properly
	bool is_loaded(); 

	// initializes 3DOF Dynamics
	void init(double mass_percentile,Units::Length altAtFAF_in, Units::Length initialAltitude,
		  Units::Speed initialIas, double initialMach, double start_time);

	//This structure, state, is not used at all within the EOM function. It only serves to 
	//represent the state outside the EOM function. Within the EOM function, the state is 
	//represented with the X vector (see the above comments).


	struct
	{
      int id;
	  double x; //aircraft position east coordinate (m)
	  double y; //aircraft position north coordinate (m)
	  double h; //aircraft altitude (m)
	  double V; // True airspeed (m/s)
	  Units::KnotsSpeed v_cas;	// calibrated/indicated airspeed
	  double psi; //aircraft heading angle measured from east counter-clockwise (rad)
	  double phi;  //aircraft roll angle (rad)
	  double gamma; //aircraft flight-path angle (rad) NOTE: for gamma, heading down is positive; heading up is negative
	  double T;  //thrust (N)
	  double speed_brake; //speed brake (% of deployment)-currently unused
	  double xd; //ground speed x component (m/s)
	  double yd; //ground speed y component (m/s)
	  int flapConfig; // 0, 1, 2, or 3 for flaps speed.
	} state;

	class Weather {
	public:
		Units::Speed Vwx, Vwy;
		Units::Frequency dVwx_dh, dVwy_dh;
		Units::Temperature temperature;
	};

	/** Retrieves a weather record but does not set the weather field */
	Weather getWeatherFromTime(double time);

	const Units::Speed &getVwx() const;
	const Units::Speed &getVwy() const;

 private:

	static const Units::Time pilot_delay_mean;
	static const Units::Time pilot_delay_std;

	/*
	X is the internal state visible only in this class. Its units are metric.
	The meanings of the elements of X are as follows:
	X[1]: aircraft position east coordinate (m)
	X[2]: aircraft position  north coordinate (m)
	X[3]: aircraft position altitude (m)
	X[4]: aircraft true airspeed (m/s)
	X[5]: aircraft flight-path angle (rad). NOTE: for flight-path angle (gamma), heading down is positive; heading up is negative
	X[6]: aircraft heading (psi) measured from east counter-clockwise (rad).
	X[7]: aircraft thrust (N)
	X[8]: aircraft roll angle (phi) (rad)
	X[9]: aircraft speed brake (% of deployment)
	*/
	class InternalAircraftState {
	public:
		Units::Length x, y, h;	// [1..3] east, north, altitude
		Units::Speed V;			// [4] true airspeed
		Units::Angle gamma;		// [5] flight-path angle NOTE: for flight-path angle (gamma), heading down is positive; heading up is negative
		Units::Angle psi;		// [6] heading measured from east counter-clockwise
		Units::Force T;			// [7] thrust
		Units::Angle phi;		// [8] roll angle
		double speedBrake;		// [9] speed brake (% of deployment)
		int flapConfig;			// [10] flap configuration
	};
	// derivative
	class InternalAircraftStateD {
	public:
		Units::Speed dx, dy, dh;	// [1..3] east, north, altitude change
		Units::Acceleration dV;		// [4] true airspeed change
		Units::AngularSpeed dgamma;	// [5] flight-path angle change NOTE: for flight-path angle (gamma), heading down is positive; heading up is negative
		Units::AngularSpeed dpsi;	// [6] heading change measured from east counter-clockwise
		Units::ForceChange dT;		// [7] thrust change
		Units::AngularSpeed dphi;	// [8] roll angle change
		double dspeedBrake;			// [9] speed brake (% of deployment) change rate
		int flapConfig;				// [10] new flap configuration
	};

	InternalAircraftState X;

	// integrate method to integrate the command vector into the aircraft state
	AircraftState integrate(Guidance guidance_in );

	// EOM dynamics call to calculate the dX values
	InternalAircraftStateD speed_on_thrust_control_dynamics(Guidance guidance_in);

	// EOM new VNAV_speed_control dynamics model
	InternalAircraftStateD speed_on_pitch_control_dynamics(Guidance guidance_in);

	// helper method to add a new guidance command to the pilot delay buffer
	void add_to_pilot_delay(Guidance guidance_in, double time);
	
	// helper method to get the current command from the delay buffer
	Guidance get_pilot_delay_guidance(Guidance prev_guidance, double time);

	void setWeatherFromTime(Units::Time time);

	void load_env_file(std::string env_csv_file);

	TestFrameworkFMS *Fms;

	std::string ac_type_name; // Aircraft type.

	Units::Angle maxBankAngle; // Maximum bank angle for dynamics and VNAV_dynamics calculations (parameter max_bank_angle).

	AircraftState state_vector;
	bool model_loaded;

	BadaWithCalc ac;
	Guidance prev_guidance; // previous Guidance to be used if no new command
	Units::Length alt_thresh; // (m)
	Units::Speed speed_thresh; // (m/s)
	std::string speed_management_type;

	double maxThrustPercent;
	double minThrustPercent;

	int modeLast;

	// Speed brake members

	double minThrustCounter;
	double speedBrakeCounter;
	bool speedBrakeOn;
	
	bool levelFlight;

	std::map<int, Guidance> pilot_delay_buffer; // Pilot Delay Buffer keyed on command time

	//Units::Speed Vwx, Vwy;  // True wind direction values computed by dynamics and VNAV_dynamics (m/s).
	Weather *weather;
	Units::Speed Vw_para, Vw_perp; // True wind factors computed by dynamics and VNAV_dynamics (m/s).

	Units::Length altAtFAF; // Alt at FAF (last waypoint) in meters.

	std::map<Units::Time,Weather*> weatherByTime;
	std::map<Units::Length,Weather*> weatherByDistanceToGo;

	Units::Speed Vwx, Vwy;  // True wind direction values computed by dynamics and speed_on_pitch_control_dynamics (m/s).
};
