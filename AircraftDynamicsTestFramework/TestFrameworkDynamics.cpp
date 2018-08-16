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

#include <cstring>
#include "framework/TestFrameworkDynamics.h"
#include "math/CustomMath.h"
#include "public/SimulationTime.h"
#include "public/AircraftCalculations.h"
#include "math/DMatrix.h"
#include "utility/CustomUnits.h"
#include <Area.h>
#include <Density.h>
#include <SignedAngle.h>
#include <public/CoreUtils.h>
#include "public/Scenario.h"

using namespace std;

const Units::Time TestFrameworkDynamics::pilot_delay_mean = Units::SecondsTime(0.0);
const Units::Time TestFrameworkDynamics::pilot_delay_std = Units::SecondsTime(0.0);


TestFrameworkDynamics::TestFrameworkDynamics(void)
{
	Fms = NULL;
	model_loaded = false;
	ac_type_name = "";
	maxBankAngle = Units::RadiansAngle(Units::DegreesAngle(30.0));

	maxThrustPercent = 1.0;
	minThrustPercent = 1.0;

	alt_thresh = Units::FeetLength(500); // default altitude threshold is 500 feet
	speed_thresh = Units::KnotsSpeed(10);// default speed threshold of 10 Knots
	speed_management_type = "thrust";

	minThrustCounter = 0.0;
	speedBrakeCounter = 0.0;
	speedBrakeOn = false;
	levelFlight = true;

	modeLast = 0;

/*	weatherByTime = new std::map<Units::Time,Weather>();
	weatherByDistanceToGo = new std::map<Units::Length,Weather>();*/

	weather = NULL;

	Vw_para = Units::MetersPerSecondSpeed(0.0);
	Vw_perp = Units::MetersPerSecondSpeed(0.0);

	altAtFAF = Units::FeetLength(-100.0);

	// Default flaps speed values

	state.flapConfig = 0;
	mBadaWithCalc.setFlapSpeeds("");
}

TestFrameworkDynamics::~TestFrameworkDynamics(void)
{
}

// Aircraft update method that calculates the new aircraft state from the given command state
AircraftState TestFrameworkDynamics::update(const AircraftState state_in, const Guidance guidance_in)
{
	AircraftState result_state;
	Guidance pilot_delay;

	Units::Time dt0 = SimulationTime::get_simulation_time_step(); // gets simulation time step as difference in time
	Units::Time time = Units::SecondsTime(state_in.time) + dt0;
	setWeatherFromTime(time);

	// add current command to the pilot delay buffer
	add_to_pilot_delay(guidance_in, state_in.time);

	// get the aircraft guidance for the current time accounting for pilot delay
	prev_guidance = get_pilot_delay_guidance(prev_guidance, state_in.time);
	// TODO ? guidance_in = prev_guidance;

	if (InternalObserver::getInstance()->debugTrueWind()) {
		InternalObserver::getInstance()->setTrueWindHdrVals(state_in.time,state_in.id);
	}

	result_state = integrate(guidance_in);

	return result_state;
}

// integrate method to integrate the command vector into the aircraft state
AircraftState TestFrameworkDynamics::integrate(Guidance guidance_in)
{
	AircraftState result_state;

	//first-order derivative of the state calculated by the EOM
	//The units are metric.
	InternalAircraftStateD dX;

	Units::SecondsTime dt = SimulationTime::get_simulation_time_step(); // gets simulation time step as difference in time

	//call EOM speed_on_thrust_control_dynamics model to calculate the change per second for the aircraft states
	if(speed_management_type == "thrust" )
	{
		dX = speed_on_thrust_control_dynamics(guidance_in);
	}
	else if( speed_management_type == "pitch" )
	{
		dX = speed_on_pitch_control_dynamics(guidance_in);
	}

	//Integrate the state:
	X.x += dX.dx * dt;
	X.y += dX.dy * dt;
	X.h += dX.dh * dt;
	X.V += dX.dV * dt;
	X.gamma += dX.dgamma * dt;
	X.psi += dX.dpsi * dt;
	X.T += dX.dT * dt;
	X.phi += dX.dphi * dt;
	X.speedBrake += dX.dspeedBrake * dt.value();
	X.flapConfig = dX.flapConfig;

	//% States
	state.x = Units::MetersLength(X.x).value();			        // east (m)
	state.y = Units::MetersLength(X.y).value();			        // north (m)
	state.h = Units::MetersLength(X.h).value();			        // altitude (m)
	state.V = Units::MetersPerSecondSpeed(X.V).value();			// true airspeed (m/s)
	state.gamma = Units::RadiansAngle(X.gamma).value();			// flight-path angle (rad): NOTE: for gamma, heading down is positive; heading up is negative
	state.psi = Units::RadiansAngle(X.psi).value();			// heading angle measured from east counter-clockwise (rad); NOTE: use mathematical angle, not aviation heading
	state.T = Units::NewtonsForce(X.T).value();				// thrust (N)
	state.phi = Units::RadiansAngle(X.phi).value();			// roll angle (rad)
	state.speed_brake = X.speedBrake;		// speed brake (% of deployment)-currently unused
	state.flapConfig = int (X.flapConfig+0.1);	// current flap configuration

	// Use xdot and ydot from Dynamics to ensure winds are used
	Units::Speed xdot = dX.dx;  //ground speed (m/s)
	Units::Speed ydot = dX.dy;  //ground speed (m/s)

	//gwang 2013-10
	state.xd = Units::MetersPerSecondSpeed(xdot).value(); //ground speed x component (m/s)
	state.yd = Units::MetersPerSecondSpeed(ydot).value(); //ground speed y component (m/s)
	//end gwang

	// LAW: Check Thrust Limits and Limit Appropriately
	Units::Speed v_cas = ATMOSPHERE()->TAS2CAS(Units::MetersPerSecondSpeed(state.V), Units::MetersLength(state.h)); // current indicated airspeed in meters per second

	double cd0,cd2;
	int mode;
	double gear;
	Units::MetersLength state_h(state.h);
	mBadaWithCalc.getConfig(v_cas, state_h, state.flapConfig, cd0, cd2, gear, mode);
	Units::Force maxThrust = Units::NewtonsForce(mBadaWithCalc.getMaxThrust(state_h, mode, "cruise"));
	Units::Force minThrust = Units::NewtonsForce(mBadaWithCalc.getMaxThrust(state_h, mode, "descent"));

	if(state.T > Units::NewtonsForce(maxThrust).value())
	{
		state.T = Units::NewtonsForce(maxThrust).value();
	}
	else if(state.T  < Units::NewtonsForce(minThrust).value())
	{
		state.T = Units::NewtonsForce(minThrust).value();
	}

	if(state.speed_brake > 0.5)
	{
		state.speed_brake = 0.5;
	}
	else if(state.speed_brake < 0.0)
	{
		state.speed_brake = 0.0;
	}

	// assign return values. 
	result_state.x = state.x/FT_M; //(ft)
	result_state.y = state.y/FT_M; //(ft)
	result_state.z = state.h/FT_M; //(ft)
	result_state.set_psi(state.psi); //(radian)
	result_state.xd = Units::FeetPerSecondSpeed(xdot).value(); //(ft/s)
	result_state.yd = Units::FeetPerSecondSpeed(ydot).value(); //(ft/s)
	result_state.zd = -state.V/FT_M * sin(state.gamma); //(ft/s) Note: for gamma, heading down is positive
	result_state.Vwx = Units::MetersPerSecondSpeed(weather->Vwx).value();
	result_state.Vwy = Units::MetersPerSecondSpeed(weather->Vwy).value();
	result_state.Vw_para = Units::MetersPerSecondSpeed(Vw_para).value();
	result_state.Vw_perp = Units::MetersPerSecondSpeed(Vw_perp).value();

	return result_state;
}

// EOM speed_on_thrust_control_dynamics call to calculate the dX values

TestFrameworkDynamics::InternalAircraftStateD TestFrameworkDynamics::speed_on_thrust_control_dynamics(Guidance guidance_in)
{
	InternalAircraftStateD dX; //units: metric

//	int m = Wind::get_flight_level_lower_bound() + 1;
//	int n = Wind::get_flight_level_upper_bound() + 1;
//	DMatrix wind_x(m, n, 1, 2);
//	DMatrix wind_y(m, n, 1, 2);

	double lat = 0, lon = 0;	// dummy parameters

//	Wind::interpolate_true_wind(false, lat, lon, state.x/FT_M, state.y/FT_M, state.h/FT_M, wind_x, wind_y);
//
//	if (InternalObserver::getInstance()->debugTrueWind()) {
//		InternalObserver::getInstance()->writeTrueWind(this->trueWindCsvString(wind_x,wind_y));
//	}


	// Control Gains used in tracking desired states and commanded inputs
	Units::InvertedLength k_xtrk = Units::PerMeterInvertedLength(5e-4);  // meters^-1
	double k_trk = 3;      // unitless
	Units::Frequency k_alt   = Units::HertzFrequency(0.20); // 1/sec
	Units::Frequency k_gamma = Units::HertzFrequency(0.20); // 1/sec
	Units::Frequency k_phi = Units::HertzFrequency(0.40);   // 1/sec
	//Units::Frequency k_t = 0.70;     // 1/sec

	// new gain control values 2/20/2013
	double zeta = 0.88;
	Units::Frequency wn = Units::HertzFrequency(0.20);
	Units::Frequency k_t = 2 * zeta * wn; // new thrust gain, roughly .352
	Units::Frequency k_v = Units::sqr(wn)/k_t; // new velocity gain, roughly .117
	double k_i = 0.0;//0.005; // velocity error gain
	double k_speedBrake = 0.20;

	//% States: 
	Units::Length x = X.x;           // aircraft position east coordinate (m)
	Units::Length y = X.y;           // aircraft position north coordinate (m)
	Units::Length h = X.h;           // aircraft altitude (m)
	Units::Speed V = X.V;           // true airspeed (m/s)
	Units::Angle gamma = X.gamma;       // flight-path angle (rad)
	Units::Angle psi = X.psi;         // heading angle measured from east counter-clockwise (rad)
	Units::Force T = X.T;           // thrust (N)
	Units::Angle phi = X.phi;         // roll angle (rad)
	double speedBrake = X.speedBrake;  // speed brake (% of deployment)
	int flapConfig = (int) (X.flapConfig+0.1); // flap configuration

	double speedBrakeCom = 0.0;

	// winds and gradients are already set in weather

	// Commanded Track Angle
	Units::Angle trk = Units::RadiansAngle(guidance_in.psi); // GUIDANCE

	Vw_para = weather->Vwx * cos(trk) + weather->Vwy * sin(trk);
	Vw_perp = -weather->Vwx * sin(trk) + weather->Vwy * cos(trk);

	Units::Speed W = sqrt(Units::sqr(weather->Vwx) + Units::sqr(weather->Vwy));
	Units::Speed gs = sqrt(Units::sqr(V*cos(gamma)) - Units::sqr(Vw_perp)) + Vw_para;

	double temp = (Units::sqr(V*cos(gamma)) + Units::sqr(gs) - Units::sqr(W))/(V*2*cos(gamma)*gs);

	// Limit temp so acos function doesn't give undefined value.

	if (temp > 1.0)
		temp = 1.0;
	else if (temp < -1.0)
		temp = -1.0;

	Units::Angle beta = Units::RadiansAngle(acos(temp)) * -1.0 * CoreUtils::sign(Units::MetersPerSecondSpeed(Vw_perp).value());

	//Lateral Control

	// Convert track guidance to heading using winds (beta is the Wind Correction Angle)
	Units::Angle headingCom = trk + beta;

	// Error in heading angle
	Units::SignedAngle e_trk = headingCom - psi;
	e_trk.normalize();

	// Along-path distance and Cross-track Error
	Units::Length e_xtrk = Units::MetersLength(0.0);
	double dynamic_cross = 1.0;

	// check if guidance has has cross track error and use it if so
	if (guidance_in.use_cross_track)
	{
		e_xtrk = Units::MetersLength(guidance_in.cross_track);
	}

	// Calculate commanded roll angle
	Units::Angle phi_com = -k_xtrk*e_xtrk * Units::ONE_RADIAN_ANGLE - k_trk*e_trk; // CONTROL
	double unlimited_phi_com = Units::RadiansAngle(phi_com).value();

	// Limit the commanded roll angle
	double sign_phi_com = CoreUtils::sign(unlimited_phi_com);
	if (phi_com * sign_phi_com > maxBankAngle)
	{
		phi_com = maxBankAngle * sign_phi_com;
	}

	InternalObserver::getInstance()->cross_output(Units::MetersLength(x).value(),
												  Units::MetersLength(y).value(),
												  dynamic_cross, guidance_in.cross_track,guidance_in.psi, unlimited_phi_com,
												  Units::RadiansAngle(phi_com).value() );

	//% Vertical Control
	Units::Speed hdot_ref = Units::FeetPerSecondSpeed(guidance_in.altitude_rate);

	//double 	alt_ref = 10000*FT_M;   // GUIDANCE
	Units::Length alt_ref = Units::FeetLength(guidance_in.reference_altitude);   // GUIDANCE

	Units::Length e_alt = alt_ref - h;

	double temp_gamma = -(hdot_ref + k_alt*e_alt)/V; // calculate change in altitude
	if( temp_gamma > 1.0 )
	{
		temp_gamma = 1.0;
	}
	else if( temp_gamma < -1.0 )
	{
		temp_gamma = -1.0;
	}
	Units::Angle gamma_com = Units::RadiansAngle(asin(temp_gamma)); // CONTROL new 2/20/2013

	//% Speed Control

	Units::Speed tas_com = ATMOSPHERE()->CAS2TAS(
			Units::FeetPerSecondSpeed(guidance_in.indicated_airspeed),
			h);   // GUIDANCE true airspeed
	Units::Speed v_cas = ATMOSPHERE()->TAS2CAS(V, h); // current indicated airspeed in meters per second

	//% Speed Error 
	Units::Speed new_vel_error = tas_com - V;
	Units::Acceleration vel_dot_com = k_v * new_vel_error;

	// Should we limit commanded acceleration?

	//% Thrust to maintain speed 
	// Get temp, density, and pressure
	Units::Density rho;
	Units::Pressure P_tmp;
	ATMOSPHERE()->airDensity(h, rho, P_tmp);
	// Don't bother converting P_tmp from kg/m^2 because we don't need it.

	// Get mBadaWithCalc Configuration
	double cd0,cd2;
	int flapConfig_new;
	double gear;
	mBadaWithCalc.getConfig(v_cas, h, flapConfig, cd0, cd2, gear, flapConfig_new);

	Units::Mass ac_mass = Units::KilogramsMass(mBadaWithCalc.mAircraftMass);
	Units::Area wing_area = Units::MetersArea(mBadaWithCalc.aerodynamics.S);

	// Lift and Drag Coefficients
	//double 	cL = (2*mBadaWithCalc.mass*G)/(rho*V^2*mBadaWithCalc.S);
	// units of cL = kg * m / s^2 / (kg / m^3 * m^2 / s^2 * m^2) = unitless
	double 	cL = (2.*ac_mass * Units::ONE_G_ACCELERATION)/(rho*Units::sqr(V)*wing_area*cos(phi));
	//cD = cd0 + gear + cd2*cL^2;
	double 	cD = cd0 + gear + cd2 * pow(cL, 2);

	if (speedBrake != 0.0)
		cD = (1.0 + 0.6*speedBrake)*cD;

	// Drag
	Units::Force D = 1./2. * rho * cD * Units::sqr(V) * wing_area;


	// Lift
	Units::Force L = 1./2. * rho * cL * Units::sqr(V) * wing_area;

	// Nominal Thrust (no acceleration) at desired speed 
	Units::Force Tnom;
	Tnom = ac_mass*vel_dot_com
		   + D - ac_mass * Units::ONE_G_ACCELERATION*sin(gamma)
		   - ac_mass * V
			 * (weather->dVwx_dh * cos(psi) + weather->dVwy_dh * sin(psi))
			 * sin(gamma)*cos(gamma);


	Units::Force T_com = Tnom;

	// Thrust Limits
	Units::Force maxThrust = Units::NewtonsForce(
			mBadaWithCalc.getMaxThrust(h, flapConfig_new, "cruise"));
	Units::Force minThrust = Units::NewtonsForce(
			mBadaWithCalc.getMaxThrust(h, flapConfig_new, "descent"));
	speedBrakeCom = 0.0;

	// Check Configuration if minThrust is Commanded
	if(T_com < minThrust) {

		T_com = minThrust;

		int mode_new = 0;
		mBadaWithCalc.getConfigForDrag(v_cas, Units::MetersLength(h), flapConfig_new, mode_new);

		flapConfig_new = mode_new;

		minThrustCounter = minThrustCounter + 1;

	}
	else
	{
		minThrustCounter = 0.0;

		// Limit Thrust if T_com exceeds Max Thrust
		if(T_com > maxThrust)
			T_com = maxThrust;

	}

	// Use speed brakes, if necessary
	if( minThrustCounter > 15.0 && new_vel_error < Units::KnotsSpeed(-5.0))
	{
		if( flapConfig_new <= 2 )
		{
			if(!speedBrakeOn)
			{
				speedBrakeCounter = 0.0;
				speedBrakeCom = 0.5;
				speedBrakeOn = true;
			}
			else
			{
				speedBrakeCounter = speedBrakeCounter + 1;
				speedBrakeCom = 0.5;
			}
		}
		else
		{
			speedBrakeCounter = 0.0;
			speedBrakeCom = 0.0;
			speedBrakeOn = false;
		}
	}

	// If no longer commanding Min Thrust
	if( speedBrakeOn && speedBrakeCounter <= 30.0 )
	{
		speedBrakeCounter = speedBrakeCounter + 1;
		speedBrakeCom = 0.5;
	}
	else if ( speedBrakeOn && speedBrakeCounter > 30.0 )
	{
		if( minThrustCounter == 0 )
		{
			speedBrakeCounter = 0.0;
			speedBrakeCom = 0.0;
			speedBrakeOn = false;
		}
		else
		{
			speedBrakeCounter = speedBrakeCounter + 1;
			speedBrakeCom = 0.5;
		}
	}

	// calculate the first-order derivative of the state vector:
	dX.dx = Units::MetersPerSecondSpeed(V*cos(gamma)*cos(psi) + weather->Vwx);
	dX.dy = Units::MetersPerSecondSpeed(V*cos(gamma)*sin(psi) + weather->Vwy);
	dX.dh = Units::MetersPerSecondSpeed(-V*sin(gamma));
	dX.dV = (T-D)/ac_mass + Units::ONE_G_ACCELERATION*sin(gamma)
			+ V*(weather->dVwx_dh*cos(psi) + weather->dVwy_dh*sin(psi))
			  *sin(gamma)*cos(gamma);
	dX.dgamma = k_gamma*(gamma_com - gamma) -
				(weather->dVwx_dh*cos(psi) + weather->dVwy_dh*sin(psi))*pow(sin(gamma), 2) * Units::ONE_RADIAN_ANGLE;
	dX.dpsi = (-L*sin(phi)/(ac_mass*V*cos(gamma)) -
			   (weather->dVwx_dh*sin(psi) - weather->dVwy_dh*cos(psi))*tan(gamma)) * Units::ONE_RADIAN_ANGLE;
	dX.dT = k_t*(T_com - T);
	dX.dphi = k_phi*(phi_com - phi);
	dX.dspeedBrake = k_speedBrake * (speedBrakeCom - speedBrake);
	dX.flapConfig = flapConfig_new;

	return dX;
} // speed_on_thrust_control_dynamics


// sets the Dynamics FMS
void TestFrameworkDynamics::setFms(TestFrameworkFMS *Fms_in)
{
	Fms = Fms_in;
}

// method to check if the model loaded properly
bool TestFrameworkDynamics::is_loaded()
{
	return model_loaded;
}

void TestFrameworkDynamics::init(double mass_percentile, Units::Length altAtFAF_in, Units::Length initialAltitude,
								 Units::Speed initialIas, double initialMach, double start_time)
{
	setWeatherFromTime(Units::SecondsTime(start_time));

	altAtFAF = Units::MetersLength(altAtFAF_in);

	if (altAtFAF < Units::MetersLength(0.0)) {
		std::cout << "WARNING:FAF altitude " << Units::FeetLength(altAtFAF) << " < 0." << std::endl
				  << "Probably was not initialized in precalculated trajectory." << std::endl;
	}

	mBadaWithCalc.getAircraftParameters(ac_type_name, mass_percentile);
	mBadaWithCalc.setFlapSpeeds(ac_type_name);

	double ias_;

	double xdot, ydot;

	//When initializing the state, initial values should be assigned to the X vector, because it is the one that 
	//is used by the EOM function to calculate its first-order derivative.


	// Defining aircraft states	
	// ----------------------------------------------------------------------------
	state.x	            =  Fms->xWp[Fms->NextWp-1]*FT_M;
	state.y		        =  Fms->yWp[Fms->NextWp-1]*FT_M;
	state.h		        =  Units::MetersLength(initialAltitude).value();

	double lat, lon;
	lat = 0;
	lon = 0;

	state.psi		= Fms->get_psi(Fms->NextWp);
	state.gamma		= 0;
	state.phi		= 0;


	// Determine whether aircraft is flying IAS or MACH

	double ias_at_waypoint = Units::FeetPerSecondSpeed(initialIas).value(); //FPS
	double altitude = state.h/FT_M;

	if(initialMach != 0.0) //may fly MACH
	{
		//may need to do mach-cas transition:
		double tas_from_ias =
				Units::FeetPerSecondSpeed(ATMOSPHERE()->CAS2TAS(
						Units::FeetPerSecondSpeed(ias_at_waypoint),
						Units::FeetLength(altitude))).value();

		double tas_from_mach = MachToTas(initialMach, altitude); //FPS
		if(tas_from_ias <= tas_from_mach) //fly ias_at_waypoint
		{
			ias_ = ias_at_waypoint;
		}
		else //fly MACH
		{
			ias_ = MachToCas_MITRE(initialMach, altitude);
		}
	}
	else //fly ias_at_waypoint
	{
		ias_ = ias_at_waypoint;
	}

	// Convert IAS to TAS

	state.V	= Units::MetersPerSecondSpeed(ATMOSPHERE()->CAS2TAS(
			Units::FeetPerSecondSpeed(ias_),
			Units::MetersLength(state.h))).value();

	// Determine Groundspeed
	xdot = state.V*cos(state.psi)*cos(state.gamma)
		   + Units::MetersPerSecondSpeed(weather->Vwx).value(); // mps
	ydot = state.V*sin(state.psi)*cos(state.gamma)
		   + Units::MetersPerSecondSpeed(weather->Vwy).value(); // mps

	state.xd = xdot; // mps
	state.yd = ydot; // mps


	//Actually initialize the state X:
	//% States
	X.x = Units::MetersLength(state.x); // east (m)
	X.y = Units::MetersLength(state.y); // north (m)
	X.h = Units::MetersLength(state.h); // altitude (m)
	X.V = Units::MetersPerSecondSpeed(state.V); // true airspeed (m/s)
	X.gamma = Units::RadiansAngle(state.gamma);    // flight-path angle (rad); NOTE: for gamma, heading down is positve
	X.psi = Units::RadiansAngle(state.psi);      // heading angle measured from east counter-clockwise (rad)
	X.T = Units::NewtonsForce(0.0);            // thrust (N)
	X.phi = Units::RadiansAngle(state.phi);      // roll angle (rad)
	X.speedBrake = 0.0;			   // speed brake (% of deployment)
	X.flapConfig = 0;			   // flap config

	//%Calculate initial Aircraft Thrust
	// Get temp, density, and pressure
	Units::MetersLength state_h(state.h);
	Units::Density rho;
	Units::Pressure P_tmp;
	ATMOSPHERE()->airDensity(state_h, rho, P_tmp);
	// Don't bother converting P_tmp from kg/m^2 because we don't need it.

	// Get mBadaWithCalc Configuration
	double cd0,cd2;
	int mode;
	double gear;
	mBadaWithCalc.getConfig(ATMOSPHERE()->TAS2CAS(Units::MetersPerSecondSpeed(state.V), Units::MetersLength(state.h)),
				 state_h, 0, cd0, cd2, gear, mode);

	Units::Mass ac_mass = Units::KilogramsMass(mBadaWithCalc.mAircraftMass);
	Units::Area wing_area = Units::MetersArea(mBadaWithCalc.aerodynamics.S);

	// Set values for flaps speed.

	state.flapConfig = mode;

	// Set lift and Drag Coefficients

	double 	cL = (2.* ac_mass * Units::ONE_G_ACCELERATION) / (rho * Units::sqr(X.V) * wing_area);
	double 	cD = cd0 + gear + cd2*pow(cL, 2);
	Units::Force D = 1./2. * rho * cD * Units::sqr(X.V) * wing_area; // Drag
	Units::Force L = 1./2. * rho * cL * Units::sqr(X.V) * wing_area; // Lift
	// Nominal Thrust (no acceleration) at desired speed 
	Units::Force Tnom = /*ac_mass*vel_dot_com + VELOCITY ERROR IS 0*/ D - ac_mass * Units::ONE_G_ACCELERATION * sin(asin(0.0/*gamma 0.0*/));// - ac_mass*V*(dVwx_dh*cos(psi) + dVwy_dh*sin(psi))*sin(gamma)*cos(gamma); REMOVED WIND
	// new Thrust commands from model speed change MATLAB code 2/25
	Units::Force maxThrust = Units::NewtonsForce(mBadaWithCalc.getMaxThrust(state_h));
	Units::Force minThrust = Units::NewtonsForce(mBadaWithCalc.getMaxThrust(state_h, mode, "descent"));
	if(Tnom > maxThrust * maxThrustPercent)
	{
		Tnom = maxThrust * maxThrustPercent;
	}
	else if(Tnom  < minThrust * minThrustPercent)
	{
		Tnom = minThrust * minThrustPercent;
	}

	X.T = Tnom; // sets the initial Thrust

	// initialize the previous Guidance to have a default until the first command is processed from pilot delay
	prev_guidance.indicated_airspeed = ias_;
	prev_guidance.psi = state.psi;
	prev_guidance.reference_altitude = state.h/FT_M;
	prev_guidance.altitude_rate = 0;

	// True wind factors (m/s)
	Vw_para = Units::MetersPerSecondSpeed(0.0);
	Vw_perp = Units::MetersPerSecondSpeed(0.0);

}


bool TestFrameworkDynamics::load(DecodedStream *input)
{
	bool results;
	string env_csv_file = "";

	set_stream(input);

	// register the aircraft bada type
	register_var("ac_type", &ac_type_name, true);
	register_var("speed_management_type", &speed_management_type, true);
	register_var("env_csv_file", &env_csv_file, false);

	//do the actual reading:
	results = complete();

	if (ac_type_name.size() != 4) {
		cout << "ac_type was not correct. Found: " << ac_type_name << endl;
		exit(-1);
	}
	if (speed_management_type != "thrust" && speed_management_type != "pitch")
	{
		cout << "speed_management_type was not correct: " << speed_management_type << endl;
		exit(-1);
	}

	model_loaded = results;
	load_env_file(env_csv_file);

	return results;
}

void TestFrameworkDynamics::load_env_file(string env_csv_file) {
	weatherByTime.clear();
	weatherByDistanceToGo.clear();

	if (env_csv_file == "") {
		cout << "No env_csv_file specified; using default weather." << endl;
		// parameter missing, use default
		weather = new Weather();
		weather->Vwx = Units::MetersPerSecondSpeed(0);
		weather->Vwy = Units::MetersPerSecondSpeed(0);
		weather->dVwx_dh = Units::HertzFrequency(0);
		weather->dVwy_dh = Units::HertzFrequency(0);
		weather->temperature = Units::CelsiusTemperature(25);
		weatherByTime[Units::SecondsTime(0)] = weather;
		weatherByDistanceToGo[Units::MetersLength(0)] = weather;
		return;
	}

	FILE *env = fopen(env_csv_file.c_str(), "r");
	if (!env) {
		cout << "Could not open env_csv_file: " << env_csv_file << endl;
		exit(1);
	}

	char line[100];
	fgets(line, sizeof(line), env);	// skip the header line
	while (!feof(env)) {
		double time, distToGo, Vwx, Vwy, dVwx_dh, dVwy_dh, temperature;
		fgets(line, sizeof(line), env);
		if (strlen(line) > 6) {
			sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
				   &time, &distToGo, &Vwx, &Vwy,
				   &dVwx_dh, &dVwy_dh, &temperature);
			weather = new Weather();
			weather->Vwx = Units::MetersPerSecondSpeed(Vwx);
			weather->Vwy = Units::MetersPerSecondSpeed(Vwy);
			weather->dVwx_dh = Units::HertzFrequency(dVwx_dh);
			weather->dVwy_dh = Units::HertzFrequency(dVwy_dh);
			weather->temperature = Units::KelvinTemperature(temperature);
			weatherByTime[Units::SecondsTime(time)] = weather;
			weatherByDistanceToGo[Units::MetersLength(distToGo)] = weather;
		}
	}
	weather = NULL;
/*	cout << "Loaded weather from " << env_csv_file << " (" <<
			weatherByTime.size() << " records)." << endl;*/
	fclose(env);
}

void TestFrameworkDynamics::setWeatherFromTime(Units::Time time) {
	std::map<Units::Time,Weather *>::iterator it = weatherByTime.lower_bound(time);
	Units::Time t1;
	if (it == weatherByTime.end()) {
		std::map<Units::Time,Weather *>::reverse_iterator rit = weatherByTime.rbegin();
		t1 = rit->first;
	}
	else {
		t1 = it->first;
	}
	weather = weatherByTime[t1];
/*
	cout << "Weather set to time=" << Units::SecondsTime(time).value()
			<< ", Vwx=" << Units::MetersPerSecondSpeed(weather->Vwx).value()
			<< ", Vwy=" << Units::MetersPerSecondSpeed(weather->Vwy).value()
			<< ", dVwx/dh=" << Units::HertzFrequency(weather->dVwx_dh).value()
			<< ", dVwy/dh=" << Units::HertzFrequency(weather->dVwy_dh).value()
			<< endl;*/
}

TestFrameworkDynamics::Weather TestFrameworkDynamics::getWeatherFromTime(double time) {
	Units::Time t0 = Units::SecondsTime(time);
	std::map<Units::Time,Weather *>::iterator it = weatherByTime.lower_bound(t0);
	Units::Time t1;
	if (it == weatherByTime.end()) {
		std::map<Units::Time,Weather *>::reverse_iterator rit = weatherByTime.rbegin();
		t1 = rit->first;
	}
	else {
		t1 = it->first;
	}
	Weather w = *weatherByTime[t1];	// copy the object
	return w;
}

// helper method to add a new guidance command to the pilot delay buffer
void TestFrameworkDynamics::add_to_pilot_delay(Guidance guidance_in, double time)
{
	int delayed_time;

#ifdef _LINUX_
	delayed_time = (int) (round(time + Units::SecondsTime(
			Scenario::mRand.rayleighSample(
					TestFrameworkDynamics::pilot_delay_mean,
					TestFrameworkDynamics::pilot_delay_std)).value())+0.1);
#else
	delayed_time = roundToInt(time + Units::SecondsTime(
		  Scenario::mRand.rayleighSample(
				  TestFrameworkDynamics::pilot_delay_mean,
				  TestFrameworkDynamics::pilot_delay_std)).value());
#endif

	// if the delay time is less than the current time, set back to current time
	if( delayed_time < time )
	{
		delayed_time = (int) (time+0.1);
	}

	pilot_delay_buffer.insert(pair<int,Guidance>(delayed_time, guidance_in ));
}

// helper method to get the current command from the delay buffer
Guidance TestFrameworkDynamics::get_pilot_delay_guidance(Guidance prev_guidance, double time)
{
	Guidance result = prev_guidance;
	int current_key = (int) time; // gets the key time for the current time

	map<int, Guidance>::iterator guide_it; // iterator to get the current Guidance

	guide_it = pilot_delay_buffer.find(current_key);

	if(guide_it != pilot_delay_buffer.end())
	{
		result = guide_it->second;
	}

	return result;
}

// EOM new speed_on_pitch_control_dynamics speed_on_thrust_control_dynamics model
TestFrameworkDynamics::InternalAircraftStateD TestFrameworkDynamics::speed_on_pitch_control_dynamics(Guidance guidance_in)
{

	InternalAircraftStateD dX;

//	int m = Wind::get_flight_level_lower_bound() + 1;
//	int n = Wind::get_flight_level_upper_bound() + 1;
//	DMatrix wind_x(m, n, 1, 2);
//	DMatrix wind_y(m, n, 1, 2);

//	double lat = 0, lon = 0;	// dummy parameters

//	Wind::interpolate_true_wind(false, lat, lon, state.x/FT_M, state.y/FT_M, state.h/FT_M, wind_x, wind_y);

//	if (InternalObserver::getInstance()->debugTrueWind()) {
//		InternalObserver::getInstance()->writeTrueWind(this->trueWindCsvString(wind_x,wind_y));
//	}

	// Control Gains used in tracking desired states and commanded inputs
	Units::InvertedLength k_xtrk = Units::PerMeterInvertedLength(5e-4);  // meters^-1
	double k_trk = 3;      // unitless
	Units::Frequency k_alt   = Units::HertzFrequency(0.20); // 1/sec
	Units::Frequency k_gamma = Units::HertzFrequency(0.40); // 1/sec
	Units::Frequency k_phi = Units::HertzFrequency(0.40);   // 1/sec
	//Units::Frequency k_t = Units::HertzFrequency(0.390);     // 1/sec

	// new gain control values 2/20/2013
	double zeta = 0.88;
	Units::Frequency wn = Units::HertzFrequency(0.20);
	Units::Frequency k_t = 2 * zeta * wn; // new thrust gain, roughly .352
	Units::Frequency k_v = Units::sqr(wn)/k_t; // new velocity gain, roughly .117
	double k_i = 0.0;//0.005; // velocity error gain
	double k_speedBrake = 0.10;

	// read in the current aircraft States
	Units::Length x = X.x;           // east (m)
	Units::Length y = X.y;           // north (m)
	Units::Length h = X.h;           // altitude (m)
	Units::Speed V = X.V;           // true airspeed (m/s)
	Units::Angle gamma = X.gamma;       // flight-path angle (rad)
	Units::Angle psi = X.psi;         // heading angle measured from east counter-clockwise (rad)
	Units::Force T = X.T;           // thrust (N)
	Units::Angle phi = X.phi;         // roll angle (rad)
	double speedBrake = X.speedBrake; // speed brake (% of deployment)-currently unused
	int flapConfig = (int) (X.flapConfig+0.1); // flap configuration

	double speedBrakeCom = 0.0;

	// Wind Segment
	// winds are already set as part of the weather object

	Units::Angle trk = Units::RadiansAngle(guidance_in.psi); // GUIDANCE

	Vw_para = weather->Vwx * cos(trk) + weather->Vwy * sin(trk);
	Vw_perp = -weather->Vwx * sin(trk) + weather->Vwy * cos(trk);

	Units::Speed W = sqrt(Units::sqr(weather->Vwx) + Units::sqr(weather->Vwy));
	Units::Speed gs = sqrt(Units::sqr(V*cos(gamma)) - Units::sqr(Vw_perp)) + Vw_para;

	double temp = (Units::sqr(V*cos(gamma)) + Units::sqr(gs) - Units::sqr(W))/(V*2*cos(gamma)*gs);

	// Limit temp so the acos function doesn't give an undefined value.

	if (temp > 1.0)
		temp = 1.0;
	else if (temp < -1.0)
		temp = -1.0;

	Units::Angle beta = Units::RadiansAngle(acos(temp)) * -1.0 * CoreUtils::sign(Units::MetersPerSecondSpeed(Vw_perp).value());

	//Lateral Control

	// Convert track guidance to heading using winds
	Units::Angle headingCom = trk + beta;

	//% Lateral Control

	// Error in track angle limited to +-PI
	Units::SignedAngle e_trk = headingCom - psi;
	e_trk.normalize();

	Units::Length e_xtrk = Units::MetersLength(0);
	double dynamic_cross = 1.0;

	// check if guidance has has cross track error and use it if so
	if (guidance_in.use_cross_track)
	{
		e_xtrk = Units::MetersLength(guidance_in.cross_track);
	}

	// Calculate commanded roll angle
	Units::Angle phi_com = -k_xtrk*e_xtrk * Units::ONE_RADIAN_ANGLE - k_trk*e_trk; // CONTROL
	double unlimited_phi_com = Units::RadiansAngle(phi_com).value();

	// Limit the commanded roll angle
	double sign_phi_com = CoreUtils::sign(unlimited_phi_com);
	if(phi_com * sign_phi_com > maxBankAngle)
	{
		phi_com = maxBankAngle * sign_phi_com;
	}

	Units::Speed v_cas = ATMOSPHERE()->TAS2CAS(Units::MetersPerSecondSpeed(V), Units::MetersLength(h)); // current indicated airspeed in meters per second

	//% Thrust to maintain speed 
	// Get temp, density, and pressure
	Units::Density rho;
	Units::Pressure P_tmp;
	ATMOSPHERE()->airDensity(h, rho, P_tmp);
	// Don't bother converting P_tmp from kg/m^2 because we don't need it.

	// Get mBadaWithCalc Configuration
	double cd0,cd2;
	int flapConfig_new;
	double gear;
	mBadaWithCalc.getConfig(v_cas, h, flapConfig, cd0, cd2, gear, flapConfig_new);

	Units::Mass ac_mass = Units::KilogramsMass(mBadaWithCalc.mAircraftMass);
	Units::Area wing_area = Units::MetersArea(mBadaWithCalc.aerodynamics.S);

	// Lift and Drag Calculations
	double cL = (2.*ac_mass * Units::ONE_G_ACCELERATION)/(rho*Units::sqr(V)*wing_area*cos(phi));
	double cD = cd0 + gear + cd2 * pow(cL, 2);

	if (speedBrake != 0.0) {
		cD = (1.0 + 0.6 * speedBrake) * cD;
	}

	Units::Force D = 1./2. * rho * cD * Units::sqr(V) * wing_area;
	Units::Force L = 1./2. * rho * cL * Units::sqr(V) * wing_area;

	Units::Speed ias_com = Units::FeetPerSecondSpeed(guidance_in.indicated_airspeed);
	Units::Speed tas_com = ATMOSPHERE()->CAS2TAS(ias_com, h);

	Units::Force maxThrust = Units::NewtonsForce(
			mBadaWithCalc.getMaxThrust(h, flapConfig_new, "cruise"));
	Units::Force minThrust = Units::NewtonsForce(
			mBadaWithCalc.getMaxThrust(h, flapConfig_new, "descent"));

	//Speed Management Method check
	Units::Length alt_ref = Units::FeetLength(guidance_in.reference_altitude);
	Units::Length e_alt = h - alt_ref;
	Units::Speed h_dot = Units::FeetPerSecondSpeed(guidance_in.altitude_rate);
	Units::Force T_com = Units::NewtonsForce(0.0);
	Units::Angle gamma_com = Units::RadiansAngle(0.0);

	// LEVEL FLIGHT - manage speed with thrust and altitude with pitch
	if (levelFlight)
	{
		Units::Speed ev = tas_com - V;
		Units::Acceleration vDotCom = k_v * ev;

		// calculate the Thrust Command
		T_com = ac_mass * vDotCom
				+ D
				- ac_mass * Units::ONE_G_ACCELERATION*sin(gamma)
				- ac_mass * V * (weather->dVwx_dh*cos(psi) +
								 weather->dVwy_dh*sin(psi))
				  *sin(gamma)*cos(gamma);

		// command a level altitude
		gamma_com = Units::RadiansAngle(0.0);

		if( Units::MetersLength(h).value() - Fms->constraints[Fms->NextWp].constraint_altLow > 200.0 * FT_M && Units::MetersPerSecondSpeed(h_dot).value() != 0.0)
		{
			levelFlight = false;
			T_com = minThrust;
		}

	}
		// DESCENDING FLIGHT - manage altitude with thrust, speed with pitch
	else
	{
		double esf = AircraftCalculations::ESFconstantCAS(V, h);

		Units::Speed ev = tas_com - V;

		// adjust esf based on velocity error compared to the speed threshold
		if( ev <= -speed_thresh )
		{
			esf = 0.3;
		}
		else if( ev > -speed_thresh && ev < Units::ZERO_SPEED )
		{
			esf = (esf - 0.3)/speed_thresh*ev + esf;
		}
		else if( ev > Units::ZERO_SPEED && ev < speed_thresh)
		{
			esf = (1.7 - esf)/speed_thresh*ev + esf;
		}
		else if( ev >= speed_thresh )
		{
			esf = 1.7;
		}

		// descent rate
		Units::Speed dh_dt = ((T - D) * V) / (ac_mass * Units::ONE_G_ACCELERATION) *esf;

		gamma_com = Units::RadiansAngle(asin(-dh_dt/V));

		//if(gamma_com > 4.0*M_PI/180)
		//	gamma_com = 4.0*M_PI/180;

		if( e_alt < -alt_thresh )
		{
			T_com = 0.50 * maxThrust;
		}
		else if( e_alt > alt_thresh )
		{
			T_com = minThrust;
		}
		else
		{
			T_com = (minThrust - 0.50 * maxThrust)/(alt_thresh*2) * e_alt + (0.50 * maxThrust + minThrust)/2.0;
		}

		// Check if flight should level off
		if( Units::MetersLength(h).value() - Fms->constraints[Fms->NextWp].constraint_altLow < 100.0 * FT_M )
			levelFlight = true;

	}

	// limit thrust to max and min limits
	if( T_com > maxThrust )
	{
		T_com = maxThrust;
	}
	else if( T_com  < minThrust )
	{
		T_com = minThrust;
	}

	// Determine if speed brake is needed
	Units::Length xEnd = Units::FeetLength(Fms->xWp[Fms->number_of_waypoints-1]);
	Units::Length yEnd = Units::FeetLength(Fms->yWp[Fms->number_of_waypoints-1]);

	Units::Length distToEnd = sqrt(Units::sqr(x-xEnd) + Units::sqr(y-yEnd));

	//if(T_com == minThrust && distToEnd < 15.0 * NM_M && (h - Fms->constraints[Fms->NextWp].constraint_altLow) > 250.0 * FT_M)
	if(T_com == minThrust)
	{
		//T_com = minThrust;

		if(e_alt > alt_thresh)
		{

			int mode_new = 0;
			mBadaWithCalc.getConfigForDrag(v_cas, Units::MetersLength(h), flapConfig_new, mode_new);

			if(mode_new == flapConfig_new && mode_new <= 2)
			{
				speedBrakeCom = 0.5;
				speedBrakeOn = true;
				speedBrakeCounter = speedBrakeCounter + 1;
			}

			flapConfig_new = mode_new;
		}
		minThrustCounter = minThrustCounter + 1;
	}
	else
	{
		minThrustCounter = 0.0;
	}

	// If no longer commanding Min Thrust
	if( speedBrakeOn && speedBrakeCounter <= 30.0 )
	{
		speedBrakeCounter = speedBrakeCounter + 1;
		speedBrakeCom = 0.5;
	}
	else if ( speedBrakeOn && speedBrakeCounter > 30.0 )
	{
		if( minThrustCounter == 0 )
		{
			speedBrakeCounter = 0.0;
			speedBrakeCom = 0.0;
			speedBrakeOn = false;
		}
		else
		{
			speedBrakeCounter = speedBrakeCounter + 1;
			speedBrakeCom = 0.5;
		}
	}


	//% calculate the firsr-order derivative of the state vector:
	//Wind speed needs to be checked!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	dX.dx = Units::MetersPerSecondSpeed(V*cos(gamma)*cos(psi) + weather->Vwx);
	dX.dy = Units::MetersPerSecondSpeed(V*cos(gamma)*sin(psi) + weather->Vwy);
	dX.dh = Units::MetersPerSecondSpeed(-V*sin(gamma));
	dX.dV = (T-D)/ac_mass + Units::ONE_G_ACCELERATION*sin(gamma)
			+ V*(weather->dVwx_dh*cos(psi) + weather->dVwy_dh*sin(psi))
			  *sin(gamma)*cos(gamma);
	dX.dgamma = k_gamma * (gamma_com - gamma) -
				(weather->dVwx_dh*cos(psi) + weather->dVwy_dh * sin(psi)) * pow(sin(gamma), 2) * Units::ONE_RADIAN_ANGLE;
	dX.dpsi = (-L*sin(phi)/(ac_mass*V*cos(gamma))
			   - (weather->dVwx_dh*sin(psi) - weather->dVwy_dh*cos(psi))*tan(gamma)) * Units::ONE_RADIAN_ANGLE;
	dX.dT = k_t*(T_com - T);
	dX.dphi = k_phi*(phi_com - phi);
	dX.dspeedBrake = k_speedBrake*(speedBrakeCom - speedBrake);
	dX.flapConfig = flapConfig_new;

	return dX;
} // speed_on_pitch_control_dynamics

const Units::Speed &TestFrameworkDynamics::getVwx() const {
	return Vwx;
}

const Units::Speed &TestFrameworkDynamics::getVwy() const {
	return Vwy;
}
