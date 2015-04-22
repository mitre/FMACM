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
// Copyright 2015 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once
#include <vector>
#include <string>

#ifdef _LINUX_
#include <algorithm>
#include <stdlib.h>
#endif

#include "stdio.h"
#include "AircraftState.h"
#include "micros.h"
#include "Guidance.h"

#include "DMatrix.h"

class InternalObserver
{
public:
	static InternalObserver *getInstance();
	InternalObserver(void);
	~InternalObserver(void);
	static void clearInstance();
	void reset(void);
	void process(void);

	// stores and output the state-model report.

	void storeStateModel(AircraftState asv, int flapsConfig, float speed_brake);
	void outputStateModel();

	//gwang 2014-05-12: write ADS-B reports into  files
	//void collect_ads_b_ether(ADSBEther in, int id_in); // collect ADSBEther into this class
	//void collect_ptis_b_report(ADSBSVReport adsb_sv_report);
	void process_ads_b_reports(); 
	//end gwang 2014-05-12
	//void collect_ptis_b_ether(ADSBEther ether_in, int id_in) ;
	void process_ptis_b_reports_old(); 
	void process_ptis_b_reports(); 

	// output the IM commands (UNITS ARE IN METERS)
	void IM_command_output(int id_in, double time_in, double state_alt,
		double state_TAS, double state_groundspeed_in, double ias_command_in,
		double unmod_ias, double tas_command_in, double ref_vel_in,
		double ref_dist_in, double curr_dist_in, double true_dist_in); // add an entry to the command list
	void process_IM_command(); // process the IM report

	// output the Dynamics results and given guidance
	void dynamics_output(AircraftState dynamics_in, Guidance guidance_in); // add an entry to the dynamics list
	void process_dynamics(); // process the dynamics report

	// output the MSI spacing algorithm values
	void msi_output(int ac_id, double ac_time, double ac_dist, double target_time, double guidance_ias_in, double edot_ref, double error_ref, double curr_spacing, double vel_gain, double mod_speed_command, double actual_tas, double commanded_tas, double altitude);
	void process_msi();
	void process_NM_aircraft();
	void process_NM_stats();

	// output number of speed commands per aircraft
	void speed_command_count_output(std::vector<int> speed_change_list);

	void process_speed_command_count();

	// output final groundspeed
	void updateFinalGS(int id, double gs);
	void outputFinalGS();
	void processFinalGS();

	// predicted wind matrix metric
	void addPredictedWind(int id,DMatrix &predWindX,DMatrix &predWindY);
	void dumpPredictedWind();
	std::string predWindsHeading(int lastIx);
	std::string predWindsData(int id,int row,std::string field,DMatrix &mat);

	// time to go metric
	void addTTG(int aircraftId,double tm,double target_ttg_to_ach,double own_ttg_to_ach,
				double curr_distance,double reference_distance);
	void dumpTTG();

	// true wind debug data.
	bool debugTrueWind();
	void writeTrueWind(std::string str);
	void setTrueWindHdrVals(double time, int id);

	// TEST OUTPUT for cross-track output per second
	void cross_output(double x_in, double y_in, double dynamic_cross, double commanded_cross, double psi_command, double phi, double limited_phi);

	void process_cross();

	void report();

	static void FatalError(char *str)
	{
		printf("%s", str);
		getchar();
		exit(1);
	} // FatalError

	// getter/setters for the scenario name
	std::string get_scenario_name();
	void set_scenario_name(std::string in);

	// method to set the current radar data iterator position based on given radar name
	void set_radar_data_it(std::string radar_name);

	// Initializes metrics where necessary.
	void initializeIteration(int number_of_aircraft);

	// Sets NM file output flag.
	void setNMOutput(bool NMflag);

	// Determines whether to output NM data or not.
	bool outputNM(void);

private:

	// Formats state model report data.
	std::string stateModelString(AircraftState asv, int flapsConfig, float speed_brake);

	// Returns header for state model report.
	std::string stateModelHdr();

	static InternalObserver* mInstance;

};
