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
#include <vector>
#include <string>
#include <stdexcept>
#include "public/AircraftState.h"
#include "public/Guidance.h"
#include "public/IMCommandObserver.h"
#include "public/DynamicsObserver.h"
#include "public/NMObserver.h"
#include "public/CrossTrackObserver.h"
#include "public/AchieveObserver.h"
#include "public/VerticalPathObserver.h"
#include "public/MaintainMetric.h"
#include "public/MergePointMetric.h"
#include "public/ClosestPointMetric.h"
#include "public/WindStack.h"

class InternalObserver
{
public:
	static InternalObserver *getInstance();
	static void clearInstance();
	void reset(void);
	void process(void);

	// stores and output the state-model report.

	void storeStateModel(AircraftState asv, int flapsConfig, float speed_brake, double ias);
	void outputStateModel();

	//gwang 2014-05-12: write ADS-B reports into  files
	void collect_ptis_b_report(Sensor::ADSB::ADSBSVReport adsb_sv_report);
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

	// TODO: is this part of 'output the PSEB spacing algorithm values'?
	void process_NM_aircraft();
	void process_NM_stats();

	// output number of speed commands per aircraft
	void speed_command_count_output(vector<int> speed_change_list);

	void process_speed_command_count();

	// output maintain metrics 
	void outputMaintainMetrics();
	void processMaintainMetrics();

	// output final groundspeed
	void updateFinalGS(int id, double gs);
	void outputFinalGS();
	void processFinalGS();

	// output merge point metric
	void outputMergePointMetric();
	void processMergePointMetric();

	// output closest point metric
	void outputClosestPointMetric();
	void processClosestPointMetric();

	// predicted wind matrix metric
	void addPredictedWind(int id, const WindStack &predWindX, const WindStack &predWindY);
	void dumpPredictedWind();
	std::string predWindsHeading(int lastIx);
	std::string predWindsData(int id,int row,std::string field, const WindStack &mat);

	// time to go metric
	void addAchieveRcd(size_t aircraftId,double tm,double target_ttg_to_ach,double own_ttg_to_ach,
				double curr_distance,double reference_distance);
	void dumpAchieveList();

	// Kinematic trajectory output.
	void dumpOwnKinTraj(int id,VerticalPath &fullTraj);
	void dumpTargetKinTraj(int id,VerticalPath &fullTraj);

	// true wind debug data.
	bool debugTrueWind();
	void writeTrueWind(std::string str);
	void setTrueWindHdrVals(double time, int id);

	// TEST OUTPUT for cross-track output per second
	void cross_output(double x_in, double y_in, double dynamic_cross, double commanded_cross, double psi_command, double phi, double limited_phi);

	void process_cross();

	void report();

	static void FatalError(const char *str)
	{
		LOG4CPLUS_FATAL(logger, str);
		// Can this getchar() be removed?  KAL, 9/22/2016
		// getchar();
		throw std::logic_error(str);
	} // FatalError

	//reporters:

	//Data:
	double duration_adsb_transmit, duration_nav_sensor, duration_dynamics, duration_application, duration_adsb_receive;
	double adsb_receiver_1st, adsb_receiver_2nd, adsb_receiver_3rd, adsb_receiver_total;

	typedef std::vector<AircraftState> listlist;
	listlist  isa;
	std::vector<AircraftState> truth_state_vector_list;
	std::vector<AircraftState> navigation_measurement_state_vector_list;
	std::vector<double> range_to_next_waypoint;

	// output data vectors

	std::vector<std::vector <std::vector <std::string> > > stateModelOutput;
	std::vector<IMCommandObserver> im_commands;
	std::vector<DynamicsObserver> achieved_dynamics_list;
	std::vector<NMObserver> aircraft_NM_list;
	std::vector<std::vector <int> > aircraft_speed_count_list;
	CrossTrackObserver cross_entry;

	std::vector<MaintainMetric> maintainStats;
	std::vector<std::string> maintainOutput;

	std::vector<double> finalGS;
	std::vector<std::string> finalGSOutput;

	std::vector<MergePointMetric> mergePointStats;
	std::vector<std::string> mergePointOutput;

	std::vector<ClosestPointMetric> closestPointStats;
	std::vector<std::string> closestPointOutput;
	std::vector<Sensor::ADSB::ADSBSVReport> ptis_b_report_list;


	int	mode_merge_spacing[6000];
	int cycle[6000];
	double IAS[6000];
	double time_spacing_error[6000];
	double correction_term[6000];
	double speed_command_kt[6000];
	double speed_command_KIAS[6000];
	double total_duration_update_aircraft;

	std::vector<std::vector <AchieveObserver> > mAchieveList; // Organized by aircraft id.

	// Kinematic trajectory output objects.  Each dumps kinematic trajectories
	// over a whole scenario, for all iterations for all aircraft into a single
	// file.

	VerticalPathObserver *mOwnKinVertPathObs;   // Outputs own kinematic predicted vertical paths.
	VerticalPathObserver *mTargKinVertPathObs;  // Outputs target kinematic predicted vertical paths.

	// Some debug for winds from dynamics and speed_on_pitch_control_dynamics.
	double trueTime;
	int trueId;
	bool debuggingTrueWind;

	int scenario_iter; // variable to store the current scenario iteration

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

	InternalObserver(void);
	~InternalObserver(void);

	// Formats state model report data.
	std::string stateModelString(AircraftState asv, int flapsConfig, float speed_brake, double ias);

	// Returns header for state model report.
	std::string stateModelHdr();

	//Data for individual aircraft

	//Data for aggregate
	std::string scenario_name;

	//NM output flag
	bool outputNMFiles;

	std::vector<std::string> predWinds;

	static InternalObserver* mInstance;
    static log4cplus::Logger logger;

};
