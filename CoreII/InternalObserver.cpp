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

#include "InternalObserver.h"
#include "constants.h"
#include "CustomMath.h"
#include "micros.h"
#include "AircraftCalculations.h"
#include <sstream>
#include <fstream>
#include <algorithm>
#include <Length.h>

using namespace std;

InternalObserver *InternalObserver::mInstance = NULL;

InternalObserver *InternalObserver::getInstance()
{
   if(mInstance==NULL)
      mInstance = new InternalObserver();
   return mInstance;
}

void InternalObserver::clearInstance()
{
	mInstance = NULL; // blow away the instance
}

InternalObserver::InternalObserver(void) 
{

}

InternalObserver::~InternalObserver(void)
{

}

void InternalObserver::reset(void) 
{
	//reset for each iteration:

}

void InternalObserver::process(void) 
{

}

// getter/setters for the scenario name
string InternalObserver::get_scenario_name()
{
	return NULL;
}

void InternalObserver::set_scenario_name(string in)
{
}

void InternalObserver::storeStateModel(AircraftState asv, int flapsConfig, float speed_brake) {

}


string InternalObserver::stateModelString(AircraftState asv, int flapsConfig, float speed_brake) {
  return NULL;
}


string InternalObserver::stateModelHdr() {
  return NULL;
}


void InternalObserver::outputStateModel() {
}


// output the IM commands
void InternalObserver::IM_command_output(int id_in, double time_in, double state_alt,
	double state_TAS, double state_groundspeed_in, double ias_command_in,
	double unmod_ias, double tas_command_in, double ref_vel_in, double ref_dist_in,
	double predDistIn, double trueDistIn)
{

}

void InternalObserver::process_IM_command()
{

}

// output the Dynamics results and given guidance
void InternalObserver::dynamics_output(AircraftState dynamics_in, Guidance guidance_in) // add an entry to the dynamics list
{

}

void InternalObserver::process_dynamics() // process the dynamics report
{

}


// output the MSI spacing algorithm values
void InternalObserver::msi_output(int ac_id, double ac_time, double ac_dist, double target_time, double guidance_ias_in, double edot_ref, double error_ref, double curr_spacing, double vel_gain, double mod_speed_command, double actual_tas, double commanded_tas, double altitude) 
{

}

void InternalObserver::process_msi()
{

}

// outputs the Nautical Mile report for all aircraft
void InternalObserver::process_NM_aircraft()
{

}

void InternalObserver::process_NM_stats() {

}


// TEST OUTPUT for cross-track output per second
void InternalObserver::cross_output(double x_in, double y_in, double dynamic_cross, double commanded_cross, double psi_command, double phi, double limited_phi)
{

}

void InternalObserver::process_cross()
{

}

void InternalObserver::speed_command_count_output(std::vector<int> speed_change_list)
{
}

void InternalObserver::process_speed_command_count()
{

}

void InternalObserver::initializeIteration(int number_of_aircraft) {

}

void InternalObserver::updateFinalGS(int id, double gs) {

}

void InternalObserver::outputFinalGS() {

}

void InternalObserver::processFinalGS() {

}

void InternalObserver::process_ads_b_reports() // process the ADS-B reports 
{

}
//end gwang 2014-05-12

void InternalObserver::process_ptis_b_reports_old() // process the ADS-B reports 
{

}

void InternalObserver::process_ptis_b_reports() // process the ADS-B reports 
{

}
void InternalObserver::addPredictedWind(int id,DMatrix &predWindX, DMatrix &predWindY) {

}

string InternalObserver::predWindsHeading(int numVals) {

	return NULL;
}

string InternalObserver::predWindsData(int id,int col,string field,DMatrix &mat) {

	return NULL;
}

void InternalObserver::dumpPredictedWind() {

}

void InternalObserver::addTTG(
		int aircraftId,double tm,double target_ttg_to_ach,double own_ttg_to_ach,
						double curr_distance,double reference_distance)
{
}


void InternalObserver::dumpTTG() {

}

void InternalObserver::writeTrueWind(string str) {

}

void InternalObserver::setTrueWindHdrVals(double time, int id) {

}

bool InternalObserver::debugTrueWind() {
	return false;
}


void InternalObserver::setNMOutput(bool NMflag) {

}


bool InternalObserver::outputNM(void) {
  return false;
}

