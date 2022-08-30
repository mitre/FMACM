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
#include "public/WeatherPrediction.h"

class InternalObserver
{
public:
   static InternalObserver *getInstance();

   static void clearInstance();

   void reset(void);

   void process(void);

   // stores and output the state-model report.

   void storeStateModel(aaesim::open_source::AircraftState asv,
                        int flapsConfig,
                        float speed_brake,
                        double ias);

   void outputStateModel();

   //gwang 2014-05-12: write ADS-B reports into  files
   void collect_ptis_b_report(Sensor::ADSB::ADSBSVReport adsb_sv_report);

   void process_ptis_b_reports();

   // output the IM commands (UNITS ARE IN METERS)
   void IM_command_output(int id_in,
                          double time_in,
                          double state_alt,
                          double state_TAS,
                          double state_groundspeed_in,
                          double ias_command_in,
                          double unmod_ias,
                          double tas_command_in,
                          double ref_vel_in,
                          double ref_dist_in,
                          double curr_dist_in,
                          double true_dist_in); // add an entry to the command list
   void process_IM_command(); // process the IM report

   void process_NM_aircraft();

   void process_NM_stats();

   // output number of speed commands per aircraft
   void speed_command_count_output(vector<int> speed_change_list);

   void process_speed_command_count();

   // output maintain metrics
   void outputMaintainMetrics();

   void processMaintainMetrics();

   // output final groundspeed
   void updateFinalGS(int id,
                      double gs);

   void outputFinalGS();

   void processFinalGS();

   // output merge point metric
   void outputMergePointMetric();

   void processMergePointMetric();

   // output closest point metric
   void outputClosestPointMetric();

   void processClosestPointMetric();

   // predicted wind matrix metric
   void addPredictedWind(int id, const WeatherPrediction &weatherPrediction);

   void dumpPredictedWind();

   std::string predWindsHeading(int lastIx);

   std::string predWindsData(int id,
                             int row,
                             std::string field,
                             const WindStack &mat);

   std::string predTempData(int id,
                            std::string field,
                            const WeatherPrediction &weatherPrediction);


   // time to go metric
   void addAchieveRcd(size_t aircraftId,
                      double tm,
                      double target_ttg_to_ach,
                      double own_ttg_to_ach,
                      double curr_distance,
                      double reference_distance);

   void dumpAchieveList();

	static void FatalError(const char *str)
	{
		LOG4CPLUS_FATAL(logger, str);
		throw std::logic_error(str);
	} // FatalError

   NMObserver &GetNMObserver(int id);

   MaintainMetric &GetMaintainMetric(int id);

   MergePointMetric &GetMergePointMetric(int id);

   ClosestPointMetric &GetClosestPointMetric(int id);

   void set_scenario_name(std::string in);

   // Initializes metrics where necessary.
   void initializeIteration(int number_of_aircraft_in_scenario);

   // Sets NM file output flag.
   void setNMOutput(bool NMflag);

   // Determines whether to output NM data or not.
   bool outputNM(void);

   void SetRecordMaintainMetrics(bool new_value);

   const bool GetRecordMaintainMetrics() const;
   int GetScenarioIter() const;
   void SetScenarioIter(int scenario_iter);
   CrossTrackObserver& GetCrossEntry();

private:
   class AircraftIterationStats {
   public:
      MergePointMetric m_merge_point_metric;
      MaintainMetric m_maintain_metric;
      ClosestPointMetric m_closest_point_metric;
      double finalGS;
      AircraftIterationStats();
   };

   class AircraftScenarioStats {
   public:
      NMObserver m_nm_observer;
      std::vector<AchieveObserver> m_achieve_list;
   };


   static InternalObserver *mInstance;
   static log4cplus::Logger logger;

   InternalObserver(void);

   ~InternalObserver(void);

   // Formats state model report data.
   std::string stateModelString(aaesim::open_source::AircraftState asv,
                                int flapsConfig,
                                float speed_brake,
                                double ias);

   // Returns header for state model report.
   std::string stateModelHdr();

   // output flags
   bool outputNMFiles;
   bool m_save_maintain_metrics;

   //Data for aggregate
   std::string scenario_name;

   int m_scenario_iter; // variable to store the current scenario iteration

   CrossTrackObserver m_cross_entry;

   std::vector<std::string> predWinds;

   //Data for individual aircraft
   std::map<int,AircraftIterationStats> m_aircraft_iteration_stats;  // cleared between iterations
   std::map<int,AircraftScenarioStats> m_aircraft_scenario_stats; // never cleared

   // output data vectors
   std::vector<std::vector<std::vector<std::string> > > stateModelOutput;
   std::vector<IMCommandObserver> im_commands;
   std::vector<std::vector<int> > aircraft_speed_count_list;
   std::vector<Sensor::ADSB::ADSBSVReport> ptis_b_report_list;

   // string vectors for file output
   std::vector<std::string> maintainOutput;
   std::vector<std::string> finalGSOutput;
   std::vector<std::string> mergePointOutput;
   std::vector<std::string> closestPointOutput;

   // Kinematic trajectory output objects.  Each dumps kinematic trajectories
   // over a whole scenario, for all iterations for all aircraft into a single
   // file.
   VerticalPathObserver *mOwnKinVertPathObs;   // Outputs own kinematic predicted vertical paths.
   VerticalPathObserver *mTargKinVertPathObs;  // Outputs target kinematic predicted vertical paths.

};
