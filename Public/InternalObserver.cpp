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

#include "public/InternalObserver.h"
#include "public/Environment.h"
#include "public/StereographicProjection.h"

using namespace std;

InternalObserver *InternalObserver::mInstance = NULL;
log4cplus::Logger InternalObserver::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("InternalObserver"));

InternalObserver *InternalObserver::getInstance() {
   if (mInstance == NULL) {
      mInstance = new InternalObserver();
   }
   return mInstance;
}

void InternalObserver::clearInstance() {
   if (mInstance != NULL) {
      delete mInstance;
      mInstance = NULL; // blow away the instance
   }
}

InternalObserver::InternalObserver(void) {
   m_save_maintain_metrics = true;
   scenario_iter = 0;
   debuggingTrueWind = false;

   trueTime = -99999.0;
   trueId = -1;

   mOwnKinVertPathObs = NULL;
   mTargKinVertPathObs = NULL;

   outputNMFiles = true;
}

InternalObserver::~InternalObserver(void) {

   if (mOwnKinVertPathObs != NULL) {
      delete mOwnKinVertPathObs;
   }

   if (mTargKinVertPathObs != NULL) {
      delete mTargKinVertPathObs;
   }

}

void InternalObserver::reset(void) {
   //reset for each iteration:

}

void InternalObserver::process(void) {
   //ADSBEther::iterator sv_report;

   //TODO: solve the problem caused by dynamic memory allocation in track class, specifically, in cov.m
   //The memmory for cov.m (dynamic) is lost at this point. !!!!!!!!!!!!!!!!!!!!!!!!!!!!

   FILE *fp = fopen((scenario_name + "-output-WinSS.csv").c_str(), "w");

   vector<AircraftState>::iterator sv_report;
   for (sv_report = truth_state_vector_list.begin(); sv_report != truth_state_vector_list.end(); ++sv_report) {
      fprintf(fp, "%d,%f,%f,%f,%f,%f,%f,%f\n",
              (*sv_report).id,
              (*sv_report).time,
              (*sv_report).x,
              (*sv_report).y,
              (*sv_report).z,
              (*sv_report).xd,
              (*sv_report).yd,
              (*sv_report).zd
      );
   }

   fprintf(fp, "\n");

   fclose(fp);

   outputStateModel();
   dumpPredictedWind();
   process_IM_command();
   process_NM_stats();
   process_speed_command_count();
   processMaintainMetrics();
   processFinalGS();
   processMergePointMetric();
   processClosestPointMetric();
   process_ptis_b_reports();

   if (mAchieveList.size() > 0) {
      dumpAchieveList();
   }

}

void InternalObserver::set_scenario_name(string in) {
   scenario_name = in;
}

void InternalObserver::storeStateModel(AircraftState asv,
                                       int flapsConfig,
                                       float speed_brake,
                                       double ias) {
   // Stores data for state model output as string.
   //
   // asv:Aircraft state vector data.
   // flapsConfig:Configuration of flaps.

   while (scenario_iter >= (int) stateModelOutput.size()) {
      vector<vector<string> > strings;
      stateModelOutput.push_back(strings);
   }

   // Add vector for current aircraft if needed.

   while (asv.id >= (int) stateModelOutput[scenario_iter].size()) {
      vector<string> strings;
      stateModelOutput[scenario_iter].push_back(strings);
   }


   // Store state model string.

   stateModelOutput[scenario_iter][asv.id].push_back(stateModelString(asv, flapsConfig, speed_brake, ias));
}


string InternalObserver::stateModelString(AircraftState asv,
                                          int flapsConfig,
                                          float speed_brake,
                                          double ias) {
   // Formats state model data string.
   //
   // asv:Aircraft state vector object.
   // flapsConfig:flaps configuration
   //
   // returns state model report data string.

   stringstream strm;

   // get current speed in FPS

   double currSpeed = sqrt(pow(asv.xd, 2) + pow(asv.yd, 2));

   strm << scenario_iter << ","; // iteration number
   strm << asv.id << ","; // id
   strm << asv.time << ","; // time
   strm << asv.x << ","; // x value in feet
   strm << asv.y << ","; // y value in feet
   strm << asv.z << ","; // altitude value in feet
   strm << asv.xd << ","; // x velocity in FPS
   strm << asv.yd << ","; // y velocity in FPS
   strm << asv.zd * 60 << ","; // altitude velocity int Feet per Minute

   // Note:These two are the exactly the same calculation except for a units conversion.

   strm << currSpeed << ","; // ground speed in FPS
   strm << Units::KnotsSpeed(asv.getGroundSpeed()).value() << ",";

   strm << asv.xdd << ","; // x acceleration FPS^2
   strm << asv.ydd << ","; // y acceleration FPS^2
   strm << asv.zdd * 60 << ","; // z acceleration in Feet per Minute per Second
   strm << asv.Vwx << ","; // true wind Vwx in MPS
   strm << asv.Vwy << ","; // true wind Vwy in MPS
   strm << asv.Vw_para << ","; // true wind Vw_para in MPS
   strm << asv.Vw_perp << ","; // true wind Vw_perp in MPS
   strm << asv.m_distance_to_go << ","; // output distance to go in meters
   strm << flapsConfig << ","; // configuration for flaps (mainly debug)
   strm << speed_brake << ",";
   strm << ias;

   string str;

   strm >> str;

   return str;
}


string InternalObserver::stateModelHdr() {
   // Returns state model string.

   string str = "Iteration,AC_ID,Time,X(feet),Y(feet),Alt(feet),X_Vel(FPS),Y_Vel(FPS),Alt_Vel(FPM),GroundSpeed(FPS),GroundSpeed(Knots),X_Accel(FPSS),Y_Accel(FPSS),Alt_Accel(FPMPS),True_Wind_Vwx(MPS),True_Wind_Vwy(MPS),Vw_para(MPS),Vw_perp(MPS),distToGo(M),FlapsConfig,SpeedBrake,IAS(Knots)";

   return str;
}


void InternalObserver::outputStateModel() {
   // Writes state model report.

   string fname = scenario_name + "-state-model-output.csv";
   ofstream out;
   out.open(fname.c_str());

   if (out.is_open()) {
      // Header

      out << stateModelHdr().c_str() << endl;

      for (unsigned int i = 0; i < stateModelOutput.size(); i++) {
         // Loop through aircraft and output all records.

         for (unsigned int j = 0; j < stateModelOutput[i].size(); j++) {
            for (unsigned int k = 0; k < stateModelOutput[i][j].size(); k++) {

               out << stateModelOutput[i][j][k].c_str() << endl;
            }
         }
      }
   }

   out.close();


   // Clear everything
   stateModelOutput.clear();

}


// output the IM commands
void InternalObserver::IM_command_output(int id_in,
                                         double time_in,
                                         double state_alt,
                                         double state_TAS,
                                         double state_groundspeed_in,
                                         double ias_command_in,
                                         double unmod_ias,
                                         double tas_command_in,
                                         double ref_vel_in,
                                         double ref_dist_in,
                                         double predDistIn,
                                         double trueDistIn) {
   IMCommandObserver new_command;
   new_command.iteration = this->scenario_iter;
   new_command.id = id_in;
   new_command.time = time_in;
   new_command.state_altitude = state_alt * FT_M;
   new_command.distance_to_go = fabs(predDistIn); // Distance is in meters
   new_command.state_TAS = state_TAS; // TAS is in MPS
   new_command.state_groundspeed = state_groundspeed_in; // Ground Speed is in meters
   new_command.IAS_command = ias_command_in; // IAS command in meters
   new_command.unmodified_IAS = unmod_ias; // unmodified IAS command in meters
   new_command.TAS_command = tas_command_in; // TAS command in meters
   new_command.reference_velocity = ref_vel_in; // reference velocity in meters
   new_command.reference_distance = ref_dist_in;
   new_command.predictedDistance = predDistIn;
   new_command.distance_difference = ref_dist_in - predDistIn;
   new_command.trueDistance = trueDistIn;

   im_commands.push_back(new_command);
}

void InternalObserver::process_IM_command() {
   // open report data file
   string output_file_name = scenario_name + "-IM-command-output.csv";
   ofstream out;
   out.open(output_file_name.c_str());

//    sort(im_commands.begin(), im_commands.end()); // sort the list by aircraft and time

   //if file opens successfully, process the output
   if (out.is_open() == true && im_commands.empty() == false) {
      out <<
          "Iteration,Aircraft_ID,Time,Aircraft_True_Distance(Meters),Aircraft_Predicted_Distance(Meters),State_Altitude(Meters),State_TAS(MPS),State_GroundSpeed(MPS),IAS_Speed_Command(MPS),Unmodified_IAS_Command(MPS),TAS_Speed_Command(MPS)"
          <<
          endl;

      for (unsigned int loop = 0; loop < im_commands.size(); loop++) {
         out << im_commands[loop].iteration << ",";
         out << im_commands[loop].id << ",";
         out << im_commands[loop].time << ",";
         out << im_commands[loop].trueDistance << ",";
         out << im_commands[loop].predictedDistance << ",";
         out << im_commands[loop].state_altitude << ",";
         out << im_commands[loop].state_TAS << ",";
         out << im_commands[loop].state_groundspeed << ",";
         out << im_commands[loop].IAS_command << ",";
         out << im_commands[loop].unmodified_IAS << ",";
         out << im_commands[loop].TAS_command << endl;
      }

      out.close();
   }
}

// outputs the Nautical Mile report for all aircraft
void InternalObserver::process_NM_aircraft() {
   if (outputNM()) {
      // Write NM report if we are outputting NM files.

      // loop to process all of the aircraft NM reports
      for (unsigned int loop = 0; loop < aircraft_NM_list.size(); loop++) {
         // if the current aircraft has Nautical Mile output entries output them
         if (!aircraft_NM_list[loop].entry_list.empty()) {
            char *temp = new char[10];

            sprintf(temp, "%u", loop);

            string output_file_name = scenario_name + "_AC" + temp + "-NM-output.csv";
            delete[] temp;

            ofstream out;

            // if first iteration create file, otherwise append file
            if (scenario_iter == 0) {
               out.open(output_file_name.c_str());
            } else {
               out.open(output_file_name.c_str(), ios::out | ios::app);
            }

            // if file opens properly and entry list isn't empty output the Nautical Mile results
            if (out.is_open()) {
               // if first iteration create header

               if (scenario_iter == 0) {
                  out <<
                      "AC_ID,Iteration,Predicted_Distance(NM),True_Distance(NM),Time,Own_Command_IAS(Knots),Own_Current_GroundSpeed(Knots),Target_GroundSpeed(Knots),Min_IAS_Command(Knots),Max_IAS_Command(Knots),Min_GS_Command(Knots),Max_GS_Command(Knots)"
                      <<
                      endl;
               }

               aircraft_NM_list[loop].initialize_stats(); // initialize the statistics to the size of the entry list

               // loop to process all aircraft entries
               for (unsigned int index = 0; index < aircraft_NM_list[loop].entry_list.size(); index++) {
                  // output the report
                  out << loop << ",";
                  out << scenario_iter << ",";
                  out << aircraft_NM_list[loop].entry_list[index].predictedDistance / NM_M << ",";
                  out << aircraft_NM_list[loop].entry_list[index].trueDistance / NM_M << ",";
                  out << aircraft_NM_list[loop].entry_list[index].time << ",";
                  out << aircraft_NM_list[loop].entry_list[index].acIAS / KTS_MPS << ",";
                  out << aircraft_NM_list[loop].entry_list[index].acGS / KTS_MPS << ",";
                  out << aircraft_NM_list[loop].entry_list[index].targetGS / KTS_MPS << ",";
                  out << aircraft_NM_list[loop].entry_list[index].minIAS / KTS_MPS << ",";
                  out << aircraft_NM_list[loop].entry_list[index].maxIAS / KTS_MPS << ",";
                  out << aircraft_NM_list[loop].entry_list[index].minTAS / KTS_MPS << ",";
                  out << aircraft_NM_list[loop].entry_list[index].maxTAS / KTS_MPS << endl;

                  // add entries to Statistics class
                  aircraft_NM_list[loop].predictedDistance[index] =
                        aircraft_NM_list[loop].entry_list[index].predictedDistance / NM_M;
                  aircraft_NM_list[loop].trueDistance[index] =
                        aircraft_NM_list[loop].entry_list[index].trueDistance / NM_M;
                  aircraft_NM_list[loop].ac_IAS_stats[index].insert(
                        aircraft_NM_list[loop].entry_list[index].acIAS / KTS_MPS);
                  aircraft_NM_list[loop].ac_GS_stats[index].insert(
                        aircraft_NM_list[loop].entry_list[index].acGS / KTS_MPS);
                  aircraft_NM_list[loop].target_GS_stats[index].insert(
                        aircraft_NM_list[loop].entry_list[index].targetGS / KTS_MPS);
                  aircraft_NM_list[loop].min_IAS_stats[index].insert(
                        aircraft_NM_list[loop].entry_list[index].minIAS / KTS_MPS);
                  aircraft_NM_list[loop].max_IAS_stats[index].insert(
                        aircraft_NM_list[loop].entry_list[index].maxIAS / KTS_MPS);
               }

               aircraft_NM_list[loop].entry_list.clear();
               aircraft_NM_list[loop].curr_NM = -2; // resets the current NM value
               out.close();
            }
         }
      }
   }
}

void InternalObserver::process_NM_stats() {

   // Write NM stats output NM files being processed.
   if (outputNM()) {

      // loop to process all of the aircraft NM reports

      for (unsigned int loop = 0; loop < aircraft_NM_list.size(); loop++) {

         if (aircraft_NM_list[loop].predictedDistance.size() > 0) {
            char *temp = new char[10];

            sprintf(temp, "%d", loop);

            string output_file_name = scenario_name + "_AC" + temp + "-stats-NM-output.csv";

            delete[] temp;

            ofstream out;

            out.open(output_file_name.c_str(), ios::out);

            // if file opens properly and entry list isn't empty output the Nautical Mile statistics
            if (out.is_open()) {
               out <<
                   "Predicted_Distance,True_Distance,AC_IAS_Mean,AC_IAS_Dev,AC_GS_Mean,AC_GS_Dev,Target_GS_Mean,Target_GS_Dev,Min_Mean,Min_Dev,Max_Mean,Max_Dev"
                   <<
                   endl;

               // loop to process all distance entry statistics
               for (unsigned int index = 0; index < aircraft_NM_list[loop].predictedDistance.size(); index++) {
                  out << aircraft_NM_list[loop].predictedDistance[index] << ",";
                  out << aircraft_NM_list[loop].trueDistance[index] << ",";
                  out << aircraft_NM_list[loop].ac_IAS_stats[index].get_mean() << ",";
                  out << aircraft_NM_list[loop].ac_IAS_stats[index].get_std() << ",";
                  out << aircraft_NM_list[loop].ac_GS_stats[index].get_mean() << ",";
                  out << aircraft_NM_list[loop].ac_GS_stats[index].get_std() << ",";
                  out << aircraft_NM_list[loop].target_GS_stats[index].get_mean() << ",";
                  out << aircraft_NM_list[loop].target_GS_stats[index].get_std() << ",";
                  out << aircraft_NM_list[loop].min_IAS_stats[index].get_mean() << ",";
                  out << aircraft_NM_list[loop].min_IAS_stats[index].get_std() << ",";
                  out << aircraft_NM_list[loop].max_IAS_stats[index].get_mean() << ",";
                  out << aircraft_NM_list[loop].max_IAS_stats[index].get_std() << endl;
               }

               out.close();
            }

         }
      }
   }
}


// TEST OUTPUT for cross-track output per second
void InternalObserver::cross_output(double x_in,
                                    double y_in,
                                    double dynamic_cross,
                                    double commanded_cross,
                                    double psi_command,
                                    double phi,
                                    double limited_phi) {
   cross_entry.x = x_in;
   cross_entry.y = y_in;
   cross_entry.dynamic_cross = dynamic_cross;
   cross_entry.commanded_cross = commanded_cross;
   cross_entry.psi_command = psi_command;
   cross_entry.phi = phi;
   cross_entry.limited_phi = limited_phi;

}

void InternalObserver::process_cross() {
   string output_file_name = scenario_name + "-crosstrack-report.csv";
   ofstream out;
   if (cross_entry.time < 2) {
      out.open(output_file_name.c_str());
   } else {
      out.open(output_file_name.c_str(), ios::out | ios::app);
   }

   if (out.is_open() == true) {
      // if at the first time stamp create the header
      if (cross_entry.time < 2) {
         out <<
             "Time,X(Meters),Y(Meters),Dynamics_Cross(Meters),Commanded_Cross(Meters),Unmodified_Cross,Psi_Command,Phi,Limited_Phi,Reported_Distance(Meters)"
             <<
             endl;
      }

      if (cross_entry.time >= 0.0) {
         // add entry to the output file

         out << cross_entry.time << ",";
         out << cross_entry.x << ",";
         out << cross_entry.y << ",";
         out << cross_entry.dynamic_cross << ",";
         out << cross_entry.commanded_cross << ",";
         out << cross_entry.unmodified_cross << ",";
         out << cross_entry.psi_command << ",";
         out << cross_entry.phi << ",";
         out << cross_entry.limited_phi << ",";
         out << cross_entry.reported_distance << endl;
      }

      out.close(); // close output file
   }
}

void InternalObserver::speed_command_count_output(vector<int> speed_change_list) {
   aircraft_speed_count_list.push_back(speed_change_list);
}

void InternalObserver::process_speed_command_count() {
   // open report data file
   string output_file_name = scenario_name + "-Speed_Command_Count-output.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open() && aircraft_speed_count_list.size() > 0) {
      // generate header information
      out << "Iteration";
      for (unsigned int loop = 0; loop < aircraft_speed_count_list[0].size(); loop++) {
         out << ",AC" << loop;
      }
      out << endl;

      // output the data for each iteration
      for (unsigned int iter_loop = 0; iter_loop < aircraft_speed_count_list.size(); iter_loop++) {
         out << iter_loop;
         for (unsigned int ac_loop = 0; ac_loop < aircraft_speed_count_list[iter_loop].size(); ac_loop++) {
            out << "," << aircraft_speed_count_list[iter_loop][ac_loop];
         }
         out << endl;
      }

      out.close();
   }
}

void InternalObserver::initializeIteration(int number_of_aircraft_in_scenario) {
   maintainStats.clear();
   finalGS.clear();
   mergePointStats.clear();
   closestPointStats.clear();
   predWinds.clear();

   for (int ix = 0; ix < number_of_aircraft_in_scenario; ix++) {
      MaintainMetric emptyMaintain;
      maintainStats.push_back(emptyMaintain);

      finalGS.push_back(-1.0);

      MergePointMetric emptyMergePoint;
      mergePointStats.push_back(emptyMergePoint);

      ClosestPointMetric emptyClosestPoint;
      closestPointStats.push_back(emptyClosestPoint);
   }
}

void InternalObserver::outputMaintainMetrics() {
   // Post processes the maintain metric data after each iteration,
   // forming a string for each iteration and placing it in a local
   // string vector.

   string body;
   char bfr[121];


   // Add header.

   if (maintainOutput.size() == 0) {
      body = "Iteration";

      for (unsigned int ix = 1; ix < maintainStats.size(); ix++) {
         sprintf(bfr, ",ac %d-mean,ac %d-stdev,ac %d-95bound,ac %d-maintainTime,ac %d-timeGreaterThan10",
                 ix, ix, ix, ix, ix);
         body = body + bfr;
      }

      maintainOutput.push_back(body);
   }


   // Add body.
   sprintf(bfr, "%d", ((int) maintainOutput.size() - 1));  // Iteration
   body = bfr;

   for (unsigned int ix = 1; ix < maintainStats.size(); ix++) {
      if (maintainStats[ix].hasSamples()) {
         sprintf(bfr, ",%f,%f,%f,%f,%d", maintainStats[ix].getMeanErr(),
                 maintainStats[ix].getStdErr(), maintainStats[ix].getBound95(),
                 maintainStats[ix].getTotMaintain(), maintainStats[ix].getNumCycles());
      } else {
         sprintf(bfr, ",No samples,,,,");
      }

      body = body + bfr;
   }

   maintainOutput.push_back(body);


   // Clear final maintain metrics vectors.

   maintainStats.clear();
}

void InternalObserver::processMaintainMetrics() {

   // Output maintain metrics .csv file.

   string output_file_name = scenario_name + "-Maintain-Metrics.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < maintainOutput.size(); ix++) {
         out << maintainOutput[ix] << endl;
      }

      out.close();
   }


   // Clear report vector.

   maintainOutput.clear();
}

void InternalObserver::updateFinalGS(int id,
                                     double gs) {

   // Stores/replaces final ground speed for a aircraft.
   //
   // id:id of aircraft.
   // gs:ground speed.

   // TODO:Throw exception if id >= finalGS.size().  Also, clarify if the other should
   // be stored, (ie:own id > 0).  The warning that was output was not desirable.

   //if ((id<0) || (id>=finalGS.size()))
   //cout << "WARNING:BAD aircraft id-final ground speed not updated.  id = " << id << endl;
   //else

   if ((id >= 0) && (id < (int) finalGS.size())) {
      finalGS[id] = gs;
   }

}

void InternalObserver::outputFinalGS() {

   // Post processes final ground speed data after each iteration,
   // forming a string for each iteration and placing it in a local
   // string vector.

   string body;
   char bfr[51];


   // Add header.

   if (finalGSOutput.size() == 0) {
      body = "Iteration";

      for (unsigned int ix = 0; ix < finalGS.size(); ix++) {
         sprintf(bfr, ",ac %u-gs", ix);
         body = body + bfr;
      }

      finalGSOutput.push_back(body);
   }


   // Add body.
   sprintf(bfr, "%d", ((int) finalGSOutput.size() - 1)); // Iteration // TODO:A better way of determining iteration.
   body = bfr;

   for (size_t ix = 0; ix < finalGS.size(); ix++) {
      sprintf(bfr, ",%f", finalGS[ix]);
      body = body + bfr;
   }


   // Add string to output.

   finalGSOutput.push_back(body);


   // Clear final GS stats vector.

   finalGS.clear();
}

void InternalObserver::processFinalGS() {

   // Outputs the final groundspeed .csv file.

   // Open file

   string output_file_name = scenario_name + "-Final-Groundspeed.csv";
   ofstream out;
   out.open(output_file_name.c_str());


   if (out.is_open()) {
      for (size_t ix = 0; ix < finalGSOutput.size(); ix++) {
         out << finalGSOutput[ix] << endl;
      }

      out.close();
   }


   // Clear report vector.

   finalGSOutput.clear();
}

void InternalObserver::outputMergePointMetric() {

   // Creates report for merge point metric, first a column header
   // and then for each iteration, one line with merge point stats.
   // Each line is a string stored in an output vector.

   string body;
   char bfr[61];

   // Add header.

   if (mergePointOutput.size() == 0) {
      body = "Iteration";

      for (unsigned int ix = 1; ix < mergePointStats.size(); ix++) {
         sprintf(bfr, ",ac %d-mergePt,ac %d-distTo ac %d", ix, ix, (ix - 1));
         body = body + bfr;
      }

      mergePointOutput.push_back(body);
   }


   // Add body.
   sprintf(bfr, "%d", ((int) mergePointOutput.size() - 1));  // Iteration
   body = bfr;

   for (size_t ix = 1; ix < mergePointStats.size(); ix++) {
      sprintf(bfr, ",%s,%f", mergePointStats[ix].getMergePoint().c_str(),
              Units::NauticalMilesLength(mergePointStats[ix].getDist()).value());
      body = body + bfr;
   }

   mergePointOutput.push_back(body);


   // Clear merge point vector.

   mergePointStats.clear();

}

void InternalObserver::processMergePointMetric() {

   // Output merge point metric to a .csv file.

   string output_file_name = scenario_name + "-Merge-Point-Metric.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < mergePointOutput.size(); ix++) {
         out << mergePointOutput[ix] << endl;
      }

      out.close();
   }


   // Clear report vector.

   mergePointOutput.clear();
}

void InternalObserver::outputClosestPointMetric() {

   // Creates report text for the closest point metric.
   // A column header is created the first time through.
   // A line containing the closest point metric stats is
   // created for each iteration.


   string body;
   char bfr[61];

   // Add header.

   if (closestPointOutput.size() == 0) {
      body = "Iteration";

      for (unsigned int ix = 1; ix < closestPointStats.size(); ix++) {
         sprintf(bfr, ",ac %u-smallestDistTo ac %u", ix, (ix - 1));
         body = body + bfr;
      }

      closestPointOutput.push_back(body);
   }


   // Add body.
   sprintf(bfr, "%d", ((int) closestPointOutput.size() - 1));  // Iteration
   body = bfr;

   for (size_t ix = 1; ix < closestPointStats.size(); ix++) {
      sprintf(bfr, ",%f",
              Units::NauticalMilesLength(closestPointStats[ix].getMinDist()).value());
      body = body + bfr;
   }

   closestPointOutput.push_back(body);


   // Clear closest point vector.

   closestPointStats.clear();

}

void InternalObserver::processClosestPointMetric() {

   // Output closest point metric to a .csv file.

   string output_file_name = scenario_name + "-Closest-Point-Metric.csv";
   ofstream out;
   out.open(output_file_name.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < closestPointOutput.size(); ix++) {
         out << closestPointOutput[ix] << endl;
      }

      out.close();
   }


   // Clear report vector.

   closestPointOutput.clear();
}

// collect pTIS_B reports into this class
void InternalObserver::collect_ptis_b_report(Sensor::ADSB::ADSBSVReport adsb_sv_report) {
   ptis_b_report_list.push_back(adsb_sv_report);
}

void InternalObserver::process_ptis_b_reports() // process the ADS-B reports
{
   //Figure the maximum id out of all the IDs of  in the receiver_id field in ptis_b_ether_with_receiver_ID_list
   int max_id = -100;
   //loop through the receiver_id fields in ptis_b_ether_with_receiver_ID_list
   for (size_t i = 0; i < ptis_b_report_list.size(); i++) {
      int this_id = ptis_b_report_list[i].getId();
      if (this_id > max_id) {
         max_id = this_id;
      }
   }

   //loop through all ac ids
   for (int ac_id = 0; ac_id <= max_id; ac_id++) {

      // open report data file
      std::ostringstream ostr_ac_id; //output string stream
      ostr_ac_id << ac_id; //use the string stream to convert ac_id into an output string stream
      string ac_id_string = ostr_ac_id.str(); //convert to string

      string output_file_name = scenario_name + "-TIS-B-Report-output-TargetACID-" + ac_id_string + ".csv";
      ofstream out;
      out.open(output_file_name.c_str());

      //if file opens successfully, process the output
      if (out.is_open() == true && ptis_b_report_list.empty() == false) {
         //print the header
         out << "TOA,24bitAddress,Lat,Lon,Alt,EWVel,NSVel,NACp,NIC,NACv,SIL,SDA,VertRate" << endl;

         for (size_t i = 0; i < ptis_b_report_list.size(); i++) {
            Sensor::ADSB::ADSBSVReport return_report;
            return_report = ptis_b_report_list[i];
            if (return_report.getId() == ac_id) {
               // print out current record
               out << return_report.getTime().value() << ","; // outputs the TOA
               out << return_report.getId() << ","; // output  id
               Units::DegreesAngle lat_out, long_out;
               StereographicProjection::xy_to_ll(
                     Units::FeetLength(return_report.getX()),
                     Units::FeetLength(return_report.getY()),
                     lat_out, long_out); // call the Stereographic Projection to convert the aircraft X/Y to Lat/Long
               out.precision(10);
               out << lat_out.value() << ","; // output the  lat in degrees
               out << long_out.value() << ","; // output lon in degrees
               out << return_report.getZ().value() << ","; // output the current altitude value in feet
               out << Units::KnotsSpeed(return_report.getXd()).value() <<
                   ","; // output the current x velocity in knots; the unit of return_report.getxd is assumed to be feet/second
               out << Units::KnotsSpeed(return_report.getYd()).value() <<
                   ","; // output the current y velocity in knots; the unit of return_report.getyd is assumed to be feet/second
               out << return_report.getNacp() << ","; // output the NACp
               out << return_report.getNicp() << ","; // output the NICp
               out << return_report.getNacv() << ","; // output the NACv
               out << 2 << ","; // output the SIL (set at 2)
               out << 2 << ","; // output the SDA (set at 2)
               out << Units::FeetPerMinuteSpeed(return_report.getZd()).value() <<
                   endl; // output the current vertical velocity feet per minute; the unit of return_report.zd is assumed to be feet/second
            }    //end if(return_report.id == ac_id)
         } //end for(int i = 0; i <  ptis_b_ether_with_receiver_ID_list.size(); i++)
         out.close();
      } //end if( out.is_open() == true && ads_b_ether_list.empty() == false)
   } //end for(int ac_id = 0; ac_id <= max_id; ac_id++)


}

void InternalObserver::addPredictedWind(int id,
                                        const WindStack &predWindX,
                                        const WindStack &predWindY) {
   // Adds predicted wind entry for an aircraft.
   //
   // id:aircraft id.
   // predWindX, predWindY:predicted wind data for aircraft.
   //                      altitudes in feet, wind speeds in knots.

   // Add header.
   if (predWinds.size() == 0) {
      predWinds.push_back(predWindsHeading(predWindX.get_max_row()));
   }

   // Add altitudes, x speed, y speed.
   predWinds.push_back(predWindsData(id, 1, "Alt(feet)", predWindX));
   predWinds.push_back(predWindsData(id, 2, "XSpeed(Knots)", predWindX));
   predWinds.push_back(predWindsData(id, 2, "YSpeed(Knots)", predWindY));
}

string InternalObserver::predWindsHeading(int numVals) {
   // Formats header for predicted winds metric.
   //
   // numVals:number of values in the predicted winds matrices where
   //         each value is an altitude, speed pair.
   //
   // returns header line.

   string hdr = "Aircraft_id,Field";

   // Only aircraft id for now.  This will need clarification from Lesley.
   // Still need to add a blank column title for each column.

   for (int i = 1; i <= numVals; i++) {
      hdr += ",";
   }

   return hdr;
}

string InternalObserver::predWindsData(int id,
                                       int col,
                                       string field,
                                       const WindStack &mat) {
   // Formats data line for predicted winds metric for an aircraft
   // for a data row.  With respect between the data line output and
   // how the wind matrices are setup, the rows and columns are
   // inverted.  Altitudes output in meters, speeds in meters/second.
   //
   // id:aircraft id.
   // field:field name.
   // col:col of data being formatted-1 for altitude, 2 for speed.
   // mat:matrix containing data to format into string.
   //
   // returns data line.

   string str;

   char *txt = new char[31];


   // Aircraft id

   sprintf(txt, "%d", id);
   str = txt;


   // Field

   str += ",";
   str += field.c_str();


   // Data line-all in meters, meters/second.

   for (int i = 1; i <= mat.get_max_row(); i++) {
      switch (col) {
         case 1:
            sprintf(txt, ",%lf", mat.getAltitude(i).value());
            break;
         case 2:
            sprintf(txt, ",%lf", mat.getSpeed(i).value());

      }
      str += txt;
   }

   delete[] txt;

   return str;
}

void InternalObserver::dumpPredictedWind() {
   // Outputs predicted winds .csv file.

   string fileName = scenario_name + "-Predicted-Winds.csv";
   ofstream out;
   out.open(fileName.c_str());

   if (out.is_open()) {
      for (size_t ix = 0; ix < predWinds.size(); ix++) {
         out << predWinds[ix] << endl;
      }

      out.close();
   }


   // Clear predicted winds output.
   predWinds.clear();
}

void InternalObserver::addAchieveRcd(size_t aircraftId,
                                     double tm,
                                     double target_ttg_to_ach,
                                     double own_ttg_to_ach,
                                     double curr_distance,
                                     double reference_distance) {
   // Adds data record for achieve algorithms.
   //
   // aircraftId:aircraft id.
   // tm:time (seconds).
   // target_ttg_to_ach:target time to go to achieve (seconds).
   // own_ttg_to_ach:own time to go to achieve (seconds).
   // curr_distance:current distance (meters).
   // reference_distance:reference distance (meters).

   while (aircraftId >= mAchieveList.size()) {
      vector<AchieveObserver> newachievelist;
      mAchieveList.push_back(newachievelist);
   }

   AchieveObserver achievercd(this->scenario_iter, aircraftId, tm, target_ttg_to_ach, own_ttg_to_ach,
                              curr_distance, reference_distance);
   mAchieveList[aircraftId].push_back(achievercd);
}


void InternalObserver::dumpAchieveList() {
   // Outputs output from achieve algorithms to time to go .csv file.

   string fileName = scenario_name + "-time-to-go.csv";
   ofstream out;
   out.open(fileName.c_str());

   bool needHdr = true;

   if (out.is_open()) {
      for (size_t acIx = 0; acIx < mAchieveList.size(); acIx++) {
         if (mAchieveList[acIx].size() == 0) {
            continue;
         } // Nada for this ac

         // Header
         if (needHdr) {
            out << mAchieveList[acIx][0].hdr().c_str() << endl;
            needHdr = false;
         }

         for (size_t ix = 0; ix < mAchieveList[acIx].size(); ix++) {
            out << mAchieveList[acIx][ix].toString().c_str() << endl;
         }
      }
      out.close();
   }


   // Clear sublists.

   for (size_t acIx = 0; acIx < mAchieveList.size(); acIx++) {
      mAchieveList[acIx].clear();
   }
}


void InternalObserver::dumpOwnKinTraj(int id,
                                      VerticalPath &fullTraj) {
   // Dumps own kinematic trajectory after ensuring file initialized.
   //
   // id:own aircarft id.
   // fullTraj:own aircraft kinematic trajectory.


   // Check setup

   if (mOwnKinVertPathObs == NULL) {
      mOwnKinVertPathObs = new VerticalPathObserver(scenario_name,
                                                    "own_precalc_4D_KinematicTM",
                                                    false);
   }


   // Check iteration

   if (mOwnKinVertPathObs->getIter() != scenario_iter) {
      mOwnKinVertPathObs->setIter(scenario_iter);
   }


   // Write trajectory

   mOwnKinVertPathObs->addTrajectory(id, fullTraj);

}


void InternalObserver::dumpTargetKinTraj(int id,
                                         VerticalPath &fullTraj) {

   // Dumps target kinematic trajectory after ensuring file initialized.
   //
   // id:target aircraft id.
   // fullTraj:target aircraft kinematic trajectory.


   // Check setup

   if (mTargKinVertPathObs == NULL) {
      mTargKinVertPathObs = new VerticalPathObserver(scenario_name,
                                                     "target_precalc_4D_KinematicTM", true);
   }


   // Check iteration

   if (mTargKinVertPathObs->getIter() != scenario_iter) {
      mTargKinVertPathObs->setIter(scenario_iter);
   }


   // Write trajectory

   mTargKinVertPathObs->addTrajectory(id, fullTraj);

}


void InternalObserver::writeTrueWind(string str) {

   // Outputs true wind .csv file.

   string fileName = scenario_name + "-debug-true-wind.csv";
   ofstream out;
   out.open(fileName.c_str(), std::ios_base::app);

   if (out.is_open()) {
      out << "time " << trueTime << ",id " << trueId << endl;
      out << str << endl;
      out.close();
   }

}

void InternalObserver::setTrueWindHdrVals(double time,
                                          int id) {
   trueTime = time;
   trueId = id;
}

bool InternalObserver::debugTrueWind() {
   return this->debuggingTrueWind;
}


void InternalObserver::setNMOutput(bool NMflag) {
   // Sets flag to output NM data.
   //
   // NMFlag:NM output flag.
   //        true-output NM data.
   //        false-don't output NM data.

   this->outputNMFiles = NMflag;
}


bool InternalObserver::outputNM(void) {
   // Determines whether NM data being output or not.
   //
   // returns true if outputting NM data.
   //         false if not outputting NM data.

   return this->outputNMFiles;
}

void InternalObserver::SetRecordMaintainMetrics(bool new_value) {
   m_save_maintain_metrics = new_value;
}

const bool InternalObserver::GetRecordMaintainMetrics() const {
   return m_save_maintain_metrics;
}
