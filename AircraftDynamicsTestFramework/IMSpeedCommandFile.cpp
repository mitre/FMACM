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

#include "framework/IMSpeedCommandFile.h"
#include <fstream>
#include <string>
#include <stdlib.h>
#include "utility/CsvParser.h"


IMSpeedCommandFile::IMSpeedCommandFile() {

   m_loaded = false;
   m_file_path = "";

   for (int i = 0; i < m_hist_len; i++) {
      m_ias_hist[i] = 0.0;
   }

   m_history_indexer = 0;

}


IMSpeedCommandFile::~IMSpeedCommandFile() {
}


bool IMSpeedCommandFile::load(DecodedStream *strm) {

   set_stream(strm);

   register_var("imspd_csv_file", &m_file_path, true);
   register_var("use_pilot_delay", &m_apply_pilot_delay, true);
   register_var("pilot_delay_seconds", &m_pilot_delay_seconds, true);

   m_loaded = complete();

   if (m_loaded) {
      ReadData();
   }

   return m_loaded;
}


void IMSpeedCommandFile::ReadData() {

   // Parses speed file data and stores data.

   std::ifstream file(m_file_path.c_str());

   if (!file.is_open()) {
      std::cout << "Speed file " << m_file_path.c_str() << " not found" << std::endl;
      exit(-46);
   }

   bool hdrrcd = true;

   for (CsvParser::CsvIterator csviter(file); csviter != CsvParser::CsvIterator(); ++csviter) {

      if (hdrrcd) {
         hdrrcd = false;
         continue;
      }

      SpeedRecord spd;

      m_speed_data.push_back(spd);

      int ix = m_speed_data.size() - 1;

      for (int fieldix = 0; fieldix < (*csviter).Size(); ++fieldix) {
         std::string field = (*csviter)[fieldix];

         double val = atof(field.c_str());

         if (fieldix == 0) {
            m_speed_data[ix].mTime = Units::SecondsTime(val);
         } else if (fieldix == 1) {
            m_speed_data[ix].mSpeed = Units::MetersPerSecondSpeed(val);
         } else {
            std::cout << "Extra number of fields found in record "
                      << (ix + 1) << " of " << m_file_path.c_str() << std::endl;
            exit(-47);
         }
      }
   }

   file.close();

}


Guidance IMSpeedCommandFile::Update(Units::Time time) {

   // update method to return guidance speed at the time or an interpolated
   // speed.  If time before first record's time, the first record's speed is
   // used to set guidance.  If time after last record's time, the last record's
   // speed is used to set guidance.
   //
   // time: input time
   // returns guidance speed

   Guidance guidance;

   // delay code declarations
   int hardcodeDelay = m_pilot_delay_seconds.value();

   guidance.setValid(false);

   if (m_speed_data[0].mTime >= time) {

      // First

      guidance.m_im_speed_command_ias = Units::FeetPerSecondSpeed(m_speed_data[0].mSpeed).value();
      guidance.setValid(true);

   } else if (m_speed_data[(m_speed_data.size() - 1)].mTime <= time) {

      // Last

      guidance.m_im_speed_command_ias = Units::FeetPerSecondSpeed(m_speed_data[(m_speed_data.size() - 1)].mSpeed).value();
      guidance.setValid(true);


   } else {

      // Somewhere inbetween

      int ix = 0;

      while ((ix < (m_speed_data.size() - 1)) && (m_speed_data[(ix + 1)].mTime < time)) {
         ix++;
      }

      if (m_speed_data[(ix + 1)].mTime == time) {

         // Exact match on next

         guidance.m_im_speed_command_ias = Units::FeetPerSecondSpeed(m_speed_data[(ix + 1)].mSpeed).value();
         guidance.setValid(true);

      } else {

         // Interpolate

         double pct = Units::SecondsTime(time - m_speed_data[ix].mTime).value() /
                      Units::SecondsTime(m_speed_data[(ix + 1)].mTime - m_speed_data[ix].mTime).value();

         Units::Speed interpolatedspeed = (1.0 - pct) * m_speed_data[ix].mSpeed + pct * m_speed_data[(ix + 1)].mSpeed;

         guidance.m_im_speed_command_ias = Units::FeetPerSecondSpeed(interpolatedspeed).value();
         guidance.setValid(true);

      }

   }

   // Delay processing
   if (m_apply_pilot_delay) {
      // Update the ias history array
      for (int i = m_hist_len - 1; i > 0; i--) {
         m_ias_hist[i] = m_ias_hist[i - 1];
      }
      m_ias_hist[0] = guidance.m_im_speed_command_ias;

      if (m_history_indexer < hardcodeDelay) {
         // protect against not enough history for requested delay
         guidance.m_im_speed_command_ias = m_ias_hist[m_history_indexer];
      } else {
         guidance.m_im_speed_command_ias = m_ias_hist[hardcodeDelay];
      }
      m_history_indexer++;
   }//end delay processing

   return guidance;

}


std::vector<IMSpeedCommandFile::SpeedRecord> IMSpeedCommandFile::GetData() {

   // A Get data method setup to aid unittesting.

   return m_speed_data;
}


void IMSpeedCommandFile::dump() {

   std::cout << "Dumping data read from " << m_file_path.c_str() << std::endl << std::endl;
   std::cout << "Number of records " << m_speed_data.size() << std::endl << std::endl;

   for (int ix = 0; ix < m_speed_data.size(); ix++) {
      std::cout << (int) Units::SecondsTime(m_speed_data[ix].mTime).value() << ","
                << Units::MetersPerSecondSpeed(m_speed_data[ix].mSpeed).value() << std::endl;
   }

}


IMSpeedCommandFile::SpeedRecord::SpeedRecord() {
}


IMSpeedCommandFile::SpeedRecord::SpeedRecord(Units::Time t,
                                             Units::Speed s) {

   mTime = t;
   mSpeed = s;

}


IMSpeedCommandFile::SpeedRecord::~SpeedRecord() {
}


bool IMSpeedCommandFile::SpeedRecord::operator==(const IMSpeedCommandFile::SpeedRecord &sr) const {

   return ((mTime == sr.mTime) && (mSpeed == sr.mSpeed));

}


bool IMSpeedCommandFile::SpeedRecord::operator!=(const IMSpeedCommandFile::SpeedRecord &sr) const {

   return !(*this == sr);

}
