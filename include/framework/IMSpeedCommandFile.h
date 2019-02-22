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

#include "loader/Loadable.h"
#include "public/Guidance.h"
#include <Time.h>
#include <Speed.h>


class IMSpeedCommandFile : public Loadable
{

public:

   class SpeedRecord
   {

   public:

      SpeedRecord();

      SpeedRecord(Units::Time t,
                  Units::Speed s);

      ~SpeedRecord();

      bool operator==(const IMSpeedCommandFile::SpeedRecord &sr) const;

      bool operator!=(const IMSpeedCommandFile::SpeedRecord &sr) const;

      Units::Time mTime;
      Units::Speed mSpeed;
   };

   IMSpeedCommandFile();

   ~IMSpeedCommandFile();

   bool load(DecodedStream *strm);

   Guidance Update(Units::Time time);

   std::vector<IMSpeedCommandFile::SpeedRecord> GetData();

   void dump();


protected:

   std::vector<SpeedRecord> m_speed_data;


private:

   static const int m_hist_len = 20;
   double m_ias_hist[m_hist_len];
   int m_history_indexer;

   void ReadData();

   std::string m_file_path;
   bool m_apply_pilot_delay;
   Units::SecondsTime m_pilot_delay_seconds;

   bool m_loaded;

};
