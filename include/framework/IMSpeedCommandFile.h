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

#include "loader/Loadable.h"
#include "public/Guidance.h"
#include <scalar/Time.h>
#include <scalar/Speed.h>

class IMSpeedCommandFile : public Loadable {

  public:
   class SpeedRecord {

     public:
      SpeedRecord();

      SpeedRecord(Units::Time t, Units::Speed s);

      virtual ~SpeedRecord();

      bool operator==(const IMSpeedCommandFile::SpeedRecord &sr) const;

      bool operator!=(const IMSpeedCommandFile::SpeedRecord &sr) const;

      Units::Time mTime;
      Units::Speed mSpeed;
   };

   IMSpeedCommandFile();

   virtual ~IMSpeedCommandFile();

   bool load(DecodedStream *strm);

   aaesim::open_source::Guidance Update(Units::Time time);

   void dump();

  protected:
   std::vector<SpeedRecord> m_speed_data;

  private:
   void ReadData();

   static const int m_hist_len = 20;
   double m_ias_hist[m_hist_len];

   std::string m_file_path;
   Units::SecondsTime m_pilot_delay_seconds;

   bool m_apply_pilot_delay;
   bool m_loaded;
};
