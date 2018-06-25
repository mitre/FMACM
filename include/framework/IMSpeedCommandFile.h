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

#include "loader/Loadable.h"
#include "public/Guidance.h"
#include <Time.h>
#include <Speed.h>


class IMSpeedCommandFile : public Loadable
{

 public:

  class SpeedRecord {

    public:

      SpeedRecord();
      SpeedRecord(Units::Time t,Units::Speed s);
      ~SpeedRecord();

      bool operator==(const IMSpeedCommandFile::SpeedRecord &sr) const;
      bool operator!=(const IMSpeedCommandFile::SpeedRecord &sr) const;

      Units::Time mTime;
      Units::Speed mSpeed;
  };

  IMSpeedCommandFile(void);
  ~IMSpeedCommandFile(void);

  bool load(DecodedStream *strm);

  Guidance update(Units::Time time);

  std::vector<IMSpeedCommandFile::SpeedRecord> getdata(void);

  void dump(void);


 protected:

  std::vector<SpeedRecord> mSpeedData;


 private:

  static const int histLen = 20;
  double iasHist[histLen];
  int historyIndexer;

  void readData(void);

  std::string mFilePath;
  bool mApplyPilotDelay;
  Units::SecondsTime mPilotDelaySeconds;

  bool mLoaded;

};
