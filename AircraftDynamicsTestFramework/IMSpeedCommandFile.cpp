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


IMSpeedCommandFile::IMSpeedCommandFile(void) {

  mLoaded = false;
  mFilePath = "";

  for (int i=0;i<histLen;i++) {
    iasHist[i]=0.0;
  }

  historyIndexer = 0;

}


IMSpeedCommandFile::~IMSpeedCommandFile(void) {}


bool IMSpeedCommandFile::load(DecodedStream *strm) {

  set_stream(strm);

  register_var("imspd_csv_file", &mFilePath, true);
  register_var("use_pilot_delay", &mApplyPilotDelay, true);
  register_var("pilot_delay_seconds", &mPilotDelaySeconds, true);

  mLoaded = complete();

  if (mLoaded) {
    readData();
  }

  return mLoaded;
}


void IMSpeedCommandFile::readData(void) {

  // Parses speed file data and stores data.
  
  std::ifstream file(mFilePath.c_str());

  if (!file.is_open()) {
    std::cout << "Speed file " << mFilePath.c_str() << " not found" << std::endl;
    exit(-46);
  }

  bool hdrrcd = true;

  for (CsvParser::CsvIterator csviter(file);csviter != CsvParser::CsvIterator();++csviter) {

    if (hdrrcd) {
      hdrrcd = false;
      continue;
    }

    SpeedRecord spd;

    mSpeedData.push_back(spd);

    int ix = mSpeedData.size() - 1;

    for (int fieldix = 0;fieldix < (*csviter).size(); ++fieldix) {
      std::string field = (*csviter)[fieldix];

      double val = atof(field.c_str());

      if (fieldix == 0) {
	mSpeedData[ix].mTime = Units::SecondsTime(val);
      } else if (fieldix == 1) {
	mSpeedData[ix].mSpeed = Units::MetersPerSecondSpeed(val);
      } else {
	std::cout << "Extra number of fields found in record "
		  << (ix+1) << " of " << mFilePath.c_str() << std::endl;
	exit(-47);
      }
    }
  }

  file.close();

}


Guidance IMSpeedCommandFile::update(Units::Time time) {

  // update method to return guidance speed at the time or an interpolated
  // speed.  If time before first record's time, the first record's speed is
  // used to set guidance.  If time after last record's time, the last record's 
  // speed is used to set guidance.
  //
  // time: input time
  // returns guidance speed

  Guidance guidance;

  // delay code declarations
  int hardcodeDelay = mPilotDelaySeconds.value();

  guidance.setValid(false);

  if (mSpeedData[0].mTime >= time) {

    // First

    guidance.m_im_speed_command_ias = Units::FeetPerSecondSpeed(mSpeedData[0].mSpeed).value();
    guidance.setValid(true);

  } else if (mSpeedData[(mSpeedData.size()-1)].mTime <= time) {

    // Last

    guidance.m_im_speed_command_ias = Units::FeetPerSecondSpeed(mSpeedData[(mSpeedData.size()-1)].mSpeed).value();
    guidance.setValid(true);


  } else {

    // Somewhere inbetween 

    int ix = 0;

    while ((ix < (mSpeedData.size() - 1)) && (mSpeedData[(ix+1)].mTime < time)) {
      ix++;
    }

    if (mSpeedData[(ix+1)].mTime == time) {

      // Exact match on next

      guidance.m_im_speed_command_ias = Units::FeetPerSecondSpeed(mSpeedData[(ix+1)].mSpeed).value();
      guidance.setValid(true);

    } else {

      // Interpolate

      double pct = Units::SecondsTime(time - mSpeedData[ix].mTime).value() / 
	                  Units::SecondsTime(mSpeedData[(ix+1)].mTime - mSpeedData[ix].mTime).value();

      Units::Speed interpolatedspeed = (1.0-pct) * mSpeedData[ix].mSpeed + pct * mSpeedData[(ix+1)].mSpeed;

      guidance.m_im_speed_command_ias = Units::FeetPerSecondSpeed(interpolatedspeed).value();
      guidance.setValid(true);

    }

  }

  // Delay processing
	if (mApplyPilotDelay) {
		// Update the ias history array
		for (int i = histLen - 1; i > 0; i--) {
			iasHist[i] = iasHist[i - 1];
		}
		iasHist[0] = guidance.m_im_speed_command_ias;

		if (historyIndexer < hardcodeDelay) {
			// protect against not enough history for requested delay
			guidance.m_im_speed_command_ias = iasHist[historyIndexer];
		} else {
			guidance.m_im_speed_command_ias = iasHist[hardcodeDelay];
		}
		historyIndexer++;
	}//end delay processing

  return guidance;

}


std::vector<IMSpeedCommandFile::SpeedRecord> IMSpeedCommandFile::getdata(void) {

  // A Get data method setup to aid unittesting.

  return mSpeedData;
}


void IMSpeedCommandFile::dump(void) {

  std::cout << "Dumping data read from " << mFilePath.c_str() << std::endl << std::endl;
  std::cout << "Number of records " << mSpeedData.size() << std::endl << std::endl;

  for (int ix = 0; ix < mSpeedData.size(); ix++) {
    std::cout << (int)Units::SecondsTime(mSpeedData[ix].mTime).value() << ","
	      << Units::MetersPerSecondSpeed(mSpeedData[ix].mSpeed).value() << std::endl;
  }

}


IMSpeedCommandFile::SpeedRecord::SpeedRecord() {}


IMSpeedCommandFile::SpeedRecord::SpeedRecord(Units::Time t,Units::Speed s) {

  mTime = t;
  mSpeed = s;

}


IMSpeedCommandFile::SpeedRecord::~SpeedRecord() {}


bool IMSpeedCommandFile::SpeedRecord::operator==(const IMSpeedCommandFile::SpeedRecord &sr) const {

  return ((mTime==sr.mTime) && (mSpeed==sr.mSpeed));

}


bool IMSpeedCommandFile::SpeedRecord::operator!=(const IMSpeedCommandFile::SpeedRecord &sr) const {

  return !(*this == sr);

}
