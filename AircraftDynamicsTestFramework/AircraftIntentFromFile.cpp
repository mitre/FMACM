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

#include "framework/AircraftIntentFromFile.h"

#include "utility/CsvParser.h"
#include <fstream>

using std::string;

AircraftIntentFromFile::AircraftIntentFromFile() = default;

AircraftIntentFromFile::~AircraftIntentFromFile() = default;

bool AircraftIntentFromFile::load(DecodedStream *input) {
   set_stream(input);

   std::string csvfile;
   register_var("hfp_csv_file", &csvfile, true);

   bool loaded = complete();

   if (loaded) {
      PopulateWaypointsFromCsv(csvfile);
   }

   return loaded;
}

void AircraftIntentFromFile::PopulateWaypointsFromCsv(const std::string &csvfile) {

   std::ifstream file(csvfile.c_str());
   if (!file.is_open()) {
      std::cout << "Error: Unable to open file " << csvfile << std::endl;
      exit(-1);
   }

   /*
    * Notes on Loading:
    * - note that the CSV file contains the waypoint information in reverse order from the way it is needed here
    * - The first two rows may be skipped as header information
    * - There is no way to know how many rows exist. Just read until the buffer is empty
    * - Data structured like this:
    *       HPT_j, x[m], y[m], DTG[m], Segment Type, Course[rad], Turn Center x[m], Turn Center y[m], Angle Start of Turn[rad], Angle End of Turn[rad], R[m], Lat[deg], Lon[deg], Turn Center Lat[deg], Turn Center Lon[deg]
    *       1,  0.00,  0.00,  0.00,straight, 0.7918,  0.00,  0.00,  0.00,  0.00,  0.00,33.4286000,-111.8130000,0,0,
    * - The course column contains 10000000.0000 when the value is irrelevant
    * - The lat/lon pairs are in WGS-84 space
    */
   Units::MetersLength xwptlocation[128], ywptlocation[128], disttogo[128], xturncenter[128], yturncenter[128], turnradius[128];
   string segmenttype[128];
   Units::RadiansAngle course[128], startofturn[128], endofturn[128];
   Units::DegreesAngle latwpt[128], lonwpt[128], latturncenter[128], lonturncenter[128];
   int irow = -1;
   for (CsvParser::CsvIterator csvrow(file); csvrow != CsvParser::CsvIterator(); ++csvrow) {
      irow++;
      if (irow < 2) {
         continue;
      }

      int rowindex = 0;
      for (int rowElement = 0; rowElement < (*csvrow).Size(); ++rowElement) {
         string s = (*csvrow)[rowElement];
         switch (rowElement) {
            case 0:
               rowindex = LocalStringToInt(s) - 1; // the value extracted from the CSV file is 1-based, we need 0-based
               break;
            case 1:
               xwptlocation[rowindex] = Units::MetersLength(LocalStringToDouble(s));
               break;
            case 2:
               ywptlocation[rowindex] = Units::MetersLength(LocalStringToDouble(s));
               break;
            case 3:
               disttogo[rowindex] = Units::MetersLength(LocalStringToDouble(s));
               break;
            case 4:
               segmenttype[rowindex] = s;
               break;
            case 5:
               course[rowindex] = Units::RadiansAngle(LocalStringToDouble(s));
               break;
            case 6:
               xturncenter[rowindex] = Units::MetersLength(LocalStringToDouble(s));
               break;
            case 7:
               yturncenter[rowindex] = Units::MetersLength(LocalStringToDouble(s));
               break;
            case 8:
               startofturn[rowindex] = Units::RadiansAngle(LocalStringToDouble(s));
               break;
            case 9:
               endofturn[rowindex] = Units::RadiansAngle(LocalStringToDouble(s));
               break;
            case 10:
               turnradius[rowindex] = Units::MetersLength(LocalStringToDouble(s));
               break;
            case 11:
               latwpt[rowindex] = Units::DegreesAngle(LocalStringToDouble(s));
               break;
            case 12:
               lonwpt[rowindex] = Units::DegreesAngle(LocalStringToDouble(s));
               break;
            case 13:
               latturncenter[rowindex] = Units::DegreesAngle(LocalStringToDouble(s));
               break;
            case 14:
               lonturncenter[rowindex] = Units::DegreesAngle(LocalStringToDouble(s));
               break;
            default:
               break;
         }
      }
   }

   // Now store all information into the public Fms struct in the appropriate order
   int numberofrowsofdata = irow - 1, forwardrow = 0;
   SetId(0); // hardcoding in this test framework, must match the id in TestFrameworkAircraft.cpp
   SetNumberOfWaypoints(numberofrowsofdata);
   for (int reverserow = numberofrowsofdata - 1; reverserow >= 0; --reverserow) {
      m_fms.m_latitude[forwardrow] = latwpt[reverserow];
      m_fms.m_longitude[forwardrow] = lonwpt[reverserow];
      m_fms.m_x[forwardrow] = xwptlocation[reverserow];
      m_fms.m_y[forwardrow] = ywptlocation[reverserow];
      forwardrow++;
   }

}

double AircraftIntentFromFile::LocalStringToDouble(const string &s) {
   std::istringstream iss(s);
   double val = 0;
   iss >> val;
   return val;
}

int AircraftIntentFromFile::LocalStringToInt(const string &s) {
   std::istringstream iss(s);
   int val = 0;
   iss >> val;
   return val;
}

void AircraftIntentFromFile::update_xy_from_latlon() {
   // intentionally empty to prevent parent method from running
}
