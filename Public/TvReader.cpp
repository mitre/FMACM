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

/*
 * TvReader.cpp
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#include "public/TvReader.h"
#include <stdexcept>
#include <map>

using namespace std;

namespace aaesim {
namespace open_source {

const size_t TvReader::EXPECTED_TV_COLUMN_COUNT(17);

TvReader::TvReader(std::string file_name, int header_lines) : DataReader(file_name, 0, 0) {
   SetColumnIndexesFromHeader(header_lines);
}

TvReader::TvReader(std::shared_ptr<std::istream> input_stream, int header_lines) : DataReader(input_stream, 0, 0) {
   SetColumnIndexesFromHeader(header_lines);
}

TvReader::~TvReader() {}

bool TvReader::Advance() {
   bool result = DataReader::Advance();
   if (result) {
      m_time_of_receipt = Units::SecondsTime(GetDouble(0));
   } else {
      m_time_of_receipt = DataReader::UNDEFINED_TIME;
   }
   return result;
}

const Units::SecondsTime TvReader::GetTimeOfReceipt() const { return m_time_of_receipt; }

const int TvReader::GetAcid() const { return GetDouble(m_aircraft_id_column); }

const Units::SecondsTime TvReader::GetToap() const {
   return Units::SecondsTime(GetDouble(m_time_of_applicability_position_column));
}

const Units::DegreesAngle TvReader::GetLat() const { return Units::DegreesAngle(GetDouble(m_latitude_column)); }

const Units::DegreesAngle TvReader::GetLon() const { return Units::DegreesAngle(GetDouble(m_longitude_column)); }

const Units::FeetLength TvReader::GetAlt() const { return Units::FeetLength(GetDouble(m_altitude_column)); }

const Units::KnotsSpeed TvReader::GetEwvel() const { return Units::KnotsSpeed(GetDouble(m_east_velocity_column)); }

const Units::KnotsSpeed TvReader::GetNsvel() const { return Units::KnotsSpeed(GetDouble(m_north_velocity_column)); }

const Units::SecondsTime TvReader::GetToav() const {
   return Units::SecondsTime(GetDouble(m_time_of_applicability_velocity_column));
}

const int TvReader::GetNacp() const { return GetDouble(m_nacp_column); }

const int TvReader::GetNic() const { return GetDouble(m_nic_column); }

const int TvReader::GetNacv() const { return GetDouble(m_nacv_column); }

const Units::FeetPerMinuteSpeed TvReader::GetVertRate() const {
   return Units::FeetPerMinuteSpeed(GetDouble(m_vert_rate_column));
}

void TvReader::SetColumnIndexesFromHeader(const int header_lines) {
   if (header_lines < 1) {
      throw logic_error("Headers are required for TvReader.");
   }
   Advance();  // first header line
   BuildColumnIndex();

   // Record required column indexes.
   // This will insert an entry if index is missing.
   // tRec[sec] -- hardcoded to column 0
   m_aircraft_id_column = GetColumnNumber("ACID");
   // TargetType -- not used
   m_time_of_applicability_position_column = GetColumnNumber("TOAp[sec]");
   m_latitude_column = GetColumnNumber("Lat[degrees]");
   m_longitude_column = GetColumnNumber("Lon[degrees]");
   m_altitude_column = GetColumnNumber("Alt[feet]");
   m_east_velocity_column = GetColumnNumber("EWVel[knots]");
   m_north_velocity_column = GetColumnNumber("NSVel[knots]");
   m_time_of_applicability_velocity_column = GetColumnNumber("TOAv[sec]");
   m_nacp_column = GetColumnNumber("NACp");
   m_nic_column = GetColumnNumber("NIC");
   m_nacv_column = GetColumnNumber("NACv");
   // SIL -- Source Integrity Level, not used
   // SDA -- System Design Assurance, not used
   m_vert_rate_column = GetColumnNumber("VertRate[fpm]");

   SetExpectedColumnCount(GetColumnCount());

   SkipLines(header_lines - 1);
}

}  // namespace open_source
}  // namespace aaesim
