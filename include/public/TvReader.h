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
// Copyright 2020 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

/*
 * TvReader.h
 *
 * Reads a TV.csv file containing a sequence of aircraft states.
 *
 *  Created on: Mar 23, 2019
 *      Author: klewis
 */

#pragma once

#include "public/DataReader.h"
#include "Angle.h"
#include "Length.h"
#include "utility/CustomUnits.h"

namespace testvector {

class TvReader : public DataReader {
public:
   static const size_t EXPECTED_TV_COLUMN_COUNT;
   TvReader(std::string file_name, int header_lines);
   TvReader(std::shared_ptr<std::istream> input_stream, int header_lines);
   TvReader();
   virtual ~TvReader();
   virtual bool Advance();
   const Units::SecondsTime GetTimeOfReceipt() const;
   const int GetAcid() const;
   const Units::SecondsTime GetToap() const;
   const Units::DegreesAngle GetLat() const;
   const Units::DegreesAngle GetLon() const;
   const Units::FeetLength GetAlt() const;
   const Units::KnotsSpeed GetEwvel() const;
   const Units::KnotsSpeed GetNsvel() const;
   const Units::SecondsTime GetToav() const;
   const int GetNacp() const;
   const int GetNic() const;
   const int GetNacv() const;
   const Units::FeetPerMinuteSpeed GetVertRate() const;

private:
   Units::SecondsTime m_time_of_receipt; // column 1
   void SetColumnIndexesFromHeader(const int header_lines);
   int m_aircraft_id_column;
   int m_time_of_applicability_position_column;
   int m_latitude_column;
   int m_longitude_column;
   int m_altitude_column;
   int m_east_velocity_column;
   int m_north_velocity_column;
   int m_time_of_applicability_velocity_column;
   int m_nacp_column;
   int m_nic_column;
   int m_nacv_column;
   int m_vert_rate_column;
};

} // namespace testvector

