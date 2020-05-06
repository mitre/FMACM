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
 * EarthModel.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: klewis
 */

#include "public/EarthModel.h"

EarthModel::EarthModel() {
}

EarthModel::~EarthModel() {
}

std::ostream& operator <<(std::ostream& out,
      const EarthModel::GeodeticPosition& geo) {
   out << "(" << Units::DegreesAngle(geo.latitude) << "," << Units::DegreesAngle(geo.longitude) << ")";
   return out;
}

std::ostream& operator <<(std::ostream& out,
      const EarthModel::LocalPositionEnu& local) {
   out << "(" << Units::MetersLength(local.x) <<
         "," << Units::MetersLength(local.y) <<
         "," << Units::MetersLength(local.z) << ")";
   return out;
}
