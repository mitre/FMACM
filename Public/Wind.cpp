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
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/Wind.h"

#include <cstring>
#include <sstream>
#include <stdexcept>

#include "public/AircraftCalculations.h"

using std::shared_ptr;
using std::string;
using namespace aaesim::open_source;

void Wind::InterpolateTrueWind(const Units::Angle lat_in, const Units::Angle lon_in, const Units::Length altitude,
                               aaesim::open_source::WindStack &east_west, aaesim::open_source::WindStack &north_south) {
   InterpolateWindMatrix(lat_in, lon_in, altitude, east_west, north_south);
}

void Wind::InterpolateForecastWind(const shared_ptr<TangentPlaneSequence> &tangentPlaneSequence,
                                   const Units::Length x_in, const Units::Length y_in, const Units::Length altitude,
                                   Units::Speed &east_west, Units::Speed &north_south) {
   Units::RadiansAngle lat(0.), lon(0.);

   EarthModel::LocalPositionEnu localPosition;
   localPosition.x = x_in;
   localPosition.y = y_in;
   localPosition.z = altitude;
   EarthModel::GeodeticPosition wpnt;
   tangentPlaneSequence->ConvertLocalToGeodetic(localPosition, wpnt);
   lat = wpnt.latitude;
   lon = wpnt.longitude;

   InterpolateWindScalar(lat, lon, altitude, east_west, north_south);
}
