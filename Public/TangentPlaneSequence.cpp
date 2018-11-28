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

/*
 * TangentPlaneSequence.cpp
 *
 *  Created on: Jul 5, 2015
 *      Author: klewis
 */

#include "public/TangentPlaneSequence.h"
#include "public/Environment.h"

using namespace std;
log4cplus::Logger TangentPlaneSequence::logger = log4cplus::Logger::getInstance("TangentPlaneSequence");

TangentPlaneSequence::TangentPlaneSequence() {
}

TangentPlaneSequence::TangentPlaneSequence(list<Waypoint> &waypoint_list) {
   initialize(waypoint_list);
}

TangentPlaneSequence::~TangentPlaneSequence() {
}

TangentPlaneSequence::TangentPlaneSequence(const TangentPlaneSequence &in) {
   copy(in);
}

void TangentPlaneSequence::copy(const TangentPlaneSequence &in) {
   this->localPositionsFromInitialization = in.localPositionsFromInitialization;
   this->tangentPlanesFromInitialization = in.tangentPlanesFromInitialization;
   this->waypointsFromInitialization = in.waypointsFromInitialization;
}

void TangentPlaneSequence::initialize(std::list<Waypoint> &waypoint_list) {
   // Setup the private members that will facilitate future calls to the this class.
   EarthModel *earthModel = Environment::getInstance()->getEarthModel();

   shared_ptr<LocalTangentPlane> plane = shared_ptr<LocalTangentPlane>((LocalTangentPlane *) NULL);
   EarthModel::LocalPositionEnu enu;
   enu.x = enu.y = enu.z = Units::MetersLength(0);

   unsigned int ix = waypoint_list.size() - 1;
   waypointsFromInitialization.resize(ix + 1);
   tangentPlanesFromInitialization.resize(ix + 1);
   localPositionsFromInitialization.resize(ix + 1);
   // use reverse iterator so that last will be processed first
   for (list<Waypoint>::reverse_iterator it = waypoint_list.rbegin(); it != waypoint_list.rend(); it++) {
      //Waypoint *wp = (*it);
      EarthModel::GeodeticPosition geo;
      geo.altitude = Units::MetersLength(0);
      geo.latitude = (*it).getLatitude();
      geo.longitude = (*it).getLongitude();

      EarthModel::LocalPositionEnu enu;
      if (plane == NULL) {
         enu.x = enu.y = enu.z = Units::MetersLength(0);
      } else {
         // convert using previous plane
         plane->convertGeodeticToLocal(geo, enu);
      }

      // make the new plane
      plane = earthModel->makeEnuConverter(geo, enu);

      // update vectors
      tangentPlanesFromInitialization[ix] = plane;
      localPositionsFromInitialization[ix] = enu;
      waypointsFromInitialization[ix] = *it;

      // decrement index
      ix--;
   }
}


void TangentPlaneSequence::convertLocalToGeodetic(
      EarthModel::LocalPositionEnu localPosition,
      EarthModel::GeodeticPosition &geoPosition) const {

   // For the current localPostion, find the closest waypoint in the initialization information
   int ix = -1;
   Units::Area minD2 = Units::KilometersArea(1e12);   // just a big number that should not be found on the earth
   for (unsigned int ind2 = 0; ind2 < tangentPlanesFromInitialization.size(); ind2++) {
      const EarthModel::LocalPositionEnu &pointOfTangency = tangentPlanesFromInitialization[ind2]->getPointOfTangencyEnu();
      Units::Length x = localPosition.x - pointOfTangency.x;
      Units::Length y = localPosition.y - pointOfTangency.y;
      Units::Area d2 = x * x + y * y;
      if (d2 < minD2) {
         minD2 = d2;
         ix = (int) ind2;
      }
   }
   if (ix < 0) {
      LOG4CPLUS_FATAL(logger, "size of tangentPlanesFromInitialization: " << tangentPlanesFromInitialization.size());
      throw logic_error("Unable to determine closest point (empty?)");
   }

   tangentPlanesFromInitialization[ix]->convertLocalToGeodetic(localPosition, geoPosition);
}

void TangentPlaneSequence::convertGeodeticToLocal(
      EarthModel::GeodeticPosition geoPosition,
      EarthModel::LocalPositionEnu &localPosition) const {

   EarthModel::AbsolutePositionEcef ecefPosition;
   Environment::getInstance()->getEarthModel()->convertGeodeticToAbsolute(geoPosition, ecefPosition);
   int ix = -1;
   // find the closest waypoint
   Units::Area minD2 = Units::KilometersArea(1e12);   // just a big number that should not be found on the earth
   for (unsigned int ind2 = 0; ind2 < tangentPlanesFromInitialization.size(); ind2++) {
      const EarthModel::AbsolutePositionEcef &pointOfTangency = tangentPlanesFromInitialization[ind2]->getPointOfTangencyEcef();
      Units::Length x = ecefPosition.x - pointOfTangency.x;
      Units::Length y = ecefPosition.y - pointOfTangency.y;
      Units::Length z = ecefPosition.z - pointOfTangency.z;
      Units::Area d2 = x * x + y * y + z * z;
      if (d2 < minD2) {
         minD2 = d2;
         ix = (int) ind2;
      }
   }
   if (ix < 0) {
      LOG4CPLUS_FATAL(logger, "size of tangentPlanesFromInitialization: " << tangentPlanesFromInitialization.size());
      throw logic_error("Unable to determine closest point (empty?)");
   }

   tangentPlanesFromInitialization[ix]->convertAbsoluteToLocal(ecefPosition, localPosition);
}

const std::vector<EarthModel::LocalPositionEnu> &TangentPlaneSequence::getLocalPositionsFromInitialization() const {
   return localPositionsFromInitialization;
}

const std::vector<Waypoint> &TangentPlaneSequence::getWaypointsFromInitialization() const {
   return waypointsFromInitialization;
}

const std::vector<std::shared_ptr<LocalTangentPlane> > &
TangentPlaneSequence::getTangentPlanesFromInitialization() const {
   return tangentPlanesFromInitialization;
}
