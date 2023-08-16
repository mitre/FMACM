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

#include "public/TangentPlaneSequence.h"

#include "public/Environment.h"

using namespace std;
log4cplus::Logger TangentPlaneSequence::logger = log4cplus::Logger::getInstance("TangentPlaneSequence");

TangentPlaneSequence::TangentPlaneSequence() {}

TangentPlaneSequence::TangentPlaneSequence(list<Waypoint> &waypoint_list) { Initialize(waypoint_list); }

TangentPlaneSequence::TangentPlaneSequence(const TangentPlaneSequence &in) { copy(in); }

void TangentPlaneSequence::copy(const TangentPlaneSequence &in) {
   this->localPositionsFromInitialization = in.localPositionsFromInitialization;
   this->tangentPlanesFromInitialization = in.tangentPlanesFromInitialization;
   this->waypointsFromInitialization = in.waypointsFromInitialization;
}

void TangentPlaneSequence::Initialize(std::list<Waypoint> &waypoint_list) {
   // Setup the private members that will facilitate future calls to the this class.
   EarthModel *earthModel = Environment::GetInstance()->GetEarthModel();

   shared_ptr<LocalTangentPlane> plane = shared_ptr<LocalTangentPlane>((LocalTangentPlane *)NULL);
   EarthModel::LocalPositionEnu enu;
   enu.x = enu.y = enu.z = Units::MetersLength(0);

   unsigned int ix = waypoint_list.size() - 1;
   waypointsFromInitialization.resize(ix + 1);
   tangentPlanesFromInitialization.resize(ix + 1);
   localPositionsFromInitialization.resize(ix + 1);
   // use reverse iterator so that last will be processed first
   for (list<Waypoint>::reverse_iterator it = waypoint_list.rbegin(); it != waypoint_list.rend(); ++it) {
      // Waypoint *wp = (*it);
      EarthModel::GeodeticPosition geo;
      geo.altitude = Units::MetersLength(0);
      geo.latitude = (*it).GetLatitude();
      geo.longitude = (*it).GetLongitude();

      EarthModel::LocalPositionEnu enu;
      if (plane == NULL) {
         enu.x = enu.y = enu.z = Units::MetersLength(0);
      } else {
         // convert using previous plane
         plane->convertGeodeticToLocal(geo, enu);
      }

      // make the new plane
      plane = earthModel->MakeEnuConverter(geo, enu);

      // update vectors
      tangentPlanesFromInitialization[ix] = plane;
      localPositionsFromInitialization[ix] = enu;
      waypointsFromInitialization[ix] = *it;

      // decrement index
      ix--;
   }
}

void TangentPlaneSequence::convertLocalToGeodetic(EarthModel::LocalPositionEnu localPosition,
                                                  EarthModel::GeodeticPosition &geoPosition) const {

   // For the current localPosition, find the closest waypoint in the initialization information
   int ix = -1;
   Units::Area minD2 = Units::KilometersArea(Units::infinity());
   for (unsigned int ind2 = 0; ind2 < tangentPlanesFromInitialization.size(); ind2++) {
      const EarthModel::LocalPositionEnu &pointOfTangency =
            tangentPlanesFromInitialization[ind2]->getPointOfTangencyEnu();
      Units::Length x = localPosition.x - pointOfTangency.x;
      Units::Length y = localPosition.y - pointOfTangency.y;
      Units::Area d2 = x * x + y * y;
      if (d2 < minD2) {
         minD2 = d2;
         ix = (int)ind2;
      }
   }
   if (ix < 0) {
      LOG4CPLUS_FATAL(logger, "size of tangentPlanesFromInitialization: " << tangentPlanesFromInitialization.size());
      throw logic_error("Unable to determine closest point (empty?)");
   }

   tangentPlanesFromInitialization[ix]->convertLocalToGeodetic(localPosition, geoPosition);
}

void TangentPlaneSequence::convertGeodeticToLocal(EarthModel::GeodeticPosition geoPosition,
                                                  EarthModel::LocalPositionEnu &localPosition) const {

   EarthModel::AbsolutePositionEcef ecefPosition;
   Environment::GetInstance()->GetEarthModel()->ConvertGeodeticToAbsolute(geoPosition, ecefPosition);
   int ix = -1;
   // find the closest waypoint
   Units::Area minD2 = Units::KilometersArea(Units::infinity());
   for (unsigned int ind2 = 0; ind2 < tangentPlanesFromInitialization.size(); ind2++) {
      const EarthModel::AbsolutePositionEcef &pointOfTangency =
            tangentPlanesFromInitialization[ind2]->getPointOfTangencyEcef();
      Units::Length x = ecefPosition.x - pointOfTangency.x;
      Units::Length y = ecefPosition.y - pointOfTangency.y;
      Units::Length z = ecefPosition.z - pointOfTangency.z;
      Units::Area d2 = x * x + y * y + z * z;
      if (d2 < minD2) {
         minD2 = d2;
         ix = (int)ind2;
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

const std::vector<std::shared_ptr<LocalTangentPlane> > &TangentPlaneSequence::getTangentPlanesFromInitialization()
      const {
   return tangentPlanesFromInitialization;
}
