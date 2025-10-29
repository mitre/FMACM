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

TangentPlaneSequence::TangentPlaneSequence() {}

TangentPlaneSequence::TangentPlaneSequence(list<Waypoint> &waypoint_list) { Initialize(waypoint_list); }

TangentPlaneSequence::TangentPlaneSequence(const TangentPlaneSequence &in) { Copy(in); }

void TangentPlaneSequence::Copy(const TangentPlaneSequence &in) {
   this->local_positions_from_initialization_ = in.local_positions_from_initialization_;
   this->tangent_planes_from_initialization_ = in.tangent_planes_from_initialization_;
   this->waypoints_from_initialization_ = in.waypoints_from_initialization_;
}

void TangentPlaneSequence::Initialize(const std::list<Waypoint> &waypoint_list) {
   shared_ptr<LocalTangentPlane> tangent_plane = shared_ptr<LocalTangentPlane>((LocalTangentPlane *)NULL);
   EarthModel::LocalPositionEnu enu;
   enu.x = enu.y = enu.z = Units::zero();

   auto vector_idx = waypoint_list.size() - 1;
   waypoints_from_initialization_.resize(vector_idx + 1);
   tangent_planes_from_initialization_.resize(vector_idx + 1);
   local_positions_from_initialization_.resize(vector_idx + 1);
   auto build_tangent_planes = [this, &tangent_plane, &vector_idx](const Waypoint &waypoint) {
      EarthModel::GeodeticPosition geo;
      geo.altitude = Units::zero();
      geo.latitude = waypoint.GetLatitude();
      geo.longitude = waypoint.GetLongitude();

      EarthModel::LocalPositionEnu enu;
      if (tangent_plane == NULL) {
         enu.x = enu.y = enu.z = Units::zero();
      } else {
         // convert using previous tangent_plane
         tangent_plane->ConvertGeodeticToLocal(geo, enu);
      }

      // make the new tangent plane
      tangent_plane = Environment::GetInstance()->GetEarthModel()->MakeEnuConverter(geo, enu);

      // update vectors
      tangent_planes_from_initialization_[vector_idx] = tangent_plane;
      local_positions_from_initialization_[vector_idx] = enu;
      waypoints_from_initialization_[vector_idx] = waypoint;

      // decrement index
      vector_idx--;
   };
   // use reverse iterator so that last will be processed first
   std::for_each(waypoint_list.rbegin(), waypoint_list.rend(), build_tangent_planes);
}

void TangentPlaneSequence::ConvertLocalToGeodetic(EarthModel::LocalPositionEnu local_position,
                                                  EarthModel::GeodeticPosition &geo_position) const {
   std::vector<Units::Area> areas;
   auto compute_metric = [&areas, local_position](const EarthModel::LocalPositionEnu &point_of_tangency) {
      Units::Length x = local_position.x - point_of_tangency.x;
      Units::Length y = local_position.y - point_of_tangency.y;
      Units::Area d2 = x * x + y * y;
      areas.push_back(d2);
   };
   auto get_point_of_tangency = [&compute_metric](const std::shared_ptr<LocalTangentPlane> &tangent_plane) {
      compute_metric(tangent_plane->getPointOfTangencyEnu());
   };
   std::for_each(tangent_planes_from_initialization_.begin(), tangent_planes_from_initialization_.end(),
                 get_point_of_tangency);
   auto minimum = std::min_element(areas.begin(), areas.end());
   if (minimum == areas.end()) {
      LOG4CPLUS_FATAL(logger_,
                      "size of tangent_planes_from_initialization_: " << tangent_planes_from_initialization_.size());
      throw logic_error("Unable to determine closest point (empty?)");
   }
   auto closest_tangent_plane_itr = std::distance(areas.begin(), minimum);
   tangent_planes_from_initialization_[closest_tangent_plane_itr]->ConvertLocalToGeodetic(local_position, geo_position);
}

void TangentPlaneSequence::ConvertGeodeticToLocal(EarthModel::GeodeticPosition geo_position,
                                                  EarthModel::LocalPositionEnu &local_position) const {
   EarthModel::AbsolutePositionEcef ecef_position;
   Environment::GetInstance()->GetEarthModel()->ConvertGeodeticToAbsolute(geo_position, ecef_position);
   std::vector<Units::Area> areas;
   auto compute_metric = [&areas, ecef_position](const EarthModel::AbsolutePositionEcef &point_of_tangency) {
      Units::Length x = ecef_position.x - point_of_tangency.x;
      Units::Length y = ecef_position.y - point_of_tangency.y;
      Units::Length z = ecef_position.z - point_of_tangency.z;
      Units::Area d2 = x * x + y * y + z * z;
      areas.push_back(d2);
   };
   auto get_point_of_tangency = [&compute_metric](const std::shared_ptr<LocalTangentPlane> &tangent_plane) {
      compute_metric(tangent_plane->getPointOfTangencyEcef());
   };
   std::for_each(tangent_planes_from_initialization_.begin(), tangent_planes_from_initialization_.end(),
                 get_point_of_tangency);
   auto minimum = std::min_element(areas.begin(), areas.end());
   if (minimum == areas.end()) {
      LOG4CPLUS_FATAL(logger_,
                      "size of tangent_planes_from_initialization_: " << tangent_planes_from_initialization_.size());
      throw logic_error("Unable to determine closest point (empty?)");
   }
   auto closest_tangent_plane_itr = std::distance(areas.begin(), minimum);
   tangent_planes_from_initialization_[closest_tangent_plane_itr]->ConvertAbsoluteToLocal(ecef_position,
                                                                                          local_position);
}

const std::vector<EarthModel::LocalPositionEnu> &TangentPlaneSequence::GetLocalPositionsFromInitialization() const {
   return local_positions_from_initialization_;
}

const std::vector<Waypoint> &TangentPlaneSequence::GetWaypointsFromInitialization() const {
   return waypoints_from_initialization_;
}

const std::vector<std::shared_ptr<LocalTangentPlane> > &TangentPlaneSequence::GetTangentPlanesFromInitialization()
      const {
   return tangent_planes_from_initialization_;
}
