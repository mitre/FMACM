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
 * EarthModel.h
 *
 *  Created on: Jun 25, 2015
 *      Author: klewis
 */

#pragma once

#include <scalar/Length.h>
#include <scalar/SignedAngle.h>
#include "public/Waypoint.h"
#include "utility/CustomUnits.h"
#include <memory>

// can't include LocalTangentPlane.h here because of mutual dependency
class LocalTangentPlane;

/**
 * EarthModel is a base class for a singleton that converts between
 * geodetic and ECEF coordinate systems.  It also acts as a factory
 * for EnuConverter, which converts between ECEF and ENU coordinates.
 */
class EarthModel {
  public:
   class GeodeticPosition {
     public:
      Units::SignedAngle latitude, longitude;
      Units::Length altitude;
      static EarthModel::GeodeticPosition CreateFromWaypoint(const Waypoint &waypoint) {
         EarthModel::GeodeticPosition return_this;
         return_this.altitude = Units::zero();
         return_this.latitude = waypoint.GetLatitude();
         return_this.longitude = waypoint.GetLongitude();
         return return_this;
      }
   };

   class AbsolutePositionEcef {
     public:
      Units::MetersLength x, y, z;
      AbsolutePositionEcef ToUnitVector() const {
         AbsolutePositionEcef unit_vector;
         const Units::MetersLength vector_mag = Units::sqrt(Units::sqr(x) + Units::sqr(y) + Units::sqr(z));
         unit_vector.x = Units::MetersLength(x / vector_mag);
         unit_vector.y = Units::MetersLength(y / vector_mag);
         unit_vector.z = Units::MetersLength(z / vector_mag);
         return unit_vector;
      }
   };

   class LocalPositionEnu {
     public:
      static LocalPositionEnu Of(Units::Length x, Units::Length y, Units::Length z);
      static LocalPositionEnu OfZeros();
      Units::Length x, y, z;
   };

   virtual ~EarthModel();

   virtual void ConvertGeodeticToAbsolute(const EarthModel::GeodeticPosition &geo,
                                          EarthModel::AbsolutePositionEcef &ecef) const = 0;

   virtual void ConvertAbsoluteToGeodetic(const EarthModel::AbsolutePositionEcef &ecef,
                                          EarthModel::GeodeticPosition &geo) const = 0;

   virtual std::shared_ptr<LocalTangentPlane> MakeEnuConverter(const GeodeticPosition &pointOfTangencyGeo,
                                                               const LocalPositionEnu &pointOfTangencyEnu) const = 0;

  protected:
   EarthModel();
};

std::ostream &operator<<(std::ostream &out, const EarthModel::GeodeticPosition &geo);
std::ostream &operator<<(std::ostream &out, const EarthModel::LocalPositionEnu &local);

inline Units::Length VectorDotProduct(const EarthModel::AbsolutePositionEcef &a,
                                      const EarthModel::AbsolutePositionEcef &b) {
   Units::MetersLength dp_result = a.x * b.x.value() + a.y * b.y.value() + a.z * b.z.value();
   return dp_result;
}

inline EarthModel::AbsolutePositionEcef VectorCrossProduct(const EarthModel::AbsolutePositionEcef &a,
                                                           const EarthModel::AbsolutePositionEcef &b) {
   EarthModel::AbsolutePositionEcef cp_result;
   cp_result.x = a.y * b.z.value() - a.z * b.y.value();
   cp_result.y = a.z * b.x.value() - a.x * b.z.value();
   cp_result.z = a.x * b.y.value() - a.y * b.x.value();
   return cp_result;
}

inline EarthModel::LocalPositionEnu EarthModel::LocalPositionEnu::Of(Units::Length x, Units::Length y,
                                                                     Units::Length z) {
   LocalPositionEnu lpe;
   lpe.x = x;
   lpe.y = y;
   lpe.z = z;
   return lpe;
}

inline EarthModel::LocalPositionEnu EarthModel::LocalPositionEnu::OfZeros() {
   LocalPositionEnu lpe;
   lpe.x = Units::zero();
   lpe.y = Units::zero();
   lpe.z = Units::zero();
   return lpe;
}
