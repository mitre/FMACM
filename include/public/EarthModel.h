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
 * EarthModel.h
 *
 *  Created on: Jun 25, 2015
 *      Author: klewis
 */

#pragma once

#include "Length.h"
#include "SignedAngle.h"
#include <memory>

// can't include LocalTangentPlane.h here because of mutual dependency
class LocalTangentPlane;

/**
 * EarthModel is a base class for a singleton that converts between
 * geodetic and ECEF coordinate systems.  It also acts as a factory
 * for EnuConverter, which converts between ECEF and ENU coordinates.
 */
class EarthModel
{
public:
   class GeodeticPosition
   {
   public:
      Units::SignedAngle latitude, longitude;
      Units::Length altitude;
   };

   class AbsolutePositionEcef
   {
   public:
      Units::Length x, y, z;
   };

   class LocalPositionEnu
   {
   public:
      Units::Length x, y, z;
   };

   virtual ~EarthModel();

   virtual void convertGeodeticToAbsolute(
         const EarthModel::GeodeticPosition &geo,
         EarthModel::AbsolutePositionEcef &ecef) const = 0;

   virtual void convertAbsoluteToGeodetic(
         const EarthModel::AbsolutePositionEcef &ecef,
         EarthModel::GeodeticPosition &geo) const = 0;

   virtual std::shared_ptr<LocalTangentPlane> makeEnuConverter(
         const GeodeticPosition &pointOfTangencyGeo,
         const LocalPositionEnu &pointOfTangencyEnu) const = 0;

protected:
   EarthModel();
};

std::ostream& operator<<(std::ostream &out, const EarthModel::GeodeticPosition &geo);
std::ostream& operator<<(std::ostream &out, const EarthModel::LocalPositionEnu &local);
