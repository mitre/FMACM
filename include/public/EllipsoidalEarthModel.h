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

/*
 * EllipsoidalEarthModel.h
 *
 *  Created on: Jun 30, 2015
 *      Author: klewis
 */

#pragma once

#include "public/EarthModel.h"
#include "Area.h"
#include "public/LocalTangentPlane.h"
#include "utility/Logging.h"

class EllipsoidalEarthModel : public EarthModel
{
public:
   static const Units::Length mWGS84SemiMajorRadius;
   static const double mWGS84EccentricitySquared;

   EllipsoidalEarthModel();

   virtual ~EllipsoidalEarthModel();

   virtual void convertGeodeticToAbsolute(
         const EarthModel::GeodeticPosition &geo,
         EarthModel::AbsolutePositionEcef &ecef) const;

   virtual void convertAbsoluteToGeodetic(
         const EarthModel::AbsolutePositionEcef &ecef,
         EarthModel::GeodeticPosition &geo) const;

   virtual std::shared_ptr<LocalTangentPlane> makeEnuConverter(const GeodeticPosition &pointOfTangencyGeo,
                                                               const LocalPositionEnu &pointOfTangencyEnu) const;

private:
   static log4cplus::Logger logger;
   const Units::Length semiMajorRadius;
   const Units::Area semiMajorRadius2;
   const double eccentricity2;
   const double eccentricity4;
};

