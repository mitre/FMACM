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
 * EllipsoidalEarthModel.h
 *
 *  Created on: Jun 30, 2015
 *      Author: klewis
 */

#pragma once

#include "public/EarthModel.h"
#include <scalar/Area.h>
#include "public/LocalTangentPlane.h"
#include "utility/Logging.h"

class EllipsoidalEarthModel : public EarthModel
{
public:
   static const Units::Length m_wgs84_semimajor_radius;
   static const double m_wgs84_eccentricity_squared;

   EllipsoidalEarthModel();

   virtual ~EllipsoidalEarthModel();

   virtual void ConvertGeodeticToAbsolute(
         const EarthModel::GeodeticPosition &geo,
         EarthModel::AbsolutePositionEcef &ecef) const override;

   virtual void ConvertAbsoluteToGeodetic(
         const EarthModel::AbsolutePositionEcef &ecef,
         EarthModel::GeodeticPosition &geo) const override;

   virtual std::shared_ptr<LocalTangentPlane> MakeEnuConverter(const GeodeticPosition &pointOfTangencyGeo,
                                                               const LocalPositionEnu &pointOfTangencyEnu) const override;

private:
   static log4cplus::Logger m_logger;
   const Units::Length m_semi_major_radius;
   const Units::Area m_semi_major_radius_squared;
   const double m_eccentricity_squared;
   const double m_eccentricity_4;
};

