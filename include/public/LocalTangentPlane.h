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

/*
 * LocalTangentPlane.h
 *
 *  Created on: Jun 25, 2015
 *      Author: klewis
 */

#pragma once

#include "public/EarthModel.h"
#include "public/DMatrix.h"
#include "public/Logging.h"

class LocalTangentPlane {
  public:
   static void printCoordinates(std::string title, Units::Length x, Units::Length y, Units::Length z);

   LocalTangentPlane(const EarthModel *earthModel, const EarthModel::AbsolutePositionEcef &ecefPointOfTangency,
                     const EarthModel::LocalPositionEnu &enuPointOfTangency);

   virtual ~LocalTangentPlane();

   /**
    * Sets the rotation matrices to what they would be
    * at (0N, 0E) in a typical round-earth model.  That is,
    * x = Up, y = East, z = North.
    *
    * The value from the constructor, (x = East, y = North,
    * z = Up), is more useful for flat-earth models.
    */
   void initializeRotationForGeodeticOrigin();

   /**
    * This method rotates the ENU frame.  The new rotation around
    * the vector <x,y,z>, angle theta, is appended to the end of
    * the existing ecefToEnu rotation.  Its inverse is prepended
    * to the beginning of the existing enuToEcef transformation.
    */
   void rotateEnuFrame(const double x, const double y, const double z, const Units::Angle theta);

   void convertGeodeticToAbsolute(const EarthModel::GeodeticPosition &geo,
                                  EarthModel::AbsolutePositionEcef &ecef) const;

   void convertAbsoluteToGeodetic(const EarthModel::AbsolutePositionEcef &ecef,
                                  EarthModel::GeodeticPosition &geo) const;

   void convertLocalToAbsolute(const EarthModel::LocalPositionEnu &enu, EarthModel::AbsolutePositionEcef &ecef) const;

   void convertAbsoluteToLocal(const EarthModel::AbsolutePositionEcef &ecef, EarthModel::LocalPositionEnu &enu) const;

   void convertGeodeticToLocal(const EarthModel::GeodeticPosition &geo, EarthModel::LocalPositionEnu &enu) const;

   void convertLocalToGeodetic(const EarthModel::LocalPositionEnu &enu, EarthModel::GeodeticPosition &geo) const;

   const EarthModel::LocalPositionEnu &getPointOfTangencyEnu() const;

   const EarthModel::AbsolutePositionEcef &getPointOfTangencyEcef() const;

  private:
   static log4cplus::Logger logger;
   /** The model which created us, and which we use for geodetic conversion */
   const EarthModel *earthModel;
   /** The point which maps to pointOfTangencyEnu */
   EarthModel::AbsolutePositionEcef pointOfTangencyEcef;
   EarthModel::LocalPositionEnu pointOfTangencyEnu;
   /** Rotation matrix for (x,y,z)-->(e,n,u) */
   DMatrix ecefToEnu;
   /** Rotation matrix for (e,n,u)-->(x,y,z), the inverse */
   DMatrix enuToEcef;
   const static double identity3x3[3][3];  // = { { 1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};
