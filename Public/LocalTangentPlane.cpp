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
 * LocalTangentPlane.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: klewis
 */

#include "public/LocalTangentPlane.h"
#include "public/CustomMath.h"

using namespace std;

log4cplus::Logger LocalTangentPlane::logger = log4cplus::Logger::getInstance(LOG4CPLUS_TEXT("LocalTangentPlane"));
const double LocalTangentPlane::identity3x3[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
const double yzx[3][3] = {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}};
const double zxy[3][3] = {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}};

LocalTangentPlane::LocalTangentPlane(const EarthModel *earthModel,
                                     const EarthModel::AbsolutePositionEcef &ecefPointOfTangency,
                                     const EarthModel::LocalPositionEnu &enuPointOfTangency)
   : earthModel(earthModel),
     pointOfTangencyEcef(ecefPointOfTangency),
     pointOfTangencyEnu(enuPointOfTangency),
     m_ecef_to_enu(DMatrix((double **)&identity3x3, 0, 2, 0, 2)),
     m_enu_to_ecef(DMatrix((double **)&identity3x3, 0, 2, 0, 2)) {}

LocalTangentPlane::~LocalTangentPlane() {}

void LocalTangentPlane::InitializeRotationForGeodeticOrigin() {
   m_ecef_to_enu = DMatrix((double **)zxy, 0, 2, 0, 2);
   m_enu_to_ecef = DMatrix((double **)yzx, 0, 2, 0, 2);
}

void LocalTangentPlane::RotateEnuFrame(const double x, const double y, const double z, const Units::Angle theta) {
   DMatrix &r = CreateRotationMatrix(x, y, z, theta);
   DMatrix &ecefToEnu1 = m_ecef_to_enu * r;
   m_ecef_to_enu = ecefToEnu1;
   delete &r;
   delete &ecefToEnu1;

   DMatrix &rInv = CreateRotationMatrix(x, y, z, -theta);
   DMatrix &enuToEcef1 = rInv * m_enu_to_ecef;
   m_enu_to_ecef = enuToEcef1;
   delete &rInv;
   delete &enuToEcef1;
}

void LocalTangentPlane::ConvertGeodeticToAbsolute(const EarthModel::GeodeticPosition &geo,
                                                  EarthModel::AbsolutePositionEcef &ecef) const {
   earthModel->ConvertGeodeticToAbsolute(geo, ecef);
}

void LocalTangentPlane::ConvertAbsoluteToGeodetic(const EarthModel::AbsolutePositionEcef &ecef,
                                                  EarthModel::GeodeticPosition &geo) const {
   earthModel->ConvertAbsoluteToGeodetic(ecef, geo);
}

void LocalTangentPlane::printCoordinates(string title, Units::Length x, Units::Length y, Units::Length z) {
   LOG4CPLUS_TRACE(logger, title << " (" << Units::MetersLength(x).value() << "," << Units::MetersLength(y).value()
                                 << "," << Units::MetersLength(z).value() << ")");
}

/**
 * Converts local coordinates to absolute.
 */
void LocalTangentPlane::ConvertLocalToAbsolute(const EarthModel::LocalPositionEnu &enu,
                                               EarthModel::AbsolutePositionEcef &ecef) const {
   Units::Length x1 = enu.x - pointOfTangencyEnu.x;
   Units::Length y1 = enu.y - pointOfTangencyEnu.y;
   Units::Length z1 = enu.z - pointOfTangencyEnu.z;
   printCoordinates("Local relative: ", x1, y1, z1);

   ecef.x = m_enu_to_ecef[0][0] * x1 + m_enu_to_ecef[0][1] * y1 + m_enu_to_ecef[0][2] * z1;
   ecef.y = m_enu_to_ecef[1][0] * x1 + m_enu_to_ecef[1][1] * y1 + m_enu_to_ecef[1][2] * z1;
   ecef.z = m_enu_to_ecef[2][0] * x1 + m_enu_to_ecef[2][1] * y1 + m_enu_to_ecef[2][2] * z1;

   printCoordinates("Rotated relative: ", ecef.x, ecef.y, ecef.z);

   ecef.x += pointOfTangencyEcef.x;
   ecef.y += pointOfTangencyEcef.y;
   ecef.z += pointOfTangencyEcef.z;

   printCoordinates("Absolute: ", ecef.x, ecef.y, ecef.z);
}

/**
 * Converts absolute coordinates to local.
 */
void LocalTangentPlane::ConvertAbsoluteToLocal(const EarthModel::AbsolutePositionEcef &ecef,
                                               EarthModel::LocalPositionEnu &enu) const {
   Units::Length x1 = ecef.x - pointOfTangencyEcef.x;
   Units::Length y1 = ecef.y - pointOfTangencyEcef.y;
   Units::Length z1 = ecef.z - pointOfTangencyEcef.z;

   enu.x = m_ecef_to_enu[0][0] * x1 + m_ecef_to_enu[0][1] * y1 + m_ecef_to_enu[0][2] * z1;
   enu.y = m_ecef_to_enu[1][0] * x1 + m_ecef_to_enu[1][1] * y1 + m_ecef_to_enu[1][2] * z1;
   enu.z = m_ecef_to_enu[2][0] * x1 + m_ecef_to_enu[2][1] * y1 + m_ecef_to_enu[2][2] * z1;

   enu.x += pointOfTangencyEnu.x;
   enu.y += pointOfTangencyEnu.y;
   enu.z += pointOfTangencyEnu.z;
}

void LocalTangentPlane::ConvertGeodeticToLocal(const EarthModel::GeodeticPosition &geo,
                                               EarthModel::LocalPositionEnu &enu) const {
   EarthModel::AbsolutePositionEcef temp;
   earthModel->ConvertGeodeticToAbsolute(geo, temp);
   ConvertAbsoluteToLocal(temp, enu);
}

void LocalTangentPlane::ConvertLocalToGeodetic(const EarthModel::LocalPositionEnu &enu,
                                               EarthModel::GeodeticPosition &geo) const {
   EarthModel::AbsolutePositionEcef temp;
   ConvertLocalToAbsolute(enu, temp);
   earthModel->ConvertAbsoluteToGeodetic(temp, geo);
}

const EarthModel::LocalPositionEnu &LocalTangentPlane::getPointOfTangencyEnu() const { return pointOfTangencyEnu; }

const EarthModel::AbsolutePositionEcef &LocalTangentPlane::getPointOfTangencyEcef() const {
   return pointOfTangencyEcef;
}
